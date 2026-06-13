#!/usr/bin/env python3
"""Fusion primitives for board-side autonomous parking.

This module is intentionally pure-stdlib and side-effect free. It is the B1/B2
foundation from the fusion plan:

- parse STM32 protocol lines (ACK/TLM/DONE/ERR/STAT)
- load C0 chassis sign configuration
- propagate a minimal fused slot-frame pose from TLM odometry + yaw

It does not open serial ports, sockets, camera, or actuator paths.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import re
from pathlib import Path
from typing import Any


REQUIRED_SIGN_KEYS = (
    "yaw_cw_positive",
    "odom_d_reverse_negative",
    "odom_x_right_positive",
    "vision_lateral_left_negative",
)


def _to_number(value: str) -> Any:
    try:
        if re.fullmatch(r"[-+]?\d+", value):
            return int(value)
        if re.fullmatch(r"[-+]?\d+(?:\.\d+)?", value):
            return float(value)
    except TypeError:
        pass
    return value


def _parse_kv(tokens: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for tok in tokens:
        if "=" not in tok:
            continue
        key, value = tok.split("=", 1)
        if key:
            out[key.lower()] = _to_number(value)
    return out


def parse_stm32_line(line: str) -> dict[str, Any]:
    """Parse one STM32 text line into a structured event.

    Unknown or malformed lines are returned as {"type": "raw", "raw": ...}.
    """
    raw = line.strip().replace("\r", "")
    if not raw:
        return {"type": "empty", "raw": raw}
    parts = raw.split()
    head = parts[0].upper()

    if head == "ACK" and len(parts) >= 3:
        return {"type": "ack", "seq": _to_number(parts[1]), "cmd": parts[2].upper(), "raw": raw}

    if head == "TLM" and len(parts) >= 2:
        event = {"type": "tlm", "n": _to_number(parts[1]), "raw": raw}
        event.update(_parse_kv(parts[2:]))
        return event

    if head == "DONE" and len(parts) >= 3:
        event = {"type": "done", "seq": _to_number(parts[1]), "cmd": parts[2].upper(), "raw": raw}
        event.update(_parse_kv(parts[3:]))
        return event

    if head == "ERR" and len(parts) >= 2:
        event = {"type": "err", "seq": _to_number(parts[1]), "raw": raw}
        event.update(_parse_kv(parts[2:]))
        return event

    if head == "STAT" and len(parts) >= 2:
        event = {"type": "stat", "seq": _to_number(parts[1]), "raw": raw}
        event.update(_parse_kv(parts[2:]))
        return event

    if head == "VER" and len(parts) >= 2:
        event = {"type": "ver", "seq": _to_number(parts[1]), "raw": raw}
        event.update(_parse_kv(parts[2:]))
        return event

    if head == "GDIAG":
        event = {"type": "gdiag", "raw": raw}
        event.update(_parse_kv(parts[1:]))
        return event

    return {"type": "raw", "raw": raw}


def parse_stm32_text(text: str) -> list[dict[str, Any]]:
    return [parse_stm32_line(line) for line in text.splitlines() if line.strip()]


@dataclass(frozen=True)
class ChassisSigns:
    yaw_cw_positive: bool
    odom_d_reverse_negative: bool
    odom_x_right_positive: bool
    vision_lateral_left_negative: bool
    source: str = ""

    @property
    def reverse_odom_d_sign(self) -> float:
        # The fused slot frame treats reverse progress toward the slot as positive.
        return -1.0 if self.odom_d_reverse_negative else 1.0

    @property
    def yaw_to_cw_sign(self) -> float:
        # Internal fused phi uses clockwise-positive heading changes.
        return 1.0 if self.yaw_cw_positive else -1.0


def load_chassis_signs(path: str | Path) -> ChassisSigns:
    p = Path(path)
    data = json.loads(p.read_text(encoding="utf-8"))
    missing = [key for key in REQUIRED_SIGN_KEYS if data.get(key) is None]
    if missing:
        raise ValueError("chassis signs are incomplete: %s" % ", ".join(missing))
    values = {}
    for key in REQUIRED_SIGN_KEYS:
        if not isinstance(data.get(key), bool):
            raise ValueError("chassis sign %s must be boolean, got %r" % (key, data.get(key)))
        values[key] = bool(data[key])
    return ChassisSigns(source=str(p), **values)


def wrap_degrees(delta: float) -> float:
    while delta > 180.0:
        delta -= 360.0
    while delta <= -180.0:
        delta += 360.0
    return delta


def vision_anchor_from_slot_state(slot_state: dict[str, Any]) -> dict[str, float]:
    """Convert slot_relative_state into the plan's slot-frame pose.

    Slot-frame convention used here:
    - x_s_cm: lateral error, positive left
    - y_s_cm: rear axle position outside the slot entrance, negative before entry
    - phi_deg: heading error, clockwise positive after C0 sign normalization
    """
    ground = slot_state.get("ground_estimate") or slot_state
    image = slot_state.get("image") or slot_state
    lateral = float(ground["slot_lateral_cm"])
    y_dist = float(ground["slot_y_dist_cm"])
    heading = float(
        ground.get("slot_axis_heading_deg")
        if ground.get("slot_axis_heading_deg") is not None
        else image.get("slot_heading_err_deg", 0.0)
    )
    return {"x_s_cm": lateral, "y_s_cm": -y_dist, "phi_deg": heading}


@dataclass
class PoseFuser:
    signs: ChassisSigns
    x_s_cm: float = 0.0
    y_s_cm: float = 0.0
    phi_deg: float = 0.0
    source: str = "uninitialized"
    tlm_count: int = 0
    last_yaw_deg: float | None = None
    last_d_cm: float | None = None
    last_tlm_n: int | None = None
    anomaly_count: int = 0
    innovation: dict[str, float] = field(default_factory=lambda: {"x_cm": 0.0, "y_cm": 0.0, "phi_deg": 0.0})

    def anchor_vision(self, slot_state: dict[str, Any]) -> dict[str, Any]:
        anchor = vision_anchor_from_slot_state(slot_state)
        self.x_s_cm = anchor["x_s_cm"]
        self.y_s_cm = anchor["y_s_cm"]
        self.phi_deg = anchor["phi_deg"]
        self.source = "vision_anchor"
        self.tlm_count = 0
        self.last_yaw_deg = None
        self.last_d_cm = None
        self.last_tlm_n = None
        self.anomaly_count = 0
        self.innovation = {"x_cm": 0.0, "y_cm": 0.0, "phi_deg": 0.0}
        return self.snapshot()

    def ingest_tlm(self, tlm: dict[str, Any]) -> dict[str, Any]:
        if tlm.get("type") != "tlm":
            raise ValueError("ingest_tlm expects a parsed TLM event")
        yaw = float(tlm["yaw"])
        d = float(tlm["d"])
        n = int(tlm.get("n", 0))
        if self.last_yaw_deg is None or self.last_d_cm is None:
            self.last_yaw_deg = yaw
            self.last_d_cm = d
            self.last_tlm_n = n
            self.tlm_count += 1
            self.source = "dead_reckon"
            return self.snapshot()

        raw_d_delta = d - self.last_d_cm
        raw_yaw_delta = wrap_degrees(yaw - self.last_yaw_deg)
        ds = self.signs.reverse_odom_d_sign * raw_d_delta
        dphi = self.signs.yaw_to_cw_sign * raw_yaw_delta
        phi0 = self.phi_deg
        phi_mid = math.radians(phi0 + dphi * 0.5)
        self.x_s_cm += ds * math.sin(phi_mid)
        self.y_s_cm += ds * math.cos(phi_mid)
        self.phi_deg = wrap_degrees(self.phi_deg + dphi)
        self.last_yaw_deg = yaw
        self.last_d_cm = d
        self.last_tlm_n = n
        self.tlm_count += 1
        self.source = "dead_reckon"
        return self.snapshot(extra={"ds_cm": ds, "dphi_deg": dphi})

    def blend_vision(self, slot_state: dict[str, Any], alpha: float = 0.3,
                     max_xy_cm: float = 5.0, max_phi_deg: float = 6.0) -> dict[str, Any]:
        anchor = vision_anchor_from_slot_state(slot_state)
        dx = anchor["x_s_cm"] - self.x_s_cm
        dy = anchor["y_s_cm"] - self.y_s_cm
        dphi = wrap_degrees(anchor["phi_deg"] - self.phi_deg)
        self.innovation = {"x_cm": dx, "y_cm": dy, "phi_deg": dphi}
        if abs(dx) <= max_xy_cm and abs(dy) <= max_xy_cm and abs(dphi) <= max_phi_deg:
            self.x_s_cm += alpha * dx
            self.y_s_cm += alpha * dy
            self.phi_deg = wrap_degrees(self.phi_deg + alpha * dphi)
            self.source = "blended"
        else:
            self.anomaly_count += 1
        return self.snapshot()

    def snapshot(self, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        out = {
            "schema": "fused_pose.v1",
            "x_s_cm": round(self.x_s_cm, 3),
            "y_s_cm": round(self.y_s_cm, 3),
            "phi_deg": round(self.phi_deg, 3),
            "source": self.source,
            "tlm_count": self.tlm_count,
            "last_tlm_n": self.last_tlm_n,
            "innovation": {k: round(v, 3) for k, v in self.innovation.items()},
            "anomaly_count": self.anomaly_count,
        }
        if extra:
            out.update(extra)
        return out


def _main() -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--signs", default="configs/chassis_signs.json")
    ap.add_argument("--parse-line", action="append", default=[])
    args = ap.parse_args()
    signs = load_chassis_signs(args.signs)
    print(json.dumps({"signs": signs.__dict__}, ensure_ascii=False, indent=2))
    for line in args.parse_line:
        print(json.dumps(parse_stm32_line(line), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
