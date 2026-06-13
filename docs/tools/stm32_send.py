#!/usr/bin/env python3
"""Send ONE STM32 V2 command to the board over SSH and print the response.

This is the deliberate downlink path: planner candidate command -> board ->
STM32 serial (/dev/ttyUSB0). It is invoked once per command (no daemon, no
auto-loop), so per-step operator confirmation stays in control.

Safety:
- Read-only verbs (PING / VER / STAT) and the safety verb STOP run freely.
- Motion verbs (MOVE / ARC) require --allow-motion, otherwise only a preview is
  printed and nothing is sent.
- D / STE / V are validated and out-of-range values are REJECTED (not silently
  clamped), so a bad command never reaches the vehicle.

The board-side reader logic is embedded below and uploaded fresh each run (LF
guaranteed) to avoid CRLF / quoting problems with busybox sh.
"""

from __future__ import annotations

import argparse
import base64
import re
import sys
import time

import paramiko


BOARD_SCRIPT = """#!/bin/sh
set -u
CMD="${1:-STAT}"
READ_SEC="${2:-6}"
SEQ="${3:-1}"
TTY=/dev/ttyUSB0
VID=1a86
PID=7523
INIT=/opt/parking/stm32_uart/ch341_user_init
node=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ -f "$d/idProduct" ] || continue
  if [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ]; then
    node=$(basename "$d")
    break
  fi
done
[ -n "$node" ] || { echo "STM32_SEND=FAIL reason=no_ch341_usb"; exit 2; }
[ -e "$TTY" ] || { echo "STM32_SEND=FAIL reason=no_tty"; exit 3; }
BUS=$(cat "/sys/bus/usb/devices/$node/busnum")
DEV=$(cat "/sys/bus/usb/devices/$node/devnum")
USBDEV=$(printf "/dev/bus/usb/%03d/%03d" "$BUS" "$DEV")
[ -x "$INIT" ] && "$INIT" "$USBDEV" >/dev/null 2>&1
stty -F "$TTY" 9600 cs8 -cstopb -parenb -ixon -ixoff -crtscts -hupcl clocal cread raw -echo min 0 time 1
e=$(( $(date +%s) + 1 ))
while [ "$(date +%s)" -lt "$e" ]; do dd if="$TTY" bs=1 count=256 2>/dev/null >/dev/null; done
printf '@%s %s\\r' "$SEQ" "$CMD" > "$TTY"
echo "STM32_SENT=@$SEQ $CMD"
OUT=/tmp/stm32_send_resp.bin
: > "$OUT"
e=$(( $(date +%s) + $READ_SEC ))
while [ "$(date +%s)" -lt "$e" ]; do
  dd if="$TTY" bs=1 count=256 2>/dev/null >> "$OUT"
  if grep -Eq '(^|\r|\n)(DONE|ERR) ' "$OUT" 2>/dev/null; then
    sleep 0.2
    dd if="$TTY" bs=1 count=256 2>/dev/null >> "$OUT"
    break
  fi
done
echo "STM32_RESP_BEGIN"
tr -cd '\\11\\12\\15\\40-\\176' < "$OUT"
echo
echo "STM32_RESP_END"
"""

REMOTE_SCRIPT = "/tmp/board_stm32_send.sh"

READ_ONLY_VERBS = {
    "PING",
    "VER",
    "STAT",
    "TEL",
    "GET",
    "ZERO_ODOM",
    "ZERO_YAW",
    "ZERO_ALL",
    "GDIAG",
    "GYROCAL",
}
MOTION_VERBS = {"MOVE", "ARC"}


def validate_command(cmd: str, max_abs_d: float, max_gear: int) -> tuple[str, bool, str | None]:
    """Return (verb, is_motion, error). error is None when the command is valid."""
    cmd = cmd.strip()
    if not cmd:
        return ("", False, "empty command")
    verb = cmd.split()[0].upper()

    if verb in {"PING", "VER", "STAT", "STOP", "GDIAG", "GYROCAL"}:
        if cmd.upper() != verb:
            return (verb, False, f"{verb} takes no arguments")
        return (verb, False, None)

    if verb == "TEL":
        if cmd.upper() not in {"TEL ON", "TEL OFF", "TEL 1", "TEL 0"}:
            return (verb, False, "TEL must look like 'TEL ON' or 'TEL OFF'")
        return (verb, False, None)

    if verb == "GET":
        if not re.fullmatch(r"GET PARAM=[A-Z_]+", cmd.upper()):
            return (verb, False, "GET must look like 'GET PARAM=<name>'")
        return (verb, False, None)

    if verb in {"ZERO_ODOM", "ZERO_YAW", "ZERO_ALL"}:
        if cmd.upper() != verb:
            return (verb, False, f"{verb} takes no arguments")
        return (verb, False, None)

    if verb == "MOVE":
        m = re.fullmatch(r"MOVE D=(-?\d+(?:\.\d+)?) V=(\d+)", cmd)
        if not m:
            return (verb, True, "MOVE must look like 'MOVE D=<cm> V=<gear>'")
        d = float(m.group(1))
        v = int(m.group(2))
        if abs(d) > max_abs_d:
            return (verb, True, f"|D|={abs(d)} exceeds max_abs_d={max_abs_d}")
        if not (0 <= v <= max_gear):
            return (verb, True, f"V={v} out of range [0,{max_gear}]")
        return (verb, True, None)

    if verb == "ARC":
        m = re.fullmatch(r"ARC D=(-?\d+(?:\.\d+)?) STE=(\d+) V=(\d+)", cmd)
        if not m:
            return (verb, True, "ARC must look like 'ARC D=<cm> STE=<servo> V=<gear>'")
        d = float(m.group(1))
        ste = int(m.group(2))
        v = int(m.group(3))
        if abs(d) > max_abs_d:
            return (verb, True, f"|D|={abs(d)} exceeds max_abs_d={max_abs_d}")
        if not (45 <= ste <= 135):
            return (verb, True, f"STE={ste} out of safe servo range [45,135]")
        if not (0 <= v <= max_gear):
            return (verb, True, f"V={v} out of range [0,{max_gear}]")
        return (verb, True, None)

    return (verb, False, f"unknown V2 verb '{verb}' (allowed: PING/VER/STAT/TEL/GET/ZERO_*/GDIAG/GYROCAL/STOP/MOVE/ARC)")


def connect(host: str, user: str, password: str) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password, timeout=20,
                   banner_timeout=20, auth_timeout=20)
    return client


def run(client: paramiko.SSHClient, command: str, timeout: float) -> tuple[int, str, str]:
    _stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


def upload_script(client: paramiko.SSHClient) -> None:
    # base64 over exec (board has no working SFTP); LF preserved from our string.
    data = BOARD_SCRIPT.replace("\r\n", "\n").encode("utf-8")
    b64 = base64.b64encode(data).decode("ascii")
    rc, _out, err = run(client, f"echo '{b64}' | base64 -d > {REMOTE_SCRIPT}", 20)
    if rc != 0:
        raise RuntimeError(f"failed to upload board script: {err.strip()}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="172.20.10.2", help="Board IP (iPhone hotspot default).")
    parser.add_argument("--user", default="root")
    parser.add_argument("--password", default="ebaina")
    parser.add_argument("--cmd", required=True, help='V2 command, e.g. "STAT" or "MOVE D=-7 V=1" or "ARC D=-7 STE=93 V=1"')
    parser.add_argument("--allow-motion", action="store_true", help="Required to actually send MOVE/ARC.")
    parser.add_argument("--read-sec", type=int, default=0, help="Response read window seconds (0 = auto).")
    parser.add_argument("--seq", type=int, default=0, help="V2 sequence number (0 = auto).")
    parser.add_argument("--max-abs-d", type=float, default=30.0, help="Reject |D| above this (cm command units).")
    parser.add_argument("--max-gear", type=int, default=3)
    args = parser.parse_args()

    verb, is_motion, error = validate_command(args.cmd, args.max_abs_d, args.max_gear)
    if error:
        print(f"STM32_SEND_REJECTED reason={error}", file=sys.stderr)
        return 2

    if is_motion and not args.allow_motion:
        print("This is a MOTION command and needs explicit approval before it is sent.")
        print()
        print(f"  Command : @<seq> {args.cmd}")
        print(f"  Board   : {args.user}@{args.host}:{REMOTE_SCRIPT} -> /dev/ttyUSB0 (STM32 V2)")
        print(f"  Verb    : {verb} (closed-loop, auto-stops at DONE)")
        print("  Purpose : send one planner/operator step to the vehicle")
        print("  Risk    : the vehicle WILL move. Ensure clear space and that you can stop it.")
        print("  Stop    : stm32_send.py --cmd STOP ; or physically hold/lift the car.")
        print()
        print("Re-run with --allow-motion only after the move is confirmed safe.")
        return 4

    read_sec = args.read_sec or (8 if is_motion else 4)
    seq = args.seq or (int(time.time()) % 9000 + 1000)

    client = connect(args.host, args.user, args.password)
    try:
        upload_script(client)
        rc, out, err = run(client, f"sh {REMOTE_SCRIPT} '{args.cmd}' {read_sec} {seq}", read_sec + 30)
    finally:
        client.close()

    if out:
        print(out, end="" if out.endswith("\n") else "\n")
    if err.strip():
        print(err, end="", file=sys.stderr)

    # Concise parse of the STM32 response lines.
    resp_lines = [ln.strip() for ln in out.splitlines()
                  if any(k in ln for k in ("ACK", "DONE", "STAT", "PONG", "FW=", "STM32_SEND=FAIL"))]
    print("---")
    if resp_lines:
        print("STM32_PARSED:")
        for ln in resp_lines:
            print(f"  {ln}")
    else:
        print("STM32_PARSED: (no recognizable ACK/DONE/STAT in window)")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
