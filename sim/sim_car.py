from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path


REMOTE_TIMEOUT_MS = 2000
DISTANCE_TIMEOUT_MS = 30000
TURN_TIMEOUT_MS = 8000
AUTO_DEFAULT_SPEED = 2
AUTO_FORWARD1_CM = 100.0
AUTO_FORWARD2_CM = 60.0
AUTO_TURN_DEG = 90.0
DISTANCE_DONE_CM = 2.0
TURN_DONE_DEG = 3.0
TURN_SERVO_MAX_OFFSET = 35.0
TURN_SERVO_KP = 0.75
SPEEDSTEP = 120
RSPEEDSTEP = 15


class Mode:
    IDLE = "IDLE"
    MANUAL = "MANUAL"
    STRAIGHT = "STRAIGHT"
    DISTANCE = "DISTANCE"
    TURN_YAW = "TURN_YAW"
    AUTO_ROUTE = "AUTO_ROUTE"


class AutoStep:
    IDLE = "IDLE"
    FORWARD1 = "FORWARD1"
    TURN1 = "TURN1"
    FORWARD2 = "FORWARD2"


class RunState:
    STANDBY = "STANDBY"
    PARKING = "PARKING"
    HITTED = "HITTED"


@dataclass
class HeadingPID:
    kp: float = 2.5
    ki: float = 0.01
    kd: float = 0.18
    max_i: float = 5.0
    max_out: float = 8.0
    deadband: float = 2.0
    d_alpha: float = 0.7
    smooth_alpha: float = 0.4
    cross_track_kp: float = 2.0
    cross_track_enable: bool = True
    integral: float = 0.0
    last_error: float = 0.0
    dv: float = 0.0
    smoothed_angle: float = 90.0
    first_run: bool = True

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.dv = 0.0
        self.smoothed_angle = 90.0
        self.first_run = True
        self.cross_track_enable = True


@dataclass
class Odom:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    distance: float = 0.0

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.distance = 0.0


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_yaw(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw


def parse_command_value(text: str, scaled_hundredths: bool = False) -> float | None:
    text = text.strip(" =:")
    match = re.fullmatch(r"([+-]?)(\d+)(?:\.(\d+))?", text)
    if not match:
        return None

    sign = -1.0 if match.group(1) == "-" else 1.0
    whole = float(match.group(2))
    fraction_text = match.group(3)
    if fraction_text is None:
        value = whole
        if scaled_hundredths:
            value *= 0.01
    else:
        value = whole + float(f"0.{fraction_text}")
    return sign * value


class CarSim:
    def __init__(self) -> None:
        self.is_up = 1
        self.is_pause = True
        self.is_turn = False
        self.is_straight = False
        self.state = RunState.STANDBY
        self.angle = 90.0
        self.new_yaw = 0.0
        self.org_yaw = 0.0
        self.gyro_z = 0.0
        self.speed_rank = 0
        self.control_ticks = 0
        self.last_command_tick = 0
        self.action_start_tick = 0
        self.target_distance_cm = 0.0
        self.target_yaw = 0.0
        self.auto_speed_level = AUTO_DEFAULT_SPEED
        self.mode = Mode.IDLE
        self.auto_step = AutoStep.IDLE
        self.pid = HeadingPID()
        self.odom = Odom()
        self.responses: list[str] = []

    def reply(self, text: str) -> None:
        self.responses.append(text)

    def refresh_watchdog(self) -> None:
        self.last_command_tick = self.control_ticks

    def center_steering(self) -> None:
        self.angle = 90.0

    def hard_stop_motion(self) -> None:
        self.speed_rank = 0
        self.center_steering()
        self.is_straight = False
        self.is_turn = False
        self.auto_step = AutoStep.IDLE
        self.mode = Mode.IDLE

    def set_standby_mode(self) -> None:
        self.hard_stop_motion()
        self.state = RunState.STANDBY

    def set_manual_mode(self) -> None:
        if self.mode in (Mode.AUTO_ROUTE, Mode.DISTANCE, Mode.TURN_YAW):
            self.auto_step = AutoStep.IDLE
        self.mode = Mode.MANUAL
        self.is_straight = False
        self.is_turn = False
        self.pid.cross_track_enable = False

    def set_manual_mode_if_idle(self) -> None:
        if self.mode == Mode.IDLE:
            self.set_manual_mode()

    def set_speed_rank(self, level: int) -> None:
        if 0 <= level < 7:
            self.speed_rank = self.is_up * level * SPEEDSTEP

    def speed_acc(self) -> None:
        rank = abs(self.speed_rank)
        if rank < 720:
            rank = min(720, rank + SPEEDSTEP)
            self.speed_rank = self.is_up * rank

    def speed_slow_down(self) -> None:
        rank = abs(self.speed_rank)
        if rank > 0:
            rank = max(0, rank - SPEEDSTEP)
            self.speed_rank = self.is_up * rank

    def ex_direct(self, forward: bool) -> None:
        self.is_up = 1 if forward else -1
        self.speed_rank = abs(self.speed_rank) * self.is_up
        self.org_yaw = self.new_yaw
        self.pid.reset()
        self.odom.reset()
        if self.is_straight:
            self.pid.cross_track_enable = True

    def set_straight(self) -> None:
        self.is_straight = True
        self.is_turn = False
        self.org_yaw = self.new_yaw
        self.pid.reset()
        self.pid.cross_track_enable = True
        self.odom.reset()

    def prepare_straight_hold(self) -> None:
        self.set_straight()
        self.mode = Mode.STRAIGHT
        self.auto_step = AutoStep.IDLE

    def ensure_auto_speed(self) -> None:
        if self.speed_rank == 0:
            self.set_speed_rank(self.auto_speed_level)

    def prepare_distance_drive(self, distance_cm: float) -> bool:
        if abs(distance_cm) < 1.0:
            return False

        if distance_cm < 0.0:
            if self.is_up == 1:
                self.ex_direct(False)
            self.target_distance_cm = -distance_cm
        else:
            if self.is_up == -1:
                self.ex_direct(True)
            self.target_distance_cm = distance_cm

        self.set_straight()
        self.odom.reset()
        self.action_start_tick = self.control_ticks
        self.ensure_auto_speed()
        return True

    def start_distance_drive(self, distance_cm: float) -> None:
        if self.prepare_distance_drive(distance_cm):
            self.mode = Mode.DISTANCE
            self.auto_step = AutoStep.IDLE
            self.reply(f"Distance drive {self.target_distance_cm:.1f} cm")
        else:
            self.reply("Invalid distance target!")

    def prepare_yaw_turn(self, relative_yaw_deg: float) -> bool:
        if abs(relative_yaw_deg) < TURN_DONE_DEG:
            return False
        self.target_yaw = normalize_yaw(self.new_yaw + relative_yaw_deg)
        self.is_straight = False
        self.is_turn = True
        self.pid.cross_track_enable = False
        self.action_start_tick = self.control_ticks
        self.ensure_auto_speed()
        return True

    def start_yaw_turn(self, relative_yaw_deg: float) -> None:
        if self.prepare_yaw_turn(relative_yaw_deg):
            self.mode = Mode.TURN_YAW
            self.auto_step = AutoStep.IDLE
            self.reply(f"Yaw turn {relative_yaw_deg:.1f} deg")
        else:
            self.reply("Invalid yaw target!")

    def start_auto_route(self) -> None:
        self.state = RunState.PARKING
        self.auto_speed_level = AUTO_DEFAULT_SPEED
        self.mode = Mode.AUTO_ROUTE
        self.auto_step = AutoStep.FORWARD1
        if self.prepare_distance_drive(AUTO_FORWARD1_CM):
            self.reply("Auto route start")
        else:
            self.set_standby_mode()
            self.reply("Auto route failed")

    def update_distance_drive(self) -> bool:
        return self.target_distance_cm <= 0.0 or (
            self.target_distance_cm - self.odom.distance
        ) <= DISTANCE_DONE_CM

    def update_yaw_turn(self) -> bool:
        error = normalize_yaw(self.target_yaw - self.new_yaw)
        if abs(error) <= TURN_DONE_DEG:
            self.speed_rank = 0
            self.center_steering()
            self.is_turn = False
            return True

        correction = clamp(
            error * TURN_SERVO_KP, -TURN_SERVO_MAX_OFFSET, TURN_SERVO_MAX_OFFSET
        )
        if self.is_up == -1:
            correction = -correction
        self.angle = clamp(90.0 + correction, 0.0, 180.0)
        self.ensure_auto_speed()
        return False

    def update_control_task(self) -> None:
        if (
            self.mode in (Mode.MANUAL, Mode.STRAIGHT)
            and self.speed_rank != 0
            and self.control_ticks - self.last_command_tick > REMOTE_TIMEOUT_MS
        ):
            self.set_standby_mode()
            self.reply("Remote timeout stop!")
            return

        if (
            self.mode == Mode.DISTANCE
            or (
                self.mode == Mode.AUTO_ROUTE
                and self.auto_step in (AutoStep.FORWARD1, AutoStep.FORWARD2)
            )
        ) and self.control_ticks - self.action_start_tick > DISTANCE_TIMEOUT_MS:
            self.set_standby_mode()
            self.reply("Distance timeout stop!")
            return

        if (
            self.mode == Mode.TURN_YAW
            or (self.mode == Mode.AUTO_ROUTE and self.auto_step == AutoStep.TURN1)
        ) and self.control_ticks - self.action_start_tick > TURN_TIMEOUT_MS:
            self.set_standby_mode()
            self.reply("Turn timeout stop!")
            return

        if self.mode == Mode.DISTANCE:
            if self.update_distance_drive():
                self.set_standby_mode()
                self.reply("Distance done")
        elif self.mode == Mode.TURN_YAW:
            if self.update_yaw_turn():
                self.set_standby_mode()
                self.reply("Yaw turn done")
        elif self.mode == Mode.AUTO_ROUTE:
            if self.auto_step == AutoStep.FORWARD1 and self.update_distance_drive():
                self.speed_rank = 0
                if self.prepare_yaw_turn(AUTO_TURN_DEG):
                    self.auto_step = AutoStep.TURN1
                else:
                    self.set_standby_mode()
                    self.reply("Auto turn failed")
            elif self.auto_step == AutoStep.TURN1 and self.update_yaw_turn():
                if self.prepare_distance_drive(AUTO_FORWARD2_CM):
                    self.auto_step = AutoStep.FORWARD2
                else:
                    self.set_standby_mode()
                    self.reply("Auto forward failed")
            elif self.auto_step == AutoStep.FORWARD2 and self.update_distance_drive():
                self.set_standby_mode()
                self.state = RunState.PARKING
                self.reply("Auto route done")

    def keep_straight(self) -> None:
        error = self.org_yaw - self.new_yaw
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0

        if abs(error) < self.pid.deadband:
            error = 0.0

        p_out = self.pid.kp * error
        if 0.0 < abs(error) < 8.0:
            self.pid.integral += self.pid.ki * error
        elif abs(error) >= 8.0:
            self.pid.integral *= 0.95
        self.pid.integral = clamp(self.pid.integral, -self.pid.max_i, self.pid.max_i)
        i_out = self.pid.integral

        if self.pid.first_run:
            self.pid.last_error = error
            self.pid.first_run = False
        d_error = error - self.pid.last_error
        self.pid.dv = (1.0 - self.pid.d_alpha) * d_error * self.pid.kd + (
            self.pid.d_alpha * self.pid.dv
        )
        d_out = self.pid.dv
        self.pid.last_error = error

        correction = p_out + i_out + d_out
        if self.is_up == -1:
            correction = -correction

        if self.pid.cross_track_enable:
            cross_correction = self.pid.cross_track_kp * self.odom.x
            if self.is_up == -1:
                cross_correction = -cross_correction
            correction += cross_correction

        correction = clamp(correction, -self.pid.max_out, self.pid.max_out)
        target_angle = 90.0 + correction
        self.pid.smoothed_angle = (
            self.pid.smooth_alpha * target_angle
            + (1.0 - self.pid.smooth_alpha) * self.pid.smoothed_angle
        )
        self.angle = self.pid.smoothed_angle

    def handle_command(self, raw_command: str) -> None:
        command = raw_command.strip()
        if command.startswith("@"):
            command = command[1:]
        command = command.strip()
        self.refresh_watchdog()

        if command == "RC_HB":
            self.reply("OK")
        elif command in ("RC_STOP", "AU_STOP"):
            self.reply("Stop!")
            self.set_standby_mode()
        elif command == "RC_MAN":
            self.set_manual_mode()
            self.reply("Manual mode")
        elif command == "RC_STR":
            self.prepare_straight_hold()
            self.reply("Straight hold mode")
        elif command in ("RC_AUTO", "AU_RUN", "ST_PK"):
            if command == "ST_PK":
                self.reply("Parking auto!")
            self.start_auto_route()
        elif command.startswith("RC_DST"):
            value = parse_command_value(command[6:])
            if value is None:
                self.reply("Invalid distance value!")
            else:
                self.start_distance_drive(value)
        elif command.startswith("RC_YAW"):
            value = parse_command_value(command[6:])
            if value is None:
                self.reply("Invalid yaw value!")
            else:
                self.start_yaw_turn(value)
        elif command.startswith("RC_SPD"):
            value = parse_command_value(command[6:])
            if value is None or not (0.0 <= value <= 6.0):
                self.reply("Invalid speed rank!")
            else:
                if self.mode in (Mode.AUTO_ROUTE, Mode.DISTANCE, Mode.TURN_YAW):
                    self.set_manual_mode()
                self.set_manual_mode_if_idle()
                self.set_speed_rank(int(value))
                self.reply(f"SET {self.speed_rank} Rank!")
        elif command.startswith("RC_STE"):
            value = parse_command_value(command[6:])
            if value is None:
                self.reply("Invalid servo value!")
            else:
                self.set_manual_mode()
                self.angle = clamp(value, 0.0, 180.0)
                self.reply(f"Servo to {self.angle:.6f} deg!")
        elif command == "DT_1":
            self.set_manual_mode()
            self.reply("Up!")
            if self.is_up == -1:
                self.ex_direct(True)
        elif command == "DT_0":
            self.set_manual_mode()
            self.reply("Down!")
            if self.is_up == 1:
                self.ex_direct(False)
        elif command == "DT_STA":
            self.reply("Straight!")
            self.prepare_straight_hold()
        elif command == "DT_TUR":
            self.set_manual_mode()
            self.is_turn = True
            self.reply("Turn manual mode!")
        elif command == "SR_ACC":
            if self.mode in (Mode.AUTO_ROUTE, Mode.DISTANCE, Mode.TURN_YAW):
                self.set_manual_mode()
            self.set_manual_mode_if_idle()
            self.reply("SpeedRank add!")
            self.speed_acc()
        elif command == "SR_DEC":
            if self.mode in (Mode.AUTO_ROUTE, Mode.DISTANCE, Mode.TURN_YAW):
                self.set_manual_mode()
            self.set_manual_mode_if_idle()
            self.reply("SpeedRank decline!")
            self.speed_slow_down()
        elif command == "SR_PAU":
            self.set_manual_mode()
            self.reply("SpeedRank stop!")
            self.speed_rank = 0
        elif command.startswith("SR_SET"):
            if len(command) > 6 and command[6].isdigit() and 0 <= int(command[6]) <= 6:
                if self.mode in (Mode.AUTO_ROUTE, Mode.DISTANCE, Mode.TURN_YAW):
                    self.set_manual_mode()
                self.set_manual_mode_if_idle()
                self.set_speed_rank(int(command[6]))
                self.reply(f"SET {self.speed_rank} Rank!")
            else:
                self.reply("Invalid speed rank!")
        elif command.startswith("ST_K") and len(command) >= 5:
            value = parse_command_value(command[5:], scaled_hundredths=True)
            if command[4] not in "PID" or value is None:
                self.reply("Invalid heading PID value!")
            elif command[4] == "P":
                self.pid.kp = value
                self.reply(f"Set heading Kp to {self.pid.kp:.6f}!")
            elif command[4] == "I":
                self.pid.ki = value
                self.reply(f"Set heading Ki to {self.pid.ki:.6f}!")
            elif command[4] == "D":
                self.pid.kd = value
                self.reply(f"Set heading Kd to {self.pid.kd:.6f}!")
        elif command.startswith("RT_TO"):
            value = parse_command_value(command[5:])
            if value is None:
                self.reply("Invalid rotate value!")
            else:
                self.set_manual_mode()
                self.angle = clamp(180.0 - value, 0.0, 180.0)
                self.reply(f"Rotate to {self.angle:.6f} deg!")
        elif command == "ST_SB":
            self.reply("Stand by!")
            self.set_standby_mode()
        elif command == "ST_ER":
            self.reply("Hitted!")
            self.set_standby_mode()
            self.state = RunState.HITTED
        else:
            self.reply("Unknown command!")

    def inject(self, **values: float) -> None:
        if "yaw" in values:
            self.new_yaw = normalize_yaw(values["yaw"])
        if "gyro_z" in values:
            self.gyro_z = values["gyro_z"]
        if "x" in values:
            self.odom.x = values["x"]
        if "y" in values:
            self.odom.y = values["y"]
        if "distance" in values:
            self.odom.distance = values["distance"]

    def plant_step(self, dt_ms: int, yaw_drift_dps: float, lateral_drift_cms: float) -> None:
        dt_s = dt_ms / 1000.0
        level = abs(self.speed_rank) / SPEEDSTEP
        speed_cms = level * RSPEEDSTEP
        direction = 1.0 if self.speed_rank >= 0 else -1.0

        if self.speed_rank != 0:
            self.odom.distance += speed_cms * dt_s
            self.odom.y += direction * speed_cms * dt_s

        if self.is_straight and self.speed_rank != 0:
            self.new_yaw = normalize_yaw(self.new_yaw + yaw_drift_dps * dt_s)
            self.odom.x += lateral_drift_cms * dt_s

        if self.is_turn and self.speed_rank != 0:
            yaw_rate = (self.angle - 90.0) * 1.5 * direction
            self.new_yaw = normalize_yaw(self.new_yaw + yaw_rate * dt_s)

    def tick(
        self,
        ms: int,
        plant: bool = False,
        yaw_drift_dps: float = 0.0,
        lateral_drift_cms: float = 0.0,
    ) -> None:
        remaining = ms
        straight_accum = 0
        while remaining > 0:
            dt = min(remaining, 10)
            self.control_ticks += dt
            straight_accum += dt
            if plant:
                self.plant_step(dt, yaw_drift_dps, lateral_drift_cms)
            if self.is_straight and straight_accum >= 20:
                straight_accum = 0
                self.keep_straight()
            self.update_control_task()
            remaining -= dt

    def value(self, key: str) -> str | float | int:
        values: dict[str, str | float | int] = {
            "mode": self.mode,
            "auto": self.auto_step,
            "state": self.state,
            "speed_rank": self.speed_rank,
            "angle": self.angle,
            "yaw": self.new_yaw,
            "distance": self.odom.distance,
            "x": self.odom.x,
            "y": self.odom.y,
            "kp": self.pid.kp,
            "ki": self.pid.ki,
            "kd": self.pid.kd,
            "tick": self.control_ticks,
            "is_up": self.is_up,
        }
        if key not in values:
            raise KeyError(f"unknown key: {key}")
        return values[key]

    def summary(self) -> str:
        return (
            f"t={self.control_ticks:5d} mode={self.mode:<10} auto={self.auto_step:<8} "
            f"speed={self.speed_rank:4d} angle={self.angle:6.2f} "
            f"yaw={self.new_yaw:7.2f} dist={self.odom.distance:7.2f} "
            f"x={self.odom.x:6.2f} pid=({self.pid.kp:.3f},{self.pid.ki:.3f},{self.pid.kd:.3f})"
        )


def parse_assignments(parts: list[str]) -> dict[str, float]:
    values: dict[str, float] = {}
    for part in parts:
        if "=" not in part:
            raise ValueError(f"expected key=value, got {part!r}")
        key, raw_value = part.split("=", 1)
        values[key] = float(raw_value)
    return values


def compare(actual: str | float | int, operator: str, expected_raw: str) -> bool:
    if isinstance(actual, str):
        expected = expected_raw
        if operator in ("=", "=="):
            return actual == expected
        if operator == "!=":
            return actual != expected
        raise ValueError(f"operator {operator!r} is not valid for string values")

    expected_number = float(expected_raw)
    actual_number = float(actual)
    if operator in ("=", "=="):
        return math.isclose(actual_number, expected_number, rel_tol=1e-5, abs_tol=1e-5)
    if operator == "!=":
        return not math.isclose(actual_number, expected_number, rel_tol=1e-5, abs_tol=1e-5)
    if operator == "<":
        return actual_number < expected_number
    if operator == "<=":
        return actual_number <= expected_number
    if operator == ">":
        return actual_number > expected_number
    if operator == ">=":
        return actual_number >= expected_number
    raise ValueError(f"unknown operator: {operator}")


def run_scenario(path: Path, verbose: bool = True) -> int:
    sim = CarSim()
    failures = 0
    expectation = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(==|=|!=|<=|>=|<|>)\s*(.+)$")

    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        try:
            if line.startswith("@"):
                sim.handle_command(line)
                if verbose:
                    print(f"{path.name}:{line_no}: cmd {line}")
            else:
                parts = line.split()
                op = parts[0].lower()

                if op == "cmd":
                    command = " ".join(parts[1:])
                    sim.handle_command(command)
                    if verbose:
                        print(f"{path.name}:{line_no}: cmd {command}")
                elif op == "sense":
                    sim.inject(**parse_assignments(parts[1:]))
                    if verbose:
                        print(f"{path.name}:{line_no}: sense {' '.join(parts[1:])}")
                elif op == "tick":
                    sim.tick(int(parts[1]))
                    if verbose:
                        print(f"{path.name}:{line_no}: tick {parts[1]}")
                elif op == "drive":
                    kwargs = parse_assignments(parts[2:])
                    sim.tick(
                        int(parts[1]),
                        plant=True,
                        yaw_drift_dps=kwargs.get("yaw_drift", 0.0),
                        lateral_drift_cms=kwargs.get("lateral_drift", 0.0),
                    )
                    if verbose:
                        print(f"{path.name}:{line_no}: drive {parts[1]}")
                elif op == "expect":
                    expression = " ".join(parts[1:])
                    match = expectation.match(expression)
                    if not match:
                        raise ValueError(f"bad expectation: {expression!r}")
                    key, operator, expected = match.groups()
                    actual = sim.value(key)
                    if compare(actual, operator, expected):
                        if verbose:
                            print(f"{path.name}:{line_no}: pass {key}{operator}{expected}")
                    else:
                        failures += 1
                        print(
                            f"{path.name}:{line_no}: FAIL {key}{operator}{expected}, "
                            f"actual={actual}"
                        )
                elif op == "print":
                    print(sim.summary())
                else:
                    raise ValueError(f"unknown scenario op: {op}")

            while sim.responses:
                response = sim.responses.pop(0)
                if verbose:
                    print(f"  uart: {response}")
            if verbose:
                print(f"  {sim.summary()}")
        except Exception as exc:
            failures += 1
            print(f"{path.name}:{line_no}: ERROR {exc}")

    return failures


def main() -> int:
    default_scenario = Path(__file__).with_name("scenarios") / "basic_control.txt"
    parser = argparse.ArgumentParser(description="Run SS928_hub control logic simulation.")
    parser.add_argument("scenario", nargs="?", type=Path, default=default_scenario)
    parser.add_argument("--quiet", action="store_true", help="only print failures")
    args = parser.parse_args()

    failures = run_scenario(args.scenario, verbose=not args.quiet)
    if failures:
        print(f"simulation failed: {failures} failure(s)")
        return 1
    print("simulation passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
