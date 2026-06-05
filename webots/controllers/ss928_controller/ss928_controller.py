from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from controller import Keyboard, Supervisor


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_DIR = REPO_ROOT / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from sim_car import AutoStep, CarSim, Mode  # noqa: E402


TIME_STEP_MS = 32
YAW_DRIFT_DPS = 1.2
LATERAL_DRIFT_CMS = 0.3
TRAIL_STEP_M = 0.04
ROBOT_HEIGHT_M = 0.16
ARROW_SPEED_LEVEL = 2
LEFT_STEER_DEG = 125.0
RIGHT_STEER_DEG = 55.0
CAMERA_POSITION = [0.0, 0.85, 1.15]
CAMERA_ORIENTATION = [1.0, 0.0, 0.0, -0.66]


class WebotsCarDebug:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(TIME_STEP_MS)

        self.car = CarSim()
        self.self_node = self.robot.getSelf()
        self.translation_field = self.self_node.getField("translation")
        self.rotation_field = self.self_node.getField("rotation")
        self.root_children = self.robot.getRoot().getField("children")
        self.view_node = self.robot.getFromDef("SS928_VIEW")

        self.running = True
        self.auto_keepalive = True
        self.last_trail = (0.0, 0.0)
        self.last_ui_publish_ms = -1000
        self.command_log: list[str] = []

        self.add_help_label()
        self.reset_camera()
        self.send("@RC_STOP")

    def add_help_label(self) -> None:
        self.robot.setLabel(
            0,
            "SS928 Webots Debug | Arrows drive  1-6 speed  D 60cm  Q/E +/-90  A auto  X stop  R reset  Space pause",
            0.01,
            0.01,
            0.045,
            0x111827,
            0.0,
            "Arial",
        )

    def send(self, command: str) -> None:
        self.car.handle_command(command)
        self.command_log.append(f"tx {command}")
        while self.car.responses:
            self.command_log.append(f"rx {self.car.responses.pop(0)}")
        self.command_log = self.command_log[-6:]

    def reset(self) -> None:
        self.car = CarSim()
        self.last_trail = (0.0, 0.0)
        self.command_log.clear()
        self.reset_camera()
        self.send("@RC_STOP")

    def reset_camera(self) -> None:
        if self.view_node is None:
            return
        self.view_node.getField("position").setSFVec3f(CAMERA_POSITION)
        self.view_node.getField("orientation").setSFRotation(CAMERA_ORIENTATION)
        self.view_node.getField("follow").setSFString("SS928_CAR")
        self.view_node.getField("followType").setSFString("Tracking Shot")

    def ensure_motion_speed(self) -> None:
        if self.car.speed_rank == 0:
            self.send(f"@RC_SPD{ARROW_SPEED_LEVEL}")

    def drive_forward(self) -> None:
        self.send("@DT_1")
        self.ensure_motion_speed()
        self.send("@RC_STR")

    def drive_reverse(self) -> None:
        self.send("@DT_0")
        self.ensure_motion_speed()
        self.send("@RC_STR")

    def manual_turn(self, angle_deg: float) -> None:
        self.ensure_motion_speed()
        self.send(f"@RC_STE{angle_deg:.0f}")
        self.car.is_turn = True
        self.car.is_straight = False
        self.car.pid.cross_track_enable = False

    def process_window_message(self, message: str) -> None:
        if message.startswith("speed:"):
            value = message.split(":", 1)[1]
            if value.isdigit():
                self.send(f"@RC_SPD{value}")
            return

        actions = {
            "forward": self.drive_forward,
            "reverse": self.drive_reverse,
            "left": lambda: self.manual_turn(LEFT_STEER_DEG),
            "right": lambda: self.manual_turn(RIGHT_STEER_DEG),
            "straight": lambda: self.send("@RC_STR"),
            "drive60": lambda: self.send("@RC_DST60"),
            "yawLeft": lambda: self.send("@RC_YAW90"),
            "yawRight": lambda: self.send("@RC_YAW-90"),
            "auto": lambda: self.send("@RC_AUTO"),
            "stop": lambda: self.send("@RC_STOP"),
            "reset": self.reset,
            "camera": self.reset_camera,
            "pause": self.toggle_running,
        }
        action = actions.get(message)
        if action is not None:
            action()

    def toggle_running(self) -> None:
        self.running = not self.running

    def process_key(self, key: int) -> None:
        if key == -1:
            return

        char = chr(key).upper() if 0 <= key < 256 else ""
        if char == " ":
            self.toggle_running()
        elif char in ("1", "2", "3", "4", "5", "6"):
            self.send(f"@RC_SPD{char}")
        elif char == "S":
            self.send("@RC_STR")
        elif char == "D":
            self.send("@RC_DST60")
        elif char == "Q":
            self.send("@RC_YAW90")
        elif char == "E":
            self.send("@RC_YAW-90")
        elif char == "A":
            self.send("@RC_AUTO")
        elif char == "X":
            self.send("@RC_STOP")
        elif char == "R":
            self.reset()
        elif char == "H":
            self.send("@RC_HB")
        elif char == "+":
            self.send("@SR_ACC")
        elif char == "-":
            self.send("@SR_DEC")

        masked_key = key & Keyboard.KEY
        if masked_key in (Keyboard.UP, Keyboard.NUMPAD_UP):
            self.drive_forward()
        elif masked_key in (Keyboard.DOWN, Keyboard.NUMPAD_DOWN):
            self.drive_reverse()
        elif masked_key in (Keyboard.LEFT, Keyboard.NUMPAD_LEFT):
            self.manual_turn(LEFT_STEER_DEG)
        elif masked_key in (Keyboard.RIGHT, Keyboard.NUMPAD_RIGHT):
            self.manual_turn(RIGHT_STEER_DEG)

    def update_pose(self) -> None:
        x_m = self.car.odom.x * 0.01
        z_m = self.car.odom.y * 0.01
        yaw_rad = -math.radians(self.car.new_yaw)
        self.translation_field.setSFVec3f([x_m, ROBOT_HEIGHT_M, z_m])
        self.rotation_field.setSFRotation([0.0, 1.0, 0.0, yaw_rad])

    def add_trail_marker(self) -> None:
        x_m = self.car.odom.x * 0.01
        z_m = self.car.odom.y * 0.01
        dx = x_m - self.last_trail[0]
        dz = z_m - self.last_trail[1]
        if math.hypot(dx, dz) < TRAIL_STEP_M:
            return
        self.last_trail = (x_m, z_m)
        marker = f"""
        Solid {{
          translation {x_m:.4f} 0.012 {z_m:.4f}
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 0.1 0.35 0.95
                emissiveColor 0.02 0.08 0.25
                roughness 0.8
              }}
              geometry Sphere {{
                radius 0.018
              }}
            }}
          ]
        }}
        """
        self.root_children.importMFNodeFromString(-1, marker)

    def update_dashboard(self) -> None:
        target = "-"
        if self.car.mode in (Mode.DISTANCE, Mode.AUTO_ROUTE) and self.car.auto_step != AutoStep.TURN1:
            target = f"{self.car.target_distance_cm:.0f}cm"
        elif self.car.mode == Mode.TURN_YAW or (self.car.mode == Mode.AUTO_ROUTE and self.car.auto_step == AutoStep.TURN1):
            target = f"{self.car.target_yaw:.0f}deg"

        lines = [
            f"mode={self.car.mode} auto={self.car.auto_step} run={'ON' if self.running else 'PAUSE'} keepalive={'ON' if self.auto_keepalive else 'OFF'}",
            f"speed={self.car.speed_rank} servo={self.car.angle:.2f} yaw={self.car.new_yaw:.2f} dist={self.car.odom.distance:.1f}cm x={self.car.odom.x:.1f}cm target={target}",
            f"PID Kp={self.car.pid.kp:.2f} Ki={self.car.pid.ki:.2f} Kd={self.car.pid.kd:.2f}",
        ]
        lines.extend(self.command_log[-4:])
        self.robot.setLabel(1, "\n".join(lines), 0.01, 0.08, 0.045, 0x0F172A, 0.0, "Arial")

    def publish_window_state(self) -> None:
        if self.car.control_ticks - self.last_ui_publish_ms < 200:
            return
        self.last_ui_publish_ms = self.car.control_ticks
        state = {
            "type": "state",
            "mode": self.car.mode,
            "auto": self.car.auto_step,
            "running": self.running,
            "speed": self.car.speed_rank,
            "angle": round(self.car.angle, 1),
            "yaw": round(self.car.new_yaw, 1),
            "distance": round(self.car.odom.distance, 1),
            "x": round(self.car.odom.x, 1),
            "y": round(self.car.odom.y, 1),
            "log": self.command_log[-5:],
        }
        self.robot.wwiSendText(json.dumps(state, separators=(",", ":")))

    def step(self) -> bool:
        if self.robot.step(TIME_STEP_MS) == -1:
            return False

        key = self.keyboard.getKey()
        while key != -1:
            self.process_key(key)
            key = self.keyboard.getKey()

        message = self.robot.wwiReceiveText()
        while message:
            self.process_window_message(message)
            message = self.robot.wwiReceiveText()

        if self.running:
            if self.auto_keepalive and self.car.mode in (Mode.MANUAL, Mode.STRAIGHT) and self.car.speed_rank != 0:
                self.car.refresh_watchdog()
            self.car.tick(TIME_STEP_MS, plant=True, yaw_drift_dps=YAW_DRIFT_DPS, lateral_drift_cms=LATERAL_DRIFT_CMS)
            self.add_trail_marker()

        self.update_pose()
        self.update_dashboard()
        self.publish_window_state()
        return True


def main() -> None:
    debug = WebotsCarDebug()
    while debug.step():
        pass


if __name__ == "__main__":
    main()
