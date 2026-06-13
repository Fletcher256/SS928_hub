#!/usr/bin/env python3
"""Manage the receive-only STM32 serial -> board -> VM ROS2 link."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / ".venv" / "Scripts" / "python"
BOARD_TOOL = ROOT / "tools" / "board_serial.py"
VM_TOOL = ROOT / "tools" / "vm_ssh_run.py"
BOARD_BRIDGE_SCRIPT = ROOT / "tools" / "board_stm32_usb_serial_udp_bridge.py"
REMOTE_BOARD_SCRIPT = "/tmp/board_stm32_usb_serial_udp_bridge.py"
BOARD_STATE_DIR = "/tmp/parking_stm32_link"
VM_STATE_DIR = "/tmp/parking_stm32_link"
BOARD_PID_FILE = f"{BOARD_STATE_DIR}/board_bridge.pid"
VM_PID_FILE = f"{VM_STATE_DIR}/vm_ros.pid"
BOARD_LOG = f"{BOARD_STATE_DIR}/board_bridge.log"
VM_LOG = f"{VM_STATE_DIR}/vm_ros.log"
VM_RECORD_DIR_FILE = f"{VM_STATE_DIR}/vm_record_dir"
BOARD_LOG_TAIL_CMD = "tail -c 12000 {path} 2>/dev/null | tr '\\000' '.' || true"


def cmdline(parts: list[str]) -> str:
    return subprocess.list2cmdline(parts)


def sh_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def run_command(parts: list[str], timeout: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        parts,
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
    )


def board_tool_base(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(BOARD_TOOL),
        "--port",
        args.board_port,
        "--baud",
        str(args.board_baud),
        "--login-user",
        args.board_user,
        "--login-password",
        args.board_password,
        "--timeout",
        str(args.board_timeout),
    ]


def vm_tool_base(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(VM_TOOL),
        "--host",
        args.vm_host,
        "--user",
        args.vm_user,
        "--password",
        args.vm_password,
        "--timeout",
        str(args.vm_timeout),
    ]


def board_upload_cmd(args: argparse.Namespace) -> list[str]:
    return board_tool_base(args) + [
        "--allow-risk",
        "put-text",
        "--allow-risk",
        str(BOARD_BRIDGE_SCRIPT),
        REMOTE_BOARD_SCRIPT,
    ]


def deploy_ros_cmd(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(ROOT / "tools" / "deploy_ros_package.py"),
        "--host",
        args.vm_host,
        "--user",
        args.vm_user,
        "--password",
        args.vm_password,
        "--allow-risk",
    ]


def board_start_shell(args: argparse.Namespace) -> str:
    bridge = [
        "python3",
        REMOTE_BOARD_SCRIPT,
        "--vm-ip",
        args.vm_host,
        "--udp-port",
        str(args.udp_port),
        "--vid",
        args.vid,
        "--pid",
        args.pid,
        "--baud",
        str(args.stm32_baud),
        "--chunk-size",
        str(args.chunk_size),
        "--record-dir",
        args.board_record_dir,
    ]
    if args.bind_generic:
        bridge.append("--bind-generic")
    if args.no_board_record:
        bridge.append("--no-record")
    bridge_cmd = " ".join(sh_quote(part) for part in bridge)
    return f"""sh -lc {sh_quote(f'''
set -e
mkdir -p {sh_quote(BOARD_STATE_DIR)} {sh_quote(args.board_record_dir)}
if [ -s {sh_quote(BOARD_PID_FILE)} ]; then
  old=$(cat {sh_quote(BOARD_PID_FILE)} 2>/dev/null || true)
  if [ -n "$old" ] && [ -d "/proc/$old" ]; then
    echo BOARD_BRIDGE_ALREADY_RUNNING "$old"
    exit 0
  fi
fi
nohup {bridge_cmd} > {sh_quote(BOARD_LOG)} 2>&1 &
pid=$!
echo "$pid" > {sh_quote(BOARD_PID_FILE)}
echo BOARD_BRIDGE_PID "$pid"
echo BOARD_BRIDGE_LOG {sh_quote(BOARD_LOG)}
''')}"""


def vm_start_shell(args: argparse.Namespace) -> str:
    return f"""bash -lc {sh_quote(f'''
set -e
mkdir -p {sh_quote(VM_STATE_DIR)} {sh_quote(args.vm_record_root)}
if [ -s {sh_quote(VM_PID_FILE)} ]; then
  old=$(cat {sh_quote(VM_PID_FILE)} 2>/dev/null || true)
  if [ -n "$old" ] && [ -d "/proc/$old" ]; then
    echo VM_ROS_ALREADY_RUNNING "$old"
    cat {sh_quote(VM_RECORD_DIR_FILE)} 2>/dev/null || true
    exit 0
  fi
fi
run_id=$(date +%Y%m%d_%H%M%S)
record_dir={sh_quote(args.vm_record_root)}/run_$run_id
mkdir -p "$record_dir"
echo "$record_dir" > {sh_quote(VM_RECORD_DIR_FILE)}
nohup setsid bash -lc 'source /opt/ros/humble/setup.bash && source ~/parking_ws/install/setup.bash && exec ros2 launch parking_bridge stm32.launch.py stm32_udp_port:={args.udp_port} record_dir:="'$record_dir'" enable_recording:=true analysis_sample_bytes:={args.analysis_sample_bytes}' > {sh_quote(VM_LOG)} 2>&1 &
pid=$!
echo "$pid" > {sh_quote(VM_PID_FILE)}
echo VM_ROS_PID "$pid"
echo VM_RECORD_DIR "$record_dir"
echo VM_ROS_LOG {sh_quote(VM_LOG)}
''')}"""


def board_start_cmd(args: argparse.Namespace) -> list[str]:
    return board_tool_base(args) + [
        "--allow-risk",
        "run",
        "--allow-risk",
        board_start_shell(args),
    ]


def vm_start_cmd(args: argparse.Namespace) -> list[str]:
    return vm_tool_base(args) + [
        "--allow-risk",
        "run",
        "--allow-risk",
        vm_start_shell(args),
    ]


def board_stop_shell() -> str:
    return f"""sh -lc {sh_quote(f'''
if [ -s {sh_quote(BOARD_PID_FILE)} ]; then
  pid=$(cat {sh_quote(BOARD_PID_FILE)} 2>/dev/null || true)
  if [ -n "$pid" ] && [ -d "/proc/$pid" ]; then
    kill -INT "$pid" 2>/dev/null || true
    sleep 2
    if [ -d "/proc/$pid" ]; then
      kill -TERM "$pid" 2>/dev/null || true
      sleep 1
    fi
  fi
  echo BOARD_BRIDGE_STOPPED "$pid"
else
  echo BOARD_BRIDGE_NOT_RUNNING
fi
''')}"""


def vm_stop_shell() -> str:
    return f"""bash -lc {sh_quote(f'''
if [ -s {sh_quote(VM_PID_FILE)} ]; then
  pid=$(cat {sh_quote(VM_PID_FILE)} 2>/dev/null || true)
  if [ -n "$pid" ]; then
    kill -INT -"$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    sleep 4
    if kill -0 -"$pid" 2>/dev/null || [ -d "/proc/$pid" ]; then
      kill -TERM -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      sleep 1
    fi
  fi
  echo VM_ROS_STOPPED "$pid"
  if [ -s {sh_quote(VM_RECORD_DIR_FILE)} ]; then
    echo VM_RECORD_DIR "$(cat {sh_quote(VM_RECORD_DIR_FILE)} 2>/dev/null)"
  fi
else
  echo VM_ROS_NOT_RUNNING
fi
orphans=$(ps -eo pid,args | awk '/parking_bridge.*stm32_udp_bridge|stm32_udp_bridge/ && !/awk/ {{print $1}}')
if [ -n "$orphans" ]; then
  echo VM_STM32_ORPHANS "$orphans"
  for child in $orphans; do
    kill -INT "$child" 2>/dev/null || true
  done
  sleep 3
  for child in $orphans; do
    if [ -d "/proc/$child" ]; then kill -TERM "$child" 2>/dev/null || true; fi
  done
fi
''')}"""


def board_stop_cmd(args: argparse.Namespace) -> list[str]:
    return board_tool_base(args) + [
        "--allow-risk",
        "run",
        "--allow-risk",
        board_stop_shell(),
    ]


def vm_stop_cmd(args: argparse.Namespace) -> list[str]:
    return vm_tool_base(args) + [
        "--allow-risk",
        "run",
        "--allow-risk",
        vm_stop_shell(),
    ]


def board_health_shell() -> str:
    return f"""sh -lc {sh_quote(f'''
echo BOARD_LINK_HEALTH
uname -a
cat /proc/net/fib_trie | grep 192.168.137 || true
if [ -s {sh_quote(BOARD_PID_FILE)} ]; then
  pid=$(cat {sh_quote(BOARD_PID_FILE)} 2>/dev/null || true)
  echo BOARD_BRIDGE_PID "$pid"
  if [ -n "$pid" ] && [ -d "/proc/$pid" ]; then echo BOARD_BRIDGE_RUNNING yes; else echo BOARD_BRIDGE_RUNNING no; fi
else
  echo BOARD_BRIDGE_PID none
  echo BOARD_BRIDGE_RUNNING no
fi
cat /tmp/stm32_usb_serial_driver_status.json 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyCH341USB* 2>/dev/null || true
readlink -f /sys/bus/usb-serial/devices/ttyUSB0/driver 2>/dev/null || true
readlink -f /sys/bus/usb/devices/*:1.0/driver 2>/dev/null | grep -E 'ch341|usbserial|generic' || true
echo BOARD_LOG_TAIL_BEGIN
{BOARD_LOG_TAIL_CMD.format(path=sh_quote(BOARD_LOG))}
echo BOARD_LOG_TAIL_END
''')}"""


def vm_latest_analysis_code(record_roots: list[str]) -> str:
    roots_repr = repr(record_roots)
    return f"""from pathlib import Path
import json
roots = [Path(p) for p in {roots_repr}]
files = []
for root in roots:
    files.extend(root.glob("run_*/stm32_session_*/stm32_protocol_analysis.json"))
    files.extend(root.glob("stm32_session_*/stm32_protocol_analysis.json"))
files = sorted(files)
print("VM_STM32_ANALYSIS_COUNT", len(files))
if files:
    latest = files[-1]
    print("VM_STM32_LATEST_ANALYSIS", latest)
    data = json.loads(latest.read_text(encoding="utf-8", errors="replace"))
    print("VM_STM32_ANALYSIS_BYTES", data.get("bytes"))
    print("VM_STM32_ANALYSIS_CLASSIFICATION", data.get("classification"))
    print("VM_STM32_ANALYSIS_PROTOCOL_FAMILY", data.get("protocol_family"))
else:
    print("VM_STM32_LATEST_ANALYSIS none")
"""


def vm_health_shell(args: argparse.Namespace) -> str:
    code = vm_latest_analysis_code([
        args.vm_record_root,
        "/home/ebaina/parking_sensor_records/stm32_ros_check",
    ])
    return f"""bash -lc {sh_quote(f'''
echo VM_LINK_HEALTH
hostname
uname -a
if [ -s {sh_quote(VM_PID_FILE)} ]; then
  pid=$(cat {sh_quote(VM_PID_FILE)} 2>/dev/null || true)
  echo VM_ROS_PID "$pid"
  if [ -n "$pid" ] && [ -d "/proc/$pid" ]; then echo VM_ROS_RUNNING yes; else echo VM_ROS_RUNNING no; fi
else
  echo VM_ROS_PID none
  echo VM_ROS_RUNNING no
fi
if [ -s {sh_quote(VM_RECORD_DIR_FILE)} ]; then echo VM_RECORD_DIR "$(cat {sh_quote(VM_RECORD_DIR_FILE)} 2>/dev/null)"; fi
python3 -c {sh_quote(code)}
echo VM_LOG_TAIL_BEGIN
tail -60 {sh_quote(VM_LOG)} 2>/dev/null || true
echo VM_LOG_TAIL_END
''')}"""


def board_health_cmd(args: argparse.Namespace) -> list[str]:
    return board_tool_base(args) + [
        "--allow-risk",
        "run",
        "--allow-risk",
        board_health_shell(),
    ]


def vm_health_cmd(args: argparse.Namespace) -> list[str]:
    return vm_tool_base(args) + [
        "run",
        vm_health_shell(args),
    ]


def board_logs_cmd(args: argparse.Namespace) -> list[str]:
    command = f"sh -lc {sh_quote(BOARD_LOG_TAIL_CMD.format(path=sh_quote(BOARD_LOG)))}"
    return board_tool_base(args) + ["run", command]


def vm_logs_cmd(args: argparse.Namespace) -> list[str]:
    command = f"bash -lc {sh_quote('tail -160 ' + sh_quote(VM_LOG) + ' 2>/dev/null || true')}"
    return vm_tool_base(args) + ["run", command]


def end_to_end_cmd(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(ROOT / "tools" / "stm32_end_to_end_check.py"),
        "--vm-host",
        args.vm_host,
        "--vm-user",
        args.vm_user,
        "--vm-password",
        args.vm_password,
        "--udp-port",
        str(args.udp_port),
        "--vm-duration-sec",
        str(args.check_vm_duration_sec),
        "--board-duration-sec",
        str(args.check_board_duration_sec),
        "--receiver-warmup-sec",
        str(args.check_receiver_warmup_sec),
        "--allow-risk",
    ]


def latest_analysis_cmd(args: argparse.Namespace) -> list[str]:
    return [
        str(PYTHON),
        str(ROOT / "tools" / "vm_print_latest_stm32_analysis.py"),
        "--host",
        args.vm_host,
        "--user",
        args.vm_user,
        "--password",
        args.vm_password,
    ]


def print_result(title: str, result: subprocess.CompletedProcess[str]) -> None:
    print(f"\n=== {title} ===")
    print(result.stdout, end="")
    print(f"{title}_EXIT_CODE {result.returncode}")


def require_preview(args: argparse.Namespace) -> bool:
    return args.action in {"deploy", "start", "stop", "check"} and not args.allow_risk


def preview(args: argparse.Namespace) -> int:
    commands: list[tuple[str, list[str]]] = []
    if args.action == "deploy":
        commands = [("deploy ROS2 package", deploy_ros_cmd(args)), ("upload board bridge", board_upload_cmd(args))]
    elif args.action == "start":
        if args.deploy:
            commands.append(("deploy ROS2 package", deploy_ros_cmd(args)))
        commands.extend([
            ("upload board bridge", board_upload_cmd(args)),
            ("start VM ROS2 receiver", vm_start_cmd(args)),
            ("start board serial UDP forwarder", board_start_cmd(args)),
        ])
    elif args.action == "stop":
        commands = [("stop board forwarder", board_stop_cmd(args)), ("stop VM receiver", vm_stop_cmd(args))]
    elif args.action == "check":
        commands = [("bounded end-to-end check", end_to_end_cmd(args))]

    print("This action needs explicit approval before execution.")
    print()
    for title, command in commands:
        print(f"{title}:")
        print(cmdline(command))
        print()
    print("Purpose:")
    print("- Manage the receive-only STM32 USB serial -> board UDP -> VM ROS2 link.")
    print("- Keep records and logs in fixed board/VM locations for repeatable checks.")
    print()
    print("Risk:")
    print("- May upload helper files to /tmp on the board and rebuild/deploy ROS2 files on the VM.")
    print("- Start/check opens the board USB serial device at 9600 8N1 and may use usbserial_generic fallback.")
    print("- Stop sends INT/TERM only to the recorded receive-only bridge/ROS receiver PIDs.")
    print("- It sends no bytes to STM32 and starts no MCU/CAN/motor/steering/brake/throttle control.")
    print()
    print("Rerun with --allow-risk only after approval.")
    return 4


def do_deploy(args: argparse.Namespace) -> int:
    overall = 0
    for title, command, timeout in (
        ("Deploy ROS2 Package", deploy_ros_cmd(args), 300.0),
        ("Upload Board Bridge", board_upload_cmd(args), args.board_timeout),
    ):
        result = run_command(command, timeout)
        print_result(title, result)
        overall = overall or result.returncode
    return overall


def do_start(args: argparse.Namespace) -> int:
    overall = 0
    if args.deploy:
        result = run_command(deploy_ros_cmd(args), 300.0)
        print_result("Deploy ROS2 Package", result)
        overall = overall or result.returncode
        if result.returncode != 0:
            return overall
    for title, command, timeout in (
        ("Upload Board Bridge", board_upload_cmd(args), args.board_timeout),
        ("Start VM ROS2 Receiver", vm_start_cmd(args), args.vm_timeout),
        ("Start Board Serial UDP Forwarder", board_start_cmd(args), args.board_timeout),
    ):
        result = run_command(command, timeout)
        print_result(title, result)
        overall = overall or result.returncode
    return overall


def do_stop(args: argparse.Namespace) -> int:
    overall = 0
    for title, command, timeout in (
        ("Stop Board Serial UDP Forwarder", board_stop_cmd(args), args.board_timeout),
        ("Stop VM ROS2 Receiver", vm_stop_cmd(args), args.vm_timeout),
    ):
        result = run_command(command, timeout)
        print_result(title, result)
        overall = overall or result.returncode
    return overall


def do_health(args: argparse.Namespace) -> int:
    overall = 0
    for title, command, timeout in (
        ("Board Link Health", board_health_cmd(args), args.board_timeout),
        ("VM Link Health", vm_health_cmd(args), args.vm_timeout),
    ):
        result = run_command(command, timeout)
        print_result(title, result)
        overall = overall or result.returncode
    return overall


def do_logs(args: argparse.Namespace) -> int:
    overall = 0
    for title, command, timeout in (
        ("Board Bridge Log", board_logs_cmd(args), args.board_timeout),
        ("VM ROS Log", vm_logs_cmd(args), args.vm_timeout),
    ):
        result = run_command(command, timeout)
        print_result(title, result)
        overall = overall or result.returncode
    return overall


def do_check(args: argparse.Namespace) -> int:
    result = run_command(
        end_to_end_cmd(args),
        args.check_vm_duration_sec + args.check_board_duration_sec + 240,
    )
    print_result("Bounded End-To-End Check", result)
    return result.returncode


def do_latest_analysis(args: argparse.Namespace) -> int:
    result = run_command(latest_analysis_cmd(args), args.vm_timeout)
    print(result.stdout, end="")
    return result.returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=["deploy", "start", "stop", "health", "logs", "check", "latest-analysis"],
    )
    parser.add_argument("--allow-risk", action="store_true")
    parser.add_argument("--deploy", action="store_true", help="Deploy ROS2 package before start.")
    parser.add_argument("--board-port", default="COM11")
    parser.add_argument("--board-baud", type=int, default=115200)
    parser.add_argument("--board-user", default="root")
    parser.add_argument("--board-password", default="ebaina")
    parser.add_argument("--board-timeout", type=float, default=90.0)
    parser.add_argument("--vm-host", default="192.168.137.100")
    parser.add_argument("--vm-user", default="ebaina")
    parser.add_argument("--vm-password", default="ebaina")
    parser.add_argument("--vm-timeout", type=float, default=90.0)
    parser.add_argument("--udp-port", type=int, default=24680)
    parser.add_argument("--vid", default="1a86")
    parser.add_argument("--pid", default="7523")
    parser.add_argument("--stm32-baud", type=int, default=9600)
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument("--analysis-sample-bytes", type=int, default=8192)
    parser.add_argument("--board-record-dir", default="/tmp/stm32_serial_bridge_records")
    parser.add_argument("--vm-record-root", default="/home/ebaina/parking_sensor_records/stm32_ros_live")
    parser.add_argument("--bind-generic", dest="bind_generic", action="store_true", default=True)
    parser.add_argument("--no-bind-generic", dest="bind_generic", action="store_false")
    parser.add_argument("--no-board-record", action="store_true")
    parser.add_argument("--check-vm-duration-sec", type=int, default=45)
    parser.add_argument("--check-board-duration-sec", type=float, default=30.0)
    parser.add_argument("--check-receiver-warmup-sec", type=float, default=6.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if require_preview(args):
        return preview(args)
    actions = {
        "deploy": do_deploy,
        "start": do_start,
        "stop": do_stop,
        "health": do_health,
        "logs": do_logs,
        "check": do_check,
        "latest-analysis": do_latest_analysis,
    }
    try:
        return actions[args.action](args)
    except subprocess.TimeoutExpired as exc:
        print(f"COMMAND_TIMEOUT {exc}", file=sys.stderr)
        return 124


if __name__ == "__main__":
    raise SystemExit(main())
