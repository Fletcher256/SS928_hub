import argparse
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports


DEFAULT_BAUD = 9600
DEFAULT_TARGET_MAC = "202641EDF326"
READ_TIMEOUT_S = 0.1


def normalize_mac(value: str) -> str:
    return re.sub(r"[^0-9A-Fa-f]", "", value).upper()


def list_serial_ports():
    ports = []
    target_mac = normalize_mac(DEFAULT_TARGET_MAC)
    for item in serial.tools.list_ports.comports():
        hwid = item.hwid or ""
        description = item.description or ""
        score = 0
        if target_mac and target_mac in normalize_mac(hwid):
            score += 100
        if "BTHENUM" in hwid.upper():
            score += 20
        if "000000000000" in hwid:
            score -= 10
        if "COM" in item.device.upper():
            score += 1
        ports.append(
            {
                "device": item.device,
                "description": description,
                "hwid": hwid,
                "score": score,
            }
        )
    ports.sort(key=lambda p: (p["score"], p["device"]), reverse=True)
    return ports


def best_port():
    ports = list_serial_ports()
    return ports[0]["device"] if ports else ""


def format_command(seq: int, text: str) -> str:
    text = text.strip()
    if not text:
        return ""
    if text.startswith("@"):
        return text
    return f"@{seq} {text}"


def ping_port(port: str, baud: int = DEFAULT_BAUD, timeout_s: float = 2.0) -> tuple[bool, str]:
    response = ""
    try:
        with serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=1.0) as ser:
            time.sleep(0.4)
            ser.reset_input_buffer()
            ser.write(b"@1 PING\r")
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                chunk = ser.read(256)
                if chunk:
                    response += chunk.decode("utf-8", errors="replace")
                    if "PONG" in response:
                        return True, response.strip()
                time.sleep(0.02)
    except Exception as exc:
        return False, str(exc)
    return False, response.strip() or "no response"


class SerialSession:
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port: str, baud: int):
        self.close()
        self.stop_event.clear()
        self.ser = serial.Serial(port, baudrate=baud, timeout=READ_TIMEOUT_S, write_timeout=1.0)
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write_line(self, line: str):
        if not self.is_open:
            raise RuntimeError("serial port is not open")
        self.ser.write((line.rstrip("\r\n") + "\r").encode("utf-8"))

    def _read_loop(self):
        while not self.stop_event.is_set() and self.ser is not None:
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self.rx_queue.put(text)
            except Exception as exc:
                self.rx_queue.put(f"[RX ERROR] {exc}")
                break


class CommandTester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 Bluetooth Command Tester")
        self.geometry("1040x720")
        self.minsize(920, 620)

        self.rx_queue = queue.Queue()
        self.serial = SerialSession(self.rx_queue)

        self.port_var = tk.StringVar(value=best_port())
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.seq_var = tk.IntVar(value=1)
        self.status_var = tk.StringVar(value="Disconnected")
        self.poll_stat_var = tk.BooleanVar(value=False)

        self.dist_var = tk.StringVar(value="30")
        self.speed_var = tk.StringVar(value="2")
        self.turn_var = tk.StringVar(value="90")
        self.steer_var = tk.StringVar(value="120")
        self.custom_var = tk.StringVar(value="STAT")

        self._build_ui()
        self.refresh_ports()
        self.after(100, self._drain_rx)
        self.after(1000, self._poll_stat)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.LabelFrame(root, text="Connection")
        conn.pack(fill=tk.X)
        ttk.Label(conn, text="Port").pack(side=tk.LEFT, padx=(8, 4), pady=8)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Label(conn, text="Baud").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Entry(conn, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Auto Connect", command=self.auto_connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Connect", command=self.connect_selected).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(conn, text="Poll STAT", variable=self.poll_stat_var).pack(side=tk.LEFT, padx=12)
        ttk.Label(conn, textvariable=self.status_var).pack(side=tk.RIGHT, padx=8)

        main = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, pady=10)

        controls = ttk.Frame(main)
        main.add(controls, weight=0)
        log_frame = ttk.Frame(main)
        main.add(log_frame, weight=1)

        self._build_command_controls(controls)
        self._build_log(log_frame)

    def _build_command_controls(self, parent):
        basic = ttk.LabelFrame(parent, text="Basic")
        basic.pack(fill=tk.X, pady=(0, 8))
        for label, cmd in [
            ("PING", "PING"),
            ("VER", "VER"),
            ("STAT", "STAT"),
            ("PWM_STAT", "PWM_STAT"),
            ("TEL OFF", "TEL OFF"),
            ("TEL ON", "TEL ON"),
            ("STOP", "STOP"),
            ("ZERO_ALL", "ZERO_ALL"),
        ]:
            ttk.Button(basic, text=label, command=lambda c=cmd: self.send_command(c)).pack(fill=tk.X, padx=8, pady=2)

        motion = ttk.LabelFrame(parent, text="Motion")
        motion.pack(fill=tk.X, pady=8)
        grid = ttk.Frame(motion)
        grid.pack(fill=tk.X, padx=8, pady=6)
        ttk.Label(grid, text="D cm").grid(row=0, column=0, sticky="w")
        ttk.Entry(grid, textvariable=self.dist_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(grid, text="V").grid(row=0, column=2, sticky="w")
        ttk.Entry(grid, textvariable=self.speed_var, width=5).grid(row=0, column=3, padx=4)
        ttk.Label(grid, text="A deg").grid(row=1, column=0, sticky="w")
        ttk.Entry(grid, textvariable=self.turn_var, width=8).grid(row=1, column=1, padx=4)
        ttk.Label(grid, text="STE").grid(row=1, column=2, sticky="w")
        ttk.Entry(grid, textvariable=self.steer_var, width=5).grid(row=1, column=3, padx=4)
        ttk.Button(motion, text="MOVE forward", command=self.move_forward).pack(fill=tk.X, padx=8, pady=2)
        ttk.Button(motion, text="MOVE reverse", command=self.move_reverse).pack(fill=tk.X, padx=8, pady=2)
        ttk.Button(motion, text="TURN left", command=lambda: self.turn(+1)).pack(fill=tk.X, padx=8, pady=2)
        ttk.Button(motion, text="TURN right", command=lambda: self.turn(-1)).pack(fill=tk.X, padx=8, pady=2)
        ttk.Button(motion, text="ARC forward", command=self.arc_forward).pack(fill=tk.X, padx=8, pady=2)
        ttk.Button(motion, text="ARC reverse", command=self.arc_reverse).pack(fill=tk.X, padx=8, pady=2)

        servo = ttk.LabelFrame(parent, text="Servo")
        servo.pack(fill=tk.X, pady=8)
        for angle in [45, 60, 75, 90, 105, 120, 135]:
            ttk.Button(servo, text=f"SERVO {angle}", command=lambda a=angle: self.send_command(f"SERVO A={a}")).pack(
                fill=tk.X, padx=8, pady=2
            )

        params = ttk.LabelFrame(parent, text="Params")
        params.pack(fill=tk.X, pady=8)
        for label, cmd in [
            ("GET HEADING", "GET PARAM=HEADING"),
            ("GET MOTOR", "GET PARAM=MOTOR"),
            ("GET LIMIT", "GET PARAM=LIMIT"),
            ("Safe limit", "SET PARAM=LIMIT STE_MIN=45 STE_MAX=135 SPEED_MAX=3"),
            ("Default cfg", "DEFAULT_CFG"),
        ]:
            ttk.Button(params, text=label, command=lambda c=cmd: self.send_command(c)).pack(fill=tk.X, padx=8, pady=2)

        custom = ttk.LabelFrame(parent, text="Custom")
        custom.pack(fill=tk.X, pady=8)
        ttk.Entry(custom, textvariable=self.custom_var, width=36).pack(fill=tk.X, padx=8, pady=(8, 4))
        ttk.Button(custom, text="Send", command=lambda: self.send_command(self.custom_var.get())).pack(
            fill=tk.X, padx=8, pady=(0, 8)
        )

    def _build_log(self, parent):
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="Log").pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Clear", command=self.clear_log).pack(side=tk.RIGHT)
        self.log = tk.Text(parent, wrap=tk.WORD, height=20, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

    def refresh_ports(self):
        ports = list_serial_ports()
        values = [f"{p['device']}  {p['description']}" for p in ports]
        self.port_combo["values"] = values
        if ports:
            current_device = self.port_var.get().split()[0] if self.port_var.get() else ""
            devices = [p["device"] for p in ports]
            if current_device not in devices:
                self.port_var.set(values[0])
        self._log("SYS", f"ports: {', '.join(v for v in values) or 'none'}")

    def selected_port(self) -> str:
        return self.port_var.get().split()[0].strip()

    def selected_baud(self) -> int:
        try:
            return int(self.baud_var.get())
        except ValueError:
            return DEFAULT_BAUD

    def auto_connect(self):
        self.refresh_ports()
        port = best_port()
        if not port:
            messagebox.showerror("Auto Connect", "No serial port found.")
            return
        self.port_var.set(port)
        self.connect_selected()
        if self.serial.is_open:
            self.send_command("PING")

    def connect_selected(self):
        port = self.selected_port()
        if not port:
            messagebox.showerror("Connect", "No port selected.")
            return
        try:
            self.serial.open(port, self.selected_baud())
            self.status_var.set(f"Connected: {port} @ {self.selected_baud()}")
            self._log("SYS", f"connected {port}")
        except Exception as exc:
            self.status_var.set(f"Connect failed: {exc}")
            self._log("ERR", f"connect failed: {exc}")

    def disconnect(self):
        self.serial.close()
        self.status_var.set("Disconnected")
        self._log("SYS", "disconnected")

    def next_seq(self) -> int:
        seq = self.seq_var.get()
        self.seq_var.set(seq + 1 if seq < 65000 else 1)
        return seq

    def send_command(self, text: str):
        line = format_command(self.next_seq(), text)
        if not line:
            return
        if not self.serial.is_open:
            self.auto_connect()
        if not self.serial.is_open:
            return
        try:
            self.serial.write_line(line)
            self._log("TX", line)
        except Exception as exc:
            self._log("ERR", f"send failed: {exc}")
            self.status_var.set(f"Send failed: {exc}")

    def move_forward(self):
        self.send_command(f"MOVE D={self.dist_var.get()} V={self.speed_var.get()}")

    def move_reverse(self):
        self.send_command(f"MOVE D=-{abs(float(self.dist_var.get() or '0')):.1f} V={self.speed_var.get()}")

    def turn(self, sign: int):
        angle = abs(float(self.turn_var.get() or "0")) * sign
        self.send_command(f"TURN A={angle:.1f} V={self.speed_var.get()}")

    def arc_forward(self):
        self.send_command(f"ARC D={self.dist_var.get()} STE={self.steer_var.get()} V={self.speed_var.get()}")

    def arc_reverse(self):
        distance = abs(float(self.dist_var.get() or "0"))
        self.send_command(f"ARC D=-{distance:.1f} STE={self.steer_var.get()} V={self.speed_var.get()}")

    def _drain_rx(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._log("RX", line)
                if "PONG" in line:
                    self.status_var.set(f"Connected: {self.selected_port()} @ {self.selected_baud()}")
        except queue.Empty:
            pass
        self.after(100, self._drain_rx)

    def _poll_stat(self):
        if self.poll_stat_var.get() and self.serial.is_open:
            try:
                self.serial.write_line(format_command(self.next_seq(), "STAT"))
            except Exception as exc:
                self._log("ERR", f"poll failed: {exc}")
        self.after(1000, self._poll_stat)

    def _log(self, kind: str, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, f"[{stamp}] {kind:<3} {text}\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def clear_log(self):
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.configure(state=tk.DISABLED)

    def _on_close(self):
        self.serial.close()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="STM32 Bluetooth SPP command tester")
    parser.add_argument("--scan", action="store_true", help="list available serial ports")
    parser.add_argument("--ping", action="store_true", help="auto-detect port and send PING")
    parser.add_argument("--port", default="", help="serial port, for example COM13")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    if args.scan:
        for port in list_serial_ports():
            print(f"{port['device']}\t score={port['score']}\t {port['description']}\t {port['hwid']}")
        return

    if args.ping:
        port = args.port or best_port()
        if not port:
            raise SystemExit("no serial port found")
        ok, response = ping_port(port, args.baud)
        print(f"port={port} ok={ok} response={response}")
        raise SystemExit(0 if ok else 1)

    app = CommandTester()
    app.mainloop()


if __name__ == "__main__":
    main()
