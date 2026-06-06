import sys
import serial
import threading
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from collections import deque

# ======================
# 串口配置（请修改为实际参数）
# ======================
SERIAL_PORT = 'COM7'      # 你的串口号
BAUDRATE = 9600
TIMEOUT = 0.1             # 读取超时（秒）

# 数据缓存长度
BUFFER_LEN = 8192

# ======================
# 主窗口类
# ======================
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_data()
        self.init_serial()

    def init_ui(self):
        self.setWindowTitle("串口数据监控与指令发送")
        layout = QtWidgets.QVBoxLayout()

        # 图表区域
        self.graphics_view = pg.GraphicsLayoutWidget()
        self.plot = self.graphics_view.addPlot(title="轮速曲线")
        self.plot.addLegend()
        self.curve_left = self.plot.plot(pen='r', name='Left Speed')
        self.curve_right = self.plot.plot(pen='g', name='Right Speed')
        layout.addWidget(self.graphics_view)

        # 串口指令发送区域
        send_group = QtWidgets.QGroupBox("发送指令")
        send_layout = QtWidgets.QHBoxLayout()
        self.send_edit = QtWidgets.QLineEdit()
        self.send_edit.setPlaceholderText("输入要发送的字符串（自动添加换行）")
        self.send_btn = QtWidgets.QPushButton("发送")
        self.send_btn.clicked.connect(self.send_command)
        send_layout.addWidget(self.send_edit)
        send_layout.addWidget(self.send_btn)
        send_group.setLayout(send_layout)
        layout.addWidget(send_group)

        # 状态栏
        self.status_label = QtWidgets.QLabel("状态：未连接")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.resize(800, 600)
        self.show()

    def init_data(self):
        # 使用 deque 提高效率
        self.left_data = deque(maxlen=BUFFER_LEN)
        self.right_data = deque(maxlen=BUFFER_LEN)

        # 定时器刷新曲线（50Hz）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

    def init_serial(self):
        """打开串口并启动接收线程"""
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=TIMEOUT
            )
            self.status_label.setText(f"状态：已连接 {SERIAL_PORT} @ {BAUDRATE}")
            # 启动串口接收线程
            self.rx_thread = threading.Thread(target=self.serial_receive_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            self.status_label.setText(f"状态：串口打开失败 - {e}")
            self.ser = None

    def serial_receive_loop(self):
        """后台线程：持续读取串口数据并解析"""
        while True:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode().strip()
                    if line:
                        self.parse_serial_data(line)
                except UnicodeDecodeError:
                    pass  # 忽略解码错误
                except Exception as e:
                    print(f"读取错误: {e}")
            QtCore.QThread.msleep(10)   # 避免占用过高CPU

    def parse_serial_data(self, line):
        """
        解析串口数据行
        假设格式： left_speed, right_speed
        例如： "0.88,0.91"
        请根据实际数据格式修改
        """
        try:
            parts = line.split(',')
            if len(parts) >= 2:
                left = float(parts[0])
                right = float(parts[1])
                # 线程安全地添加到队列（使用信号或直接append，注意GIL保护）
                self.left_data.append(left)
                self.right_data.append(right)
                # 可选：打印调试信息
                # print(f"收到: left={left}, right={right}")
        except ValueError:
            print(f"解析失败，无法转换为浮点数: {line}")
        except Exception as e:
            print(f"解析异常: {e}, 原始行: {line}")

    def send_command(self):
        """发送指令到串口（自动添加换行符）"""
        if not self.ser or not self.ser.is_open:
            self.status_label.setText("状态：串口未打开，无法发送")
            return
        text = self.send_edit.text().strip()
        if not text:
            return
        try:
            # 自动添加 \r\n 换行符（可根据需要改为 \n）
            self.ser.write(("@"+text + "\r\n").encode())
            print(f"[发送] {text}")
            self.send_edit.clear()
            self.status_label.setText(f"状态：已发送指令")
        except Exception as e:
            self.status_label.setText(f"状态：发送失败 - {e}")

    def update_plot(self):
        """定时更新曲线（在主线程中执行）"""
        if len(self.left_data) > 1:
            # 将 deque 转为 list 供 setData 使用
            self.curve_left.setData(list(self.left_data))
        if len(self.right_data) > 1:
            self.curve_right.setData(list(self.right_data))

    def closeEvent(self, event):
        """关闭窗口时释放串口资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()


# ======================
# 启动程序
# ======================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    sys.exit(app.exec_())