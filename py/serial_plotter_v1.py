import sys
import serial
import threading
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from collections import deque
import numpy as np

# ======================
# 串口配置（请修改为实际参数）
# ======================
SERIAL_PORT = 'COM7'      # 你的串口号
BAUDRATE = 9600
TIMEOUT = 0.1

# 数据缓存长度
BUFFER_LEN = 1048576

# ======================
# 主窗口类
# ======================
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_data()
        self.init_serial()
        self.paused = False          # 暂停标志

    def init_ui(self):
        self.setWindowTitle("串口轨迹监控（x-y坐标）")
        layout = QtWidgets.QVBoxLayout()

        # 图表区域
        self.graphics_view = pg.GraphicsLayoutWidget()
        self.plot = self.graphics_view.addPlot(title="运动轨迹")
        self.plot.setLabel('left', 'X')
        self.plot.setLabel('bottom', 'Y')
        self.plot.setAspectLocked(True)   # 等比例缩放
        self.curve = self.plot.plot(pen='b', symbol='o', symbolSize=3, name='轨迹')
        layout.addWidget(self.graphics_view)

        # 控制区域
        ctrl_layout = QtWidgets.QHBoxLayout()
        self.pause_btn = QtWidgets.QPushButton("暂停")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        ctrl_layout.addWidget(self.pause_btn)

        # 串口指令发送区域
        self.send_edit = QtWidgets.QLineEdit()
        self.send_edit.setPlaceholderText("输入指令（自动添加换行）")
        self.send_btn = QtWidgets.QPushButton("发送")
        self.send_btn.clicked.connect(self.send_command)
        ctrl_layout.addWidget(self.send_edit)
        ctrl_layout.addWidget(self.send_btn)
        layout.addLayout(ctrl_layout)

        # 状态栏
        self.status_label = QtWidgets.QLabel("状态：未连接")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.resize(800, 600)
        self.show()

        # 鼠标移动事件（显示坐标）
        self.proxy = pg.SignalProxy(
            self.plot.scene().sigMouseMoved,
            rateLimit=30,
            slot=self.on_mouse_moved
        )

    def init_data(self):
        self.x_data = deque(maxlen=BUFFER_LEN)
        self.y_data = deque(maxlen=BUFFER_LEN)

        # 定时器刷新曲线（30Hz）
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

    def init_serial(self):
        """打开串口并启动接收线程"""
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=TIMEOUT
            )
            self.status_label.setText(f"状态：已连接 {SERIAL_PORT} @ {BAUDRATE}")
            self.rx_thread = threading.Thread(target=self.serial_receive_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            self.status_label.setText(f"状态：串口打开失败 - {e}")
            self.ser = None

    def serial_receive_loop(self):
        """后台线程：持续读取串口数据"""
        while True:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode().strip()
                    if line:
                        self.parse_serial_data(line)
                except UnicodeDecodeError:
                    pass
                except Exception as e:
                    print(f"读取错误: {e}")
            QtCore.QThread.msleep(10)

    def parse_serial_data(self, line):
        """解析串口数据，提取前两个字段作为 x, y"""
        if self.paused:
            return   # 暂停时不处理新数据
        try:
            parts = line.split(',')
            if len(parts) >= 2:
                x = float(parts[0])
                y = float(parts[1])
                # 添加到缓存（线程安全，GIL保护）
                self.x_data.append(x)
                self.y_data.append(y)
                # 可选：打印调试
                # print(f"轨迹点: ({x}, {y})")
        except ValueError:
            print(f"解析失败，无法转换为浮点数: {line}")
        except Exception as e:
            print(f"解析异常: {e}")

    def send_command(self):
        """发送指令到串口"""
        if not self.ser or not self.ser.is_open:
            self.status_label.setText("状态：串口未打开，无法发送")
            return
        text = self.send_edit.text().strip()
        if not text:
            return
        try:
            self.ser.write((text + "\r\n").encode())
            print(f"[发送] {text}")
            self.send_edit.clear()
            self.status_label.setText(f"状态：指令已发送")
        except Exception as e:
            self.status_label.setText(f"状态：发送失败 - {e}")

    def update_plot(self):
        """定时更新曲线（在主线程中执行）"""
        if len(self.x_data) > 1:
            self.curve.setData(list(self.x_data), list(self.y_data))

    def toggle_pause(self, checked):
        """暂停/恢复按钮回调"""
        self.paused = checked
        if checked:
            self.pause_btn.setText("继续")
            self.status_label.setText("状态：已暂停接收")
        else:
            self.pause_btn.setText("暂停")
            self.status_label.setText("状态：接收中")

    def on_mouse_moved(self, pos):
        """鼠标移动时显示最近数据点的坐标"""
        if not self.x_data or not self.y_data:
            return
        # 获取鼠标对应的场景坐标
        mouse_point = self.plot.vb.mapSceneToView(pos)
        x_mouse = mouse_point.x()
        y_mouse = mouse_point.y()

        # 查找曲线上最近的点
        xs = np.array(self.x_data)
        ys = np.array(self.y_data)
        distances = (xs - x_mouse)**2 + (ys - y_mouse)**2
        if len(distances) == 0:
            return
        idx = np.argmin(distances)
        closest_x = xs[idx]
        closest_y = ys[idx]
        # 如果鼠标距离该点较远，可以不显示（可调整阈值）
        threshold = 0.5  # 可根据实际缩放调整
        if distances[idx] < threshold**2:
            self.status_label.setText(f"轨迹点: ({closest_x:.3f}, {closest_y:.3f})")
        else:
            # 恢复显示普通状态信息
            if not self.paused:
                self.status_label.setText("状态：接收中")
            else:
                self.status_label.setText("状态：已暂停接收")

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