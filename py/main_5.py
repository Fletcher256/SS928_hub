import sys
import serial
import threading
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from collections import deque
import pandas as pd
from datetime import datetime
import os

# ======================
# 串口配置（请修改）
# ======================
SERIAL_PORT = 'COM7'      # 您的串口号
BAUDRATE = 9600
TIMEOUT = 0.1

# 数据缓存长度（实时显示最近N个点）
BUFFER_LEN = 200

# 通道名称及颜色
CHANNELS = ['Gx', 'Gy', 'Gz', 'Servo_Angle', 'Speed']
COLORS = ['r', 'g', 'b', 'm', 'c']


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_data()
        self.init_serial()
        self.recording = False           # 是否记录数据到文件
        self.record_buffer = []          # 存储完整记录 [(timestamp, gx,gy,gz,servo,speed), ...]

    def init_ui(self):
        self.setWindowTitle("小车数据采集与监控 (Gx/Gy/Gz/舵机/车速)")
        layout = QtWidgets.QVBoxLayout()

        # ========== 图表区域 ==========
        self.graphics_view = pg.GraphicsLayoutWidget()
        self.plot = self.graphics_view.addPlot(title="实时数据曲线")
        self.plot.addLegend()
        self.curves = {}
        for i, (name, color) in enumerate(zip(CHANNELS, COLORS)):
            curve = self.plot.plot(pen=color, name=name)
            self.curves[name] = curve
        layout.addWidget(self.graphics_view)

        # ========== 控制按钮区域 ==========
        ctrl_layout = QtWidgets.QHBoxLayout()

        self.record_btn = QtWidgets.QPushButton("开始记录")
        self.record_btn.setCheckable(True)
        self.record_btn.toggled.connect(self.toggle_record)
        ctrl_layout.addWidget(self.record_btn)

        self.save_btn = QtWidgets.QPushButton("保存数据")
        self.save_btn.clicked.connect(self.save_data)
        ctrl_layout.addWidget(self.save_btn)

        self.clear_btn = QtWidgets.QPushButton("清除曲线")
        self.clear_btn.clicked.connect(self.clear_curves)
        ctrl_layout.addWidget(self.clear_btn)

        self.pause_btn = QtWidgets.QPushButton("暂停刷新")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self.toggle_pause)
        ctrl_layout.addWidget(self.pause_btn)

        layout.addLayout(ctrl_layout)

        # ========== 指令发送区域 ==========
        send_group = QtWidgets.QGroupBox("发送指令")
        send_layout = QtWidgets.QHBoxLayout()
        self.send_edit = QtWidgets.QLineEdit()
        self.send_edit.setPlaceholderText("输入指令（自动添加换行）")
        self.send_btn = QtWidgets.QPushButton("发送")
        self.send_btn.clicked.connect(self.send_command)
        send_layout.addWidget(self.send_edit)
        send_layout.addWidget(self.send_btn)
        send_group.setLayout(send_layout)
        layout.addWidget(send_group)

        # ========== 状态栏 ==========
        self.status_label = QtWidgets.QLabel("状态：未连接")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.resize(1000, 700)
        self.show()

    def init_data(self):
        # 使用 deque 存储最近数据用于显示
        self.data_buffers = {name: deque(maxlen=BUFFER_LEN) for name in CHANNELS}
        # 定时器刷新曲线 (30Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)
        self.plot_paused = False   # 曲线刷新暂停标志

    def init_serial(self):
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
        """
        解析串口数据，期望格式: gx,gy,gz,servo_angle,speed
        例如: "0.12,0.34,0.56,30.0,0.88"
        """
        try:
            parts = line.split(',')
            if len(parts) >= 5:
                gx = float(parts[0])
                gy = float(parts[1])
                gz = float(parts[2])
                servo = float(parts[3])
                speed = float(parts[4])

                # 更新实时缓冲区（即使暂停刷新也存入deque，防止数据丢失）
                self.data_buffers['Gx'].append(gx)
                self.data_buffers['Gy'].append(gy)
                self.data_buffers['Gz'].append(gz)
                self.data_buffers['Servo_Angle'].append(servo)
                self.data_buffers['Speed'].append(speed)

                # 如果正在记录数据，存到记录缓冲
                if self.recording:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    self.record_buffer.append((
                        timestamp, gx, gy, gz, servo, speed
                    ))
                    # 可选：限制缓冲区大小，避免内存爆炸
                    if len(self.record_buffer) > 10000:
                        self.record_buffer.pop(0)

                # 可选打印调试
                # print(f"收到: gx={gx}, gy={gy}, gz={gz}, servo={servo}, speed={speed}")
        except ValueError:
            print(f"解析失败(数据非数字): {line}")
        except Exception as e:
            print(f"解析异常: {e}, 原始行: {line}")

    def update_plot(self):
        """定时更新曲线（主线程）"""
        if self.plot_paused:
            return
        for name, curve in self.curves.items():
            data = list(self.data_buffers[name])
            if len(data) > 0:
                curve.setData(data)

    def toggle_record(self, checked):
        """开始/停止记录"""
        self.recording = checked
        if checked:
            self.record_btn.setText("停止记录")
            self.status_label.setText("状态：正在记录数据...")
            # 清空之前的记录缓冲区（可选）
            # self.record_buffer.clear()
        else:
            self.record_btn.setText("开始记录")
            self.status_label.setText(f"状态：已停止记录，共 {len(self.record_buffer)} 条数据待保存")

    def save_data(self):
        """保存记录的数据到 CSV 或 Excel 文件"""
        if not self.record_buffer:
            QtWidgets.QMessageBox.information(self, "提示", "没有记录的数据，请先点击「开始记录」采集数据。")
            return

        # 弹出保存对话框
        file_path, selected_filter = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存数据", "", "CSV 文件 (*.csv);;Excel 文件 (*.xlsx)"
        )
        if not file_path:
            return

        # 将记录缓冲区转换为 DataFrame
        df = pd.DataFrame(self.record_buffer, columns=['Timestamp', 'Gx', 'Gy', 'Gz', 'Servo_Angle', 'Speed'])

        try:
            if file_path.endswith('.csv'):
                df.to_csv(file_path, index=False)
            elif file_path.endswith('.xlsx'):
                df.to_excel(file_path, index=False, engine='openpyxl')
            else:
                # 根据选择的过滤器决定格式
                if 'csv' in selected_filter.lower():
                    df.to_csv(file_path, index=False)
                elif 'xlsx' in selected_filter.lower():
                    df.to_excel(file_path, index=False, engine='openpyxl')
                else:
                    # 默认存为 CSV
                    df.to_csv(file_path, index=False)
            QtWidgets.QMessageBox.information(self, "成功", f"数据已保存至：{file_path}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "保存失败", f"错误信息：{e}")

    def clear_curves(self):
        """清除所有曲线数据和显示缓冲区"""
        for name in self.data_buffers:
            self.data_buffers[name].clear()
        self.status_label.setText("状态：曲线已清除")

    def toggle_pause(self, checked):
        """暂停/恢复曲线刷新"""
        self.plot_paused = checked
        if checked:
            self.pause_btn.setText("恢复刷新")
            self.status_label.setText("状态：曲线刷新暂停（数据仍在后台接收）")
        else:
            self.pause_btn.setText("暂停刷新")
            self.status_label.setText("状态：曲线刷新已恢复")

    def send_command(self):
        if not self.ser or not self.ser.is_open:
            self.status_label.setText("状态：串口未打开，无法发送")
            return
        text = self.send_edit.text().strip()
        if not text:
            return
        try:
            self.ser.write(("@"+text + "\r\n").encode())
            print(f"[发送] {text}")
            self.send_edit.clear()
            self.status_label.setText("状态：指令已发送")
        except Exception as e:
            self.status_label.setText(f"状态：发送失败 - {e}")

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    sys.exit(app.exec_())