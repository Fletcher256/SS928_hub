import sys
import serial
import pyqtgraph as pg
from PyQt5 import QtWidgets
from collections import deque

# ======================
# 串口配置
# ======================

ser = serial.Serial(
    'COM7',      # 改成你的串口
    9600,
    timeout=1
)

# ======================
# 创建Qt应用
# ======================

app = QtWidgets.QApplication(sys.argv)

# 创建窗口
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Ackermann Monitor")

# 添加图表
plot = win.addPlot(title="Wheel Speed")

plot.addLegend()

curve_left = plot.plot(
    pen='r',
    name='Left'
)

curve_right = plot.plot(
    pen='g',
    name='Right'
)

# 数据缓存
left_data = deque(maxlen=200)
right_data = deque(maxlen=200)

# ======================
# 主循环
# ======================

def update():

    global left_data, right_data

    if ser.in_waiting:

        line = ser.readline().decode().strip()

        try:

            data = line.split(',')

            x = float(data[0])
            y = float(data[1])
            yaw = float(data[2])

            left_speed = float(data[3])
            right_speed = float(data[4])

            left_data.append(left_speed)
            right_data.append(right_speed)

            curve_left.setData(list(left_data))
            curve_right.setData(list(right_data))

        except:
            pass

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

sys.exit(app.exec_())