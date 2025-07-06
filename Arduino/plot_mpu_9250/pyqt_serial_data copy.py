from pyqtgraph.Qt import QtWidgets
import pyqtgraph as pg
import serial
import time
from collections import deque

app = QtWidgets.QApplication([])

# Serial
SERIAL_PORT = '/dev/ttyUSB0'  # or 'COM3'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)
for _ in range(10): ser.readline()

# Buffers
max_len = 200
pitch_acc_data  = deque([0]*max_len, maxlen=max_len)
roll_acc_data   = deque([0]*max_len, maxlen=max_len)
pitch_gyro_data = deque([0]*max_len, maxlen=max_len)
roll_gyro_data  = deque([0]*max_len, maxlen=max_len)
pitch_filter    = deque([0]*max_len, maxlen=max_len)
roll_filter     = deque([0]*max_len, maxlen=max_len)
yaw_gyro_data   = deque([0]*max_len, maxlen=max_len)

# Plot Setup
win = pg.GraphicsLayoutWidget(title="Tilt Comparison: Accel vs Gyro")
win.resize(1000, 500)

# Pitch Plot
plot1 = win.addPlot(title="Pitch: Accel vs Gyro")
plot1.addLegend()
plot1.setYRange(-90, 90)
p1_acc = plot1.plot(pen='r', name='Pitch (Accel)')
p1_gyro = plot1.plot(pen='b', style=pg.QtCore.Qt.DashLine, name='Pitch (Gyro)')
p1_filter = plot1.plot(pen='g', style=pg.QtCore.Qt.DashLine, name='Pitch (Filter)')

# Roll Plot
win.nextRow()
plot2 = win.addPlot(title="Roll: Accel vs Gyro")
plot2.addLegend()
plot2.setYRange(-90, 90)
r1_acc = plot2.plot(pen='r', name='Roll (Accel)')
r1_gyro = plot2.plot(pen='b', style=pg.QtCore.Qt.DashLine, name='Roll (Gyro)')
r1_filter = plot2.plot(pen='g', style=pg.QtCore.Qt.DashLine, name='Roll (Filter)')

# Yaw Plot
win.nextRow()
plot3 = win.addPlot(title="Yaw Gyro")
plot3.addLegend()
plot3.setYRange(-90, 90)
yaw = plot3.plot(pen='r', name='yaw')

win.show()

# Update function
def update():
    line = ser.readline().decode().strip()
    if ',' in line:
        try:
            a, b, c, d, e, f, g = map(float, line.split(','))
            pitch_acc_data.append(a)
            roll_acc_data.append(b)
            pitch_gyro_data.append(c)
            roll_gyro_data.append(d)
            pitch_filter.append(e)
            roll_filter.append(f)
            yaw_gyro_data.append(g)

            p1_acc.setData(pitch_acc_data)
            p1_gyro.setData(pitch_gyro_data)
            r1_acc.setData(roll_acc_data)
            r1_gyro.setData(roll_gyro_data)
            p1_filter.setData(pitch_filter)
            r1_filter.setData(roll_filter)
            yaw.setData(yaw_gyro_data)
        except:
            pass

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

app.exec_()
