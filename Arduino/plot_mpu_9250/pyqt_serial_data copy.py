from pyqtgraph.Qt import QtWidgets
import pyqtgraph as pg
import serial
import time
from collections import deque
from kf_imu import kfImu_pitch, kfImu_roll
import time

app = QtWidgets.QApplication([])

# Serial
SERIAL_PORT = '/dev/ttyUSB0'  # or 'COM3'
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)
for _ in range(10): ser.readline()

# Buffers
max_len = 200
# acc pitch and roll
pitch_acc_data  = deque([0]*max_len, maxlen=max_len)
roll_acc_data   = deque([0]*max_len, maxlen=max_len)

#gyro pitch and roll
pitch_gyro_data = deque([0]*max_len, maxlen=max_len)
roll_gyro_data  = deque([0]*max_len, maxlen=max_len)

#complementry pitch and roll
pitch_filter    = deque([0]*max_len, maxlen=max_len)
roll_filter     = deque([0]*max_len, maxlen=max_len)

#gyro yaw
yaw_gyro_data   = deque([0]*max_len, maxlen=max_len)

#KF pitch and roll
filtered_pitch_data = deque([0]*max_len, maxlen=max_len)
filtered_roll_data  = deque([0]*max_len, maxlen=max_len)

# Plot Setup
win = pg.GraphicsLayoutWidget(title="Tilt Comparison: Accel vs Gyro")
win.resize(1000, 500)

# Pitch Plot
plot1 = win.addPlot(title="Pitch: Accel vs Gyro")
plot1.addLegend()
plot1.setYRange(-90, 90)
p1_acc = plot1.plot(pen='r', name='Pitch (Accel)')
p1_gyro = plot1.plot(pen='b', style=pg.QtCore.Qt.DashLine, name='Pitch (Gyro)')

# Roll Plot
win.nextRow()
plot2 = win.addPlot(title="Roll: Accel vs Gyro")
plot2.addLegend()
plot2.setYRange(-90, 90)
r1_acc = plot2.plot(pen='r', name='Roll (Accel)')
r1_gyro = plot2.plot(pen='b', style=pg.QtCore.Qt.DashLine, name='Roll (Gyro)')

# Yaw Plot
win.nextRow()
plot3 = win.addPlot(title="Yaw Gyro")
plot3.addLegend()
plot3.setYRange(-90, 90)
yaw = plot3.plot(pen='r', name='yaw')

# Yaw Plot
win.nextRow()
plot4 = win.addPlot(title="Filter pitch")
plot4.addLegend()
plot4.setYRange(-90, 90)
p1_filter = plot4.plot(pen='r', style=pg.QtCore.Qt.DashLine, name='Pitch (Comp Filter)')
r1_filter = plot4.plot(pen='r', style=pg.QtCore.Qt.DashLine, name='Roll (Comp Filter)')
p1_kf_filter = plot4.plot(pen='g', style=pg.QtCore.Qt.DashLine, name='Pitch (KF Filter)')
r1_kf_filter = plot4.plot(pen='g', style=pg.QtCore.Qt.DashLine, name='Roll (KF Filter)')
win.show()

kf_filter_pitch = kfImu_pitch()
kf_filter_roll  = kfImu_roll()

def filter_pitch(time, pitch, sensor_id):
    measurement = [time, pitch, sensor_id]
    return kf_filter_pitch.filter_pitch(measurement)

def filter_roll(time, roll, sensor_id):
    measurement = [time, roll, sensor_id]
    return kf_filter_roll.filter_roll(measurement)

# Update function
def update():
    line = ser.readline().decode().strip()
    if ',' in line:
        try:
            a, b, c, d, e, f, g = map(float, line.split(','))
            filtered_pitch = filter_pitch(time.time(), a, 0)
            filtered_pitch = filter_pitch(time.time(), c, 1)

            filtered_pitch_data.append(filtered_pitch)

            filtered_roll = filter_roll(time.time(), b, 0)
            filtered_roll = filter_roll(time.time(), d, 1)

            filtered_roll_data.append(filtered_roll)

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
            p1_kf_filter.setData(filtered_pitch_data)
            r1_kf_filter.setData(filtered_roll_data)
        except:
            pass

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

app.exec_()
