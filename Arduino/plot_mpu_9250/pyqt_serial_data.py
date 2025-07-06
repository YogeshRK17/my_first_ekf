from pyqtgraph.Qt import QtWidgets
import pyqtgraph as pg
import serial
import time
from collections import deque

# Init Qt
app = QtWidgets.QApplication([])

# Serial config
SERIAL_PORT = '/dev/ttyUSB0'  # or COM3
BAUD_RATE = 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)

for _ in range(10):
    ser.readline()

# Data buffers
max_len = 200
pitch_data = deque([0]*max_len, maxlen=max_len)
roll_data = deque([0]*max_len, maxlen=max_len)

# Setup plot
win = pg.GraphicsLayoutWidget(title="Tilt Plot (Pitch & Roll)")
win.resize(800, 400)
plot = win.addPlot(title="Real-Time Tilt")
plot.addLegend()
plot.setYRange(-90, 90)

curve1 = plot.plot(pen='r', name='Pitch')
curve2 = plot.plot(pen='b', name='Roll')

win.show()

def update():
    line = ser.readline().decode().strip()
    if ',' in line:
        try:
            pitch_str, roll_str = line.split(',')
            pitch = float(pitch_str)
            roll = float(roll_str)

            pitch_data.append(pitch)
            roll_data.append(roll)

            curve1.setData(pitch_data)
            curve2.setData(roll_data)
        except:
            pass

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

app.exec_()
