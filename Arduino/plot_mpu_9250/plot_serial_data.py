import serial
import matplotlib.pyplot as plt
import time
from collections import deque

SERIAL_PORT = '/dev/ttyUSB0'  # or 'COM3'
BAUD_RATE = 115200

max_len = 100
pitch_data = deque([0]*max_len, maxlen=max_len)
roll_data = deque([0]*max_len, maxlen=max_len)

plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot(pitch_data, label='Pitch')
line2, = ax.plot(roll_data, label='Roll')
ax.set_ylim(-90, 90)
ax.set_title("Tilt from Accelerometer")
ax.legend()

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)

# Clear initial buffer
for _ in range(10):
    ser.readline()

while True:
    try:
        line = ser.readline().decode().strip()
        if ',' in line:
            pitch_str, roll_str = line.split(',')
            pitch = float(pitch_str)
            roll = float(roll_str)

            pitch_data.append(pitch)
            roll_data.append(roll)

            line1.set_ydata(pitch_data)
            line2.set_ydata(roll_data)

            line1.set_xdata(range(len(pitch_data)))
            line2.set_xdata(range(len(roll_data)))

            ax.draw_artist(ax.patch)
            ax.draw_artist(line1)
            ax.draw_artist(line2)
            fig.canvas.flush_events()
            fig.canvas.blit(ax.bbox)

    except KeyboardInterrupt:
        print("Stopping...")
        break
    except:
        continue
