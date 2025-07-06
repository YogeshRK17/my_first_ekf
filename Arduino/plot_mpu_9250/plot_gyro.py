import serial
import matplotlib.pyplot as plt
import time
from collections import deque

# ==== ✅ CONFIGURATION ====
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your port ('COM3' on Windows)
BAUD_RATE = 115200
MAX_POINTS = 100

# ==== ✅ INIT ====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Give time for Arduino to reset

# Data buffers
pitch_buf = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
roll_buf  = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
yaw_buf   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
line1, = ax.plot([], [], label="Pitch (°)")
line2, = ax.plot([], [], label="Roll (°)")
line3, = ax.plot([], [], label="Yaw (°)")
ax.set_ylim(-180, 180)
ax.set_xlim(0, MAX_POINTS)
ax.set_title("Gyroscope Tilt (Pitch, Roll, Yaw)")
ax.set_xlabel("Sample Index")
ax.set_ylabel("Angle (°)")
ax.legend(loc="upper right")
plt.grid(True)

# ==== ✅ LOOP ====
while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if ',' in line:
            parts = line.split(',')
            if len(parts) == 3:
                pitch, roll, yaw = map(float, parts)
                pitch_buf.append(pitch)
                roll_buf.append(roll)
                yaw_buf.append(yaw)

                # Update plots
                x = list(range(len(pitch_buf)))
                line1.set_data(x, pitch_buf)
                line2.set_data(x, roll_buf)
                line3.set_data(x, yaw_buf)

                ax.relim()
                ax.autoscale_view(True, True, True)
                plt.pause(0.001)

    except KeyboardInterrupt:
        print("Stopped by user")
        break
    except Exception as e:
        print("Error:", e)
        continue
