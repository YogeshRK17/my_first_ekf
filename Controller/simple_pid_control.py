import matplotlib.pyplot as plt
import time

#PID parameters
kp = 8.0 #proportional gain
ki = 3.0 #integral gain
kd = 0.01 #derivative gain

#Simulation parameters
x  = 10.0  #initial position
xd = 110.0 #desired position
dt = 0.1

integral = 0.0
prev_error = 0.0

#For plotting
times = []
positions = []

#Simulation loop
for i in range(100):
    error = xd - x
    integral += error*dt
    derivative = (error - prev_error)/dt

    u = kp*error + ki*integral + kd*derivative
    x = x + u*dt

    positions.append(x)
    times.append(i*dt)

    prev_error = error

    time.sleep(0.05)

    # --- Plot result ---
plt.plot(times, positions, label='Actual Position')
plt.axhline(y=xd, color='r', linestyle='--', label='Goal Position')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('1D Robot Motion using PID Control')
plt.legend()
plt.grid(True)
plt.show()