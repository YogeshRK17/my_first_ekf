'''
current pose = x, y, theta
desired pose = xd, yd, theta_d

control input = v, w

vx = v*cos(theta)
vy = v*sin(theta)
w  = rate of change of theta

ex = xd -x
ey = yd -y

etheta = atan2(ey,ex) - theta
e = sqrt(ex^2 + ey^2)
'''

import numpy as np
import matplotlib.pyplot as plt
import time

#PID parameters
k_roh   = 1.8
k_alpha = 10.0
dt = 0.05

x, y, theta     = 0.0,  0.0,  0.0  # initial pose
xd, yd, theta_d = 10.0, 20.0, 0.0 # desired pose

path_x, path_y = [], []

#Simulation loop
for i in range(400):
    ex = xd - x
    ey = yd - y
    roh = np.sqrt(ex**2 + ey**2)
    alpha = np.arctan2(ey, ex) - theta

    alpha = (alpha + np.pi)%(2*np.pi) - np.pi  # wrap to [-pi, pi]

    #control inputs
    v = k_roh * roh
    w = k_alpha * alpha

    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt
    theta = theta + w * dt

    path_x.append(x)
    path_y.append(y)

# --- Plot path ---
plt.plot(path_x, path_y, 'b-', label='Robot Path')
plt.plot(xd, yd, 'ro', label='Goal')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Differential Drive Robot to Goal')
plt.legend()
plt.grid(True)
plt.show()