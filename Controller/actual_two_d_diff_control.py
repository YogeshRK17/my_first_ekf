'''
Same approach as what we saw in two_d_diff_control.py but using actual dynamics
like here conntroller gives us: v and w
as v = k*error in distance
   w = k*error in angle

Now let's use Ik to get vl and vr
vl = v - (w*l/2)
vr = v + (w*l/2)

then use FK to get robot v and w
v_robot = (vl + vr)/2
w_robot = (vr - vl)/l
'''

import numpy as np
import matplotlib.pyplot as plt
import time

#PID parameters
k_roh   = 0.1
k_alpha = -1.0
dt = 0.05

#Robot parameters
l = 0.5
r = 0.1

#initial pose
x, y, theta     = 0.0,  0.0, 0.0  

#desired pose
xd, yd = 10.0, 20.0

path_x, path_y = [], []

#simulation loop
for i in range(800):
    ex = xd -x
    ey = yd -y

    roh = np.sqrt(ex**2 + ey**2)
    alpha = np.arctan2(ey, ex) - theta

    #Normalize alpha to [-pi, pi]
    alpha = (alpha + np.pi)%2*np.pi - np.pi

    #Control inputs
    v = k_roh * roh
    w = k_alpha * alpha

    #Inverse kinematics
    vl = v - (w*l/2)
    vr = v + (w*l/2)

    #Forward kinematics
    v_robot = (vl + vr)/2
    w_robot = (vr - vl)/l

    #Update pose
    x = x + v_robot * np.cos(theta) * dt
    y = y + v_robot * np.sin(theta) * dt
    theta = theta + w_robot * dt

    path_x.append(x)
    path_y.append(y)

# --- Plot path ---
plt.plot(path_x, path_y, 'b-', label='Robot Path')
plt.plot(xd, yd, 'ro', label='Goal')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Differential Drive Robot Motion')
plt.legend()
plt.grid(True)
plt.show()
