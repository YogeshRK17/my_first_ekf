import numpy as np
import matplotlib.pyplot as plt 
import control

#System control model
A = np.array([[0, 1],
              [0, 0]])

B = np.array([[0],
              [1]])

#Cost function penalty matrix
Q = np.array([[10, 0],
              [0, 1]])   # penalize position more

R = np.array([[1]])       # penalize control effort


#Solving Math
#designs an optimal state-feedback controller.
#Find the best control law u = -Kx that keeps the system stable and minimizes cost.
# Find the control law u=−Kx that minimizes J
#J=∫0∞​(xTQx+uTRu)dt
#First it solves for P(Riccati Equation), Once P is found it computes optimal gain "K", then it computes eigen values to check stability

# It is pure math + linear algebra.

K, P, eigVals = control.lqr(A, B, Q, R)

print("LQR Gain K:", K)
print("Closed-loop eigenvalues:", eigVals)

#Let's say solution is like this:
# LQR Gain K: [[3.16227766 2.70639157]]
# Closed-loop eigenvalues: [-1.35319578+1.1537499j -1.35319578-1.1537499j]

# Gain indicates how strongly to react to position error and how strongly to react to velocity error. "This is why LQR behaves like a smart PD controller"
# But gains are not hand-tuned — they’re optimal from Q and R.
# Eigenvalues tell how the system moves back to zero, Stability of system



#Close loop dynamics
Acl = A - B @ K
sys_cl = control.ss(Acl, B, np.eye(2), 0)

x0 = [1.0, 10.0]  # initial position = 1, velocity = 0

t = np.linspace(0, 10, 500)
t, y = control.initial_response(sys_cl, T=t, X0=x0)

plt.plot(t, y[0], label="Position")
plt.plot(t, y[1], label="Velocity")
plt.xlabel("Time (s)")
plt.ylabel("State")
plt.legend()
plt.grid()
plt.show()
