import numpy as np
import matplotlib.pyplot as plt
import time

class Kalman:
	def __init__(self):
		self.Qk = 0.001*np.eye(3)
		self.Qk[4:,4:] = np.eye(1)*0.005
		self.Rk = 0.1*np.eye(1)
		self.state=np.array([[0],[0],[0]])
		self.PIX_stateE=np.array([[0],[0],[0]])
		self.Pk = 1.0 * np.eye(self.Qk.shape[0])
		self.Hk = np.array([[1,0,0]])

	def Filter(self, dt, Meas):
		Meas=Meas.reshape(-1,1)
		# model
		Ak = np.array([[1, dt,dt**2/2],[0., 1., dt],[0., 0., 1]])
		
		RK = self.Rk
		
		self.Pk = np.matmul(Ak, np.matmul(self.Pk, Ak.T)) + self.Qk
		self.state = np.matmul(Ak, self.state)
		
		Kk = np.matmul(self.Pk, np.matmul(self.Hk.T, np.linalg.inv(np.matmul(self.Hk, np.matmul(self.Pk, self.Hk.T)) + RK)))
		self.state = self.state + np.matmul(Kk, (Meas - np.matmul(self.Hk, self.state)))
		# print ("PIX_statePo",PIX_statePo.ravel())
		ImKkHk = (np.eye(Kk.shape[0]) - np.matmul(Kk, self.Hk))
		self.Pk = np.matmul(ImKkHk, np.matmul(self.Pk, ImKkHk.T)) + np.matmul(Kk,np.matmul(RK,Kk.T))
		self.PIX_stateE = self.state
		return self.state[0:2].ravel()

	# def Estimate(self,dt):
	# 	Ak = np.array([[1, 0.,dt, 0, dt**2/2,0],[0., 1.,0.0,  dt, 0,dt**2/2],[0., 0., 1., 0.,dt,0],[0., 0., 0., 1., 0.,dt],[0., 0., 0., 0., 1.,0],[0., 0., 0., 0., 0.,1]])
	# 	self.Pk = np.matmul(Ak, np.matmul(self.Pk, Ak.T)) + self.Qk
	# 	# Kk = np.matmul(self.Pk, np.matmul(self.Hk.T, np.linalg.inv(np.matmul(self.Hk, np.matmul(self.Pk, self.Hk.T)) + self.Rk)))
	# 	self.PIX_stateE = np.matmul(Ak, self.PIX_stateE)
	# 	return self.PIX_stateE[0:2].ravel()

class PointMass:
	def __init__(self):
		self.x = 0
		self.y=0
	def reset(self):
		self.x = 0
		self.y = 0
	def step(self,dt,u):
		self.x += u[0]*dt + np.random.randn()*0.01
		self.y += u[1]*dt + np.random.randn()*0.01
		return np.array([self.x,self.y])

class Car:
    def __init__(self):
        #car dimensions
        self.c1 = np.array([0.5,0.25])
        self.c2 = np.array([-0.5,0.25])
        self.c3 = np.array([-0.5,-0.25])
        self.c4 = np.array([0.5,-0.25])
    def rotate(self,theta,point):
        R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        return np.matmul(R,point.reshape(-1,1)).ravel()
    def body(self,pose):
        #get the car pose and orientation for plotting using controller outputs
        pose=pose.ravel()
        C1 = pose[:2] + self.rotate(pose[2],self.c1)
        C2 = pose[:2] + self.rotate(pose[2],self.c2)
        C3 = pose[:2] + self.rotate(pose[2],self.c3)
        C4 = pose[:2] + self.rotate(pose[2],self.c4)
        return np.array([C1,C2,C3,C4])
    def plot(self,pose):
        C=self.body(pose)
        plt.plot([C[0,0],C[1,0]],[C[0,1],C[1,1]], color='k', linewidth=2)
        plt.plot([C[1,0],C[2,0]],[C[1,1],C[2,1]], color='k', linewidth=2)
        plt.plot([C[2,0],C[3,0]],[C[2,1],C[3,1]], color="k", linewidth=2)
        plt.plot([C[3,0],C[0,0]],[C[3,1],C[0,1]], color='r', linewidth=2)
        plt.axis('equal')
        plt.axis([-10,10,-10,10])

        plt.pause(1e-6)
        plt.cla()

# meas = np.loadtxt("/home/yogesh/localization_ws/amcl_pose.txt")
meas = np.loadtxt("/home/yogesh/localization_ws/new_theta_jp_book.txt")

filter = Kalman()

def wrapAng(yaw):
    while abs(yaw)>2*np.pi:
        yaw = yaw - yaw/abs(yaw)*2*np.pi
    return yaw
def unwrapAngMeas(theta, theta_m):
    theta_ = wrapAng(theta)
    thetaDiff = theta_ - theta_m
    if abs(thetaDiff) < np.pi:
        return theta - thetaDiff
    else: 
        sign = thetaDiff/abs(thetaDiff)
        return theta + sign*np.pi*2 - thetaDiff 


theta = -365
theta_m = -5
yaw = unwrapAngMeas(np.radians(theta), np.radians(theta_m))
print (np.degrees(yaw))


dt =0.01
filtered = []
measurement = []
tac=0
time = []
# theta  = meas[0,3]
theta  = meas[0,1]
sig =1
# car = Car()
v = 0.5
x = 0
y = 0
x_ = lambda theta: v* np.cos(theta)
y_ = lambda theta: v* np.sin(theta)
for i in range(485):
    
#     stateMeas  = unwrapAngMeas(theta,sig*meas[i,3])
    stateMeas  = unwrapAngMeas(theta,sig*meas[i,1])
    
    tic = meas[i,0]
    if tac!=0:
        dt = tic-tac
    tac = tic*1
    theta = filter.Filter(0.01, stateMeas).ravel()[0]
    x+=x_(theta)*0.1
    y+=y_(theta)*0.1
    time.append(tic)
#     car.plot(np.array([x,y,theta]))
    measurement.append(stateMeas)
    filtered.append(theta)
    # X.append(np.array([0,0]))
time = np.array(time)
filtered= np.array(filtered)
measurement= np.array(measurement)
# plt.scatter(time[:],sig*meas[:,3],c="r", label="meas_1")
plt.scatter(time[:],sig*meas[:,1],c="r", label="meas_1")
plt.scatter(time[:],measurement[:],c="b", label="meas_2")
plt.scatter(time[:],filtered[:],c="k", label="filtered")
plt.legend()
plt.show()
