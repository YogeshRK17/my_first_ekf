# In this code we're implementing ekf without ros on static data
import numpy as np
import matplotlib.pyplot as plt

class sharedResources():
    def __init__(self):
        #STATES = [x, y, theta, vx, vy, omega, ax, ay, omega_dot]
                 # 0  1   2     3   4    5     6   7    8
        
        #set states matrix
        self.stateDim = 9
        self.states = np.zeros(self.stateDim)

        #process noise
        self.Qk = 0.000000001*np.eye(self.stateDim)

        #filter noise
        self.Pk = 100*np.eye(self.stateDim)

        self.dt = 0.02
        self.firstItr = True

    def stateTransModel(self, states):
        states = states.ravel()

        #general a.k.a linear equation is
        #x = x + vdt

        #more precise a.k.a non-linear equation is
        #x = x + (vx*cos(theta) - vy*sin(theta))dt
        #accordingly you have to write them for others

        return np.array([[states[0] + self.dt*(np.cos(states[2])*states[3] - np.sin(states[2])*states[4])],
                         [states[1] + self.dt*(np.sin(states[2])*states[3] + np.cos(states[2])*states[4])],
                         [states[2] + states[5]*self.dt],
                         [states[3] + states[6]*self.dt],
                         [states[4] + states[7]*self.dt],
                         [states[5] + states[8]*self.dt],
                         [states[6]],
                         [states[7]],
                         [states[8]]]).reshape(-1,1)
    
    def jacobian(self, states):
        states = states.ravel()
        jacobian = np.eye(self.stateDim)

        #  0  1  2  3  4  5  6  7  8
        #  x  y  t  vx vy w  ax ay w_d
        # [1. 0. 0. 0. 0. 0. 0. 0. 0.]
        # [0. 1. 0. 0. 0. 0. 0. 0. 0.]
        # [0. 0. 1. 0. 0. 0. 0. 0. 0.]
        # [0. 0. 0. 1. 0. 0. 0. 0. 0.]
        # [0. 0. 0. 0. 1. 0. 0. 0. 0.]
        # [0. 0. 0. 0. 0. 1. 0. 0. 0.]
        # [0. 0. 0. 0. 0. 0. 1. 0. 0.]
        # [0. 0. 0. 0. 0. 0. 0. 1. 0.]
        # [0. 0. 0. 0. 0. 0. 0. 0. 1.]

        #  0  1  2  3  4  5  6  7  8
        #  x  y  t  vx vy w  ax ay w_d
        # [1. 0. G  G  G  0. 0. 0. 0.]
        # [0. 1. G  G  G  0. 0. 0. 0.]
        # [0. 0. 1. 0. 0. G  0. 0. 0.]
        # [0. 0. 0. 1. 0. 0. G  0. 0.]
        # [0. 0. 0. 0. 1. 0. 0. G  0.]
        # [0. 0. 0. 0. 0. 1. 0. 0. G ]
        # [0. 0. 0. 0. 0. 0. 1. 0. 0.]
        # [0. 0. 0. 0. 0. 0. 0. 1. 0.]
        # [0. 0. 0. 0. 0. 0. 0. 0. 1.]
      
        #In first row, we have differentiated 'X' w.r.t other states
        jacobian[0,2] = - np.sin(states[2])*states[3]*self.dt - np.cos(states[2])*states[4]*self.dt  #d(x)/d(theta)
        jacobian[0,3] =   np.cos(states[2])*self.dt                                                  #d(x)/d(vx)
        jacobian[0,4] = - np.sin(states[2])*self.dt                                                  #d(x)/d(vy)

        #In second row, we have differentiated 'Y' w.r.t other states
        jacobian[1,2] = np.cos(states[2])*states[3]*self.dt - np.sin(states[2])*states[4]*self.dt
        jacobian[1,3] = np.sin(states[2])*self.dt
        jacobian[1,4] = np.cos(states[2])*self.dt

        #like-wise you have to go-ahead
        jacobian[2,5] = self.dt

        jacobian[3,6] = self.dt

        jacobian[4,7] = self.dt
        jacobian[5,8] = self.dt

        return jacobian
    
    def wrapYaw(self, states):
        if states[2,0]>np.pi:
            states[2,0] = states[2,0] - 2*np.pi
        if states[2,0]<-np.pi:
            states[2,0] = states[2,0] + 2*np.pi
        
        return states

class filter_laser(sharedResources):
    def __init__(self):
        super().__init__()

        self.laserDim = 3
        self.wheelDim = 3
        self.imuDIm   = 2
        self.Rk_laser = 5*np.eye(self.laserDim)
        self.Rk_wheel = 5*np.eye(self.wheelDim)
        self.Rk_imu   = 5*np.eye(self.imuDIm)

        self.stored_states = []
        self.covariance    = []

    def process_laser(self, laser_meas):
        if self.firstItr:
            self.firstItr = False
            self.lastTime = laser_meas[0]
            self.dt = 0.02
        else:
            self.dt = laser_meas[0] - self.lastTime
            # print(self.dt)
            self.lastTime = laser_meas[0]

        #let's define scale matrix to scale measurements
        self.Hk_laser = np.zeros((self.laserDim, self.stateDim))

        self.Hk_laser[0,0] = 1
        self.Hk_laser[1,1] = 1
        self.Hk_laser[2,2] = 1

        #get the predicted states
        self.states = self.wrapYaw(self.stateTransModel(self.states))

        #get the laser measurements
        #vx, vy, omega
        meas = np.array([laser_meas[1], laser_meas[2], laser_meas[3]]).reshape(-1,1)

        #get jacobain matrix 
        Fk = self.jacobian(self.states)

        #L matrix for matrix manipulation
        Lk = np.eye(self.stateDim)

        #get filter noise
        #P = F@P@F.T + L@Q@L.T
        self.Pk = Fk@(self.Pk@Fk.T) + Lk@(self.Qk@Lk.T)

        #M matrix for matrix manipulation
        Mk = np.eye(self.Rk_laser.shape[0])

        #get kalman gain
        #Kk = P@Hk.T/(Hk@P@Hk.T + Mk@Rk@Mk.T)
        Kk = self.Pk@(self.Hk_laser.T@(np.linalg.inv(self.Hk_laser@(self.Pk@self.Hk_laser.T) + Mk@(self.Rk_laser@Mk.T))))

        #update
        #state = state + Kk(meas - Hk@state)
        self.states = self.states + Kk@(meas - (self.Hk_laser@self.states))

        IKkHk = np.eye(Kk.shape[0]) - Kk@self.Hk_laser
        MkRkMkT = Mk@(self.Rk_laser@Mk.T)

        self.Pk = IKkHk@(self.Pk@IKkHk.T) + Kk@(MkRkMkT@Kk.T)

        # print(f"laser states shape: {self.states.shape}")
        self.stored_states.append(np.append(laser_meas[0], self.states.ravel()))
        self.covariance.append(np.append(laser_meas[0], self.Pk.ravel()))

    def process_wheel(self, wheel_meas):
        if self.firstItr:
            self.firstItr = False
            self.lastTime = wheel_meas[0]
            self.dt = 0.02
        else:
            self.dt = wheel_meas[0] - self.lastTime
            # print(self.dt)
            self.lastTime = wheel_meas[0]

        #let's define scale matrix to scale measurements
        self.Hk_wheel = np.zeros((self.wheelDim, self.stateDim))

        self.Hk_wheel[0,0] = 1
        self.Hk_wheel[1,1] = 1
        self.Hk_wheel[2,2] = 1

        #get the predicted states
        self.states = self.wrapYaw(self.stateTransModel(self.states))

        #get the laser measurements
        #vx, vy, omega
        meas = np.array([wheel_meas[1], wheel_meas[2], wheel_meas[3]]).reshape(-1,1)

        #get jacobain matrix 
        Fk = self.jacobian(self.states)

        #L matrix for matrix manipulation
        Lk = np.eye(self.stateDim)

        #get filter noise
        #P = F@P@F.T + L@Q@L.T
        self.Pk = Fk@(self.Pk@Fk.T) + Lk@(self.Qk@Lk.T)

        #M matrix for matrix manipulation
        Mk = np.eye(self.Rk_wheel.shape[0])

        #get kalman gain
        #Kk = P@Hk/(Hk@P@Hk.T + Mk@Rk@Mk.T)
        Kk = self.Pk@(self.Hk_wheel.T@(np.linalg.inv(self.Hk_wheel@(self.Pk@self.Hk_wheel.T) + Mk@(self.Rk_wheel@Mk.T))))

        #update
        #state = state + Kk(meas - Hk@state)
        self.states = self.states + Kk@(meas - (self.Hk_wheel@self.states))

        IKkHk = np.eye(Kk.shape[0]) - Kk@self.Hk_wheel
        MkRkMkT = Mk@(self.Rk_wheel@Mk.T)

        self.Pk = IKkHk@(self.Pk@IKkHk.T) + Kk@(MkRkMkT@Kk.T)

        # print(f"wheel states shape: {self.states.shape}")
        self.stored_states.append(np.append(wheel_meas[0], self.states.ravel()))
        self.covariance.append(np.append(wheel_meas[0], self.Pk.ravel()))

    def process_imu(self, imu_meas):
        if self.firstItr:
            self.firstItr = False
            self.lastTime = imu_meas[0]
            self.dt = 0.02
        else:
            self.dt = imu_meas[0] - self.lastTime
            # print(self.dt)
            self.lastTime = imu_meas[0]

        #let's define scale matrix to scale measurements
        self.Hk_imu = np.zeros((self.imuDIm, self.stateDim))

        self.Hk_imu[0,5] = 1
        self.Hk_imu[1,6] = 1

        #get the predicted states
        self.states = self.wrapYaw(self.stateTransModel(self.states))

        #get the laser measurements
        #vx, vy, omega
        meas = np.array([imu_meas[5], imu_meas[6]]).reshape(-1,1)

        #get jacobain matrix 
        Fk = self.jacobian(self.states)

        #L matrix for matrix manipulation
        Lk = np.eye(self.stateDim)

        #get filter noise
        #P = F@P@F.T + L@Q@L.T
        self.Pk = Fk@(self.Pk@Fk.T) + Lk@(self.Qk@Lk.T)

        #M matrix for matrix manipulation
        Mk = np.eye(self.Rk_imu.shape[0])

        #get kalman gain
        #Kk = P@Hk/(Hk@P@Hk.T + Mk@Rk@Mk.T)
        Kk = self.Pk@(self.Hk_imu.T@(np.linalg.inv(self.Hk_imu@(self.Pk@self.Hk_imu.T) + Mk@(self.Rk_imu@Mk.T))))

        #update
        #state = state + Kk(meas - Hk@state)
        self.states = self.states + Kk@(meas - (self.Hk_imu@self.states))

        IKkHk = np.eye(Kk.shape[0]) - Kk@self.Hk_imu
        MkRkMkT = Mk@(self.Rk_imu@Mk.T)

        self.Pk = IKkHk@(self.Pk@IKkHk.T) + Kk@(MkRkMkT@Kk.T)

        # print(f"imu states shape: {self.states.shape}")
        self.stored_states.append(np.append(imu_meas[0], self.states.ravel()))
        self.covariance.append(np.append(imu_meas[0], self.Pk.ravel()))

class readData():
    def __init__(self):
        self.no_of_measurements = None

    def get_data(self):
        self.data        = np.loadtxt("/home/yogesh/Documents/Kalman FIlter/measurements.txt") #(100,4)
        self.no_of_measurements = self.data.shape[0]

        timestamp = self.data[:,0].reshape(-1,1)
        x         = self.data[:,1].reshape(-1,1)
        y         = self.data[:,2].reshape(-1,1)
        theta     = self.data[:,3].reshape(-1,1)
        id        = self.data[:,4].reshape(-1,1)
        omega     = self.data[:,5].reshape(-1,1)
        ax        = self.data[:,6].reshape(-1,1)

        return np.hstack([timestamp, x, y, theta, id, omega, ax])
    
    def get_filtered_laser_data(self):
        self.filter_laser_data      = np.loadtxt("/home/yogesh/Documents/Kalman FIlter/laser_states.txt")
        timestamp = self.filter_laser_data[:,0].reshape(-1,1)
        x         = self.filter_laser_data[:,1].reshape(-1,1)
        y         = self.filter_laser_data[:,2].reshape(-1,1)
        theta     = self.filter_laser_data[:,3].reshape(-1,1)

        return np.hstack([timestamp, x, y, theta])

class plot():
    def __init__(self):
        pass

    def plot_laser_data(self):
        pred_states = np.loadtxt('/home/yogesh/Documents/Kalman FIlter/predicted_states.txt')
        measurements = np.loadtxt('/home/yogesh/Documents/Kalman FIlter/measurements.txt')
        filter       = np.loadtxt('/home/yogesh/Documents/Kalman FIlter/laser_wheel_states.txt')
        co_variance  = np.loadtxt('/home/yogesh/Documents/Kalman FIlter/co_variance.txt')

        plt.figure(0)
        plt.plot(pred_states[:,0],  pred_states[:,1],  color = 'r', label='x')
        plt.plot(measurements[:,0], measurements[:,1], color = 'r', label='x_noise')
        plt.plot(filter[:,0], filter[:,1], color = 'k', label='x_f')

        plt.figure(1)
        plt.plot(pred_states[:,0],  pred_states[:,2],  color = 'b',label='y')
        plt.plot(measurements[:,0], measurements[:,2], color = 'b',label='y_noise')
        plt.plot(filter[:,0], filter[:,2], color = 'k',label='y_f')

        plt.figure(2)
        plt.plot(pred_states[:,0],  pred_states[:,3],  color = 'g',label='theta')
        plt.plot(measurements[:,0], measurements[:,3], color = 'g',label='theta_noise')
        plt.plot(filter[:,0], filter[:,3], color = 'k',label='theta_f')

        plt.figure(3)
        plt.plot(co_variance[:,0], co_variance[:,1], color = 'k',label='x_cv')
        plt.plot(co_variance[:,0], co_variance[:,2], color = 'k',label='y_cv')
        plt.plot(co_variance[:,0], co_variance[:,3], color = 'k',label='theta_cv')

        plt.legend()
        plt.show()

class apply_filter(filter_laser):
    def __init__(self):
        filter_laser.__init__(self)
    
    def applyFilter(self):
        read_data = readData()
        measurement_data = read_data.get_data()

        # print(laser_data)

        print(read_data.no_of_measurements)
        for i in range(read_data.no_of_measurements):
            data = [measurement_data[i,0], measurement_data[i,1], measurement_data[i,2], measurement_data[i,3], measurement_data[i,4], measurement_data[i,5], measurement_data[i,6]]
            if data[4] == 0:
                self.process_laser(data)
            if data[4] == 1:
                self.process_wheel(data)
            if data[4] == 2:
                self.process_imu(data)

        np.savetxt('laser_wheel_states.txt', self.stored_states)
        np.savetxt('co_variance.txt', self.covariance)


def main():
    apply_filter_obj = apply_filter()
    apply_filter_obj.applyFilter()

    plot_data = plot()
    plot_data.plot_laser_data()

if __name__ == '__main__':
    main()
