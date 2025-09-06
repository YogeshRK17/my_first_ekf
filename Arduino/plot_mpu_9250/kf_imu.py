import numpy as np
import matplotlib.pyplot as plt

class kfImu_pitch():
    def __init__(self):
        self.stateDim  = 2
        self.measurDim = 1
        self.dt        = 0.02
        self.firstItr  = True

        # [angle, omega]
        self.states = np.zeros(self.stateDim)

        self.Qk = 0.001*np.eye(self.stateDim)
        self.Pk = 100*np.eye(self.stateDim)

        self.Rk_acc = 0.08*np.eye(self.measurDim)
        self.Rk_gro = 1.0*np.eye(self.measurDim)

        self.A  = np.array([[1, self.dt], [0, 1]]) 

        self.store_states = []

    def stateTransModel(self, states):
        states = states.ravel()

        return np.array([states[0] + states[1]*self.dt, states[1]]).reshape(-1,1)

    def filter_pitch(self, measur):
        #measur = [time, pitch, sensor_id]
        #acc = sensor_id = 0
        #gyro +sensor_id = 2
        if self.firstItr:
            self.firstItr = False
            self.lastTime = measur[0]
        else:
            self.dt = measur[0] - self.lastTime
            self.lastTime = measur[0]

        self.Hk = np.zeros((self.measurDim, self.stateDim))
        self.Hk[0,0] = 1

        #prediction step:
        self.states = self.stateTransModel(self.states)
        self.Pk = self.A@(self.Pk@self.A.T) + self.Qk

        #measurement and update step:
        meas = np.array([measur[1]]).reshape(-1,1)

        if measur[2] == 0:
            self.Rk = self.Rk_acc
        else:
            self.Rk = self.Rk_gro

        Kk = self.Pk @ (self.Hk.T @ (np.linalg.inv(self.Hk @ (self.Pk @ self.Hk.T) + self.Rk)))

        scaled_meas = meas - self.Hk@self.states

        self.states = self.states + Kk @ scaled_meas

        IKH = np.eye(Kk.shape[0]) - Kk@self.Hk

        self.Pk = IKH@(self.Pk@IKH.T) +  Kk@(self.Rk@Kk.T)

        self.store_states.append(np.append(measur[0], self.states.ravel()))

        states = self.states.ravel()
        return states[0]

class kfImu_roll():
    def __init__(self):
        self.stateDim  = 2
        self.measurDim = 1
        self.dt        = 0.02
        self.firstItr  = True

        # [angle, omega]
        self.states = np.zeros(self.stateDim)

        self.Qk = 0.001*np.eye(self.stateDim)
        self.Pk = 100*np.eye(self.stateDim)

        self.Rk_acc = 0.08*np.eye(self.measurDim)
        self.Rk_gro = 1.0*np.eye(self.measurDim)

        self.A  = np.array([[1, self.dt], [0, 1]]) 

        self.store_states = []

    def stateTransModel(self, states):
        states = states.ravel()

        return np.array([states[0] + states[1]*self.dt, states[1]]).reshape(-1,1)

    def filter_roll(self, measur):
        #measur = [time, pitch, sensor_id]
        #acc = sensor_id = 0
        #gyro +sensor_id = 2
        if self.firstItr:
            self.firstItr = False
            self.lastTime = measur[0]
        else:
            self.dt = measur[0] - self.lastTime
            self.lastTime = measur[0]

        self.Hk = np.zeros((self.measurDim, self.stateDim))
        self.Hk[0,0] = 1

        #prediction step:
        self.states = self.stateTransModel(self.states)
        self.Pk = self.A@(self.Pk@self.A.T) + self.Qk

        #measurement and update step:
        meas = np.array([measur[1]]).reshape(-1,1)

        if measur[2] == 0:
            self.Rk = self.Rk_acc
        else:
            self.Rk = self.Rk_gro

        Kk = self.Pk @ (self.Hk.T @ (np.linalg.inv(self.Hk @ (self.Pk @ self.Hk.T) + self.Rk)))

        scaled_meas = meas - self.Hk@self.states

        self.states = self.states + Kk @ scaled_meas

        IKH = np.eye(Kk.shape[0]) - Kk@self.Hk

        self.Pk = IKH@(self.Pk@IKH.T) +  Kk@(self.Rk@Kk.T)

        self.store_states.append(np.append(measur[0], self.states.ravel()))

        states = self.states.ravel()
        return states[0]

def main():
    kf_filter = kfImu_pitch()

    #Dummy testing
    measurements = [
        [12.57, 0.5099],
        [12.58, 0.5074],
        [12.59, 0.5334],
        [12.6, 0.5611],
        [12.61, 0.5361],
        [12.62, 0.5623],
        [12.63, 0.5537],
        [12.64, 0.5858],
        [12.65, 0.5841],
        [12.66, 0.5996],
        [12.67, 0.6131],
        [12.68, 0.6249],
        [12.69, 0.6375],
        [12.7, 0.6217],
        [12.71, 0.6528],
        [12.72, 0.6566],
        [12.73, 0.6738],
        [12.74, 0.6729],
        [12.75, 0.6829],
        [12.76, 0.701],
        [12.77, 0.7033],
        [12.78, 0.7241],
        [12.79, 0.7383],
        [12.8, 0.723],
        [12.81, 0.7384],
        [12.82, 0.7476],
        [12.83, 0.7667],
        [12.84, 0.7607],
        [12.85, 0.7867],
        [12.86, 0.7747],
        [12.87, 0.8024],
        [12.88, 0.7904],
        [12.89, 0.8158],
        [12.9, 0.8181],
        [12.91, 0.8264],
        [12.92, 0.8381],
        [12.93, 0.8423],
        [12.94, 0.8428],
        [12.95, 0.8642],
        [12.96, 0.8581],
        [12.97, 0.8766],
        [12.98, 0.8747],
        [12.99, 0.8937],
        [13.0, 0.9002],
        [13.01, 0.9083],
        [13.02, 0.9106],
        [13.03, 0.9267],
        [13.04, 0.9315],
        [13.05, 0.9353],
        [13.06, 0.9583],
        [13.07, 0.9713]
    ]

    for measurement in measurements:
        kf_filter.filter(measurement)

    np.savetxt("states.txt", kf_filter.store_states)

# Convert to numpy arrays
# Time
    time = np.array([
        12.57, 12.58, 12.59, 12.60, 12.61, 12.62, 12.63, 12.64, 12.65, 12.66,
        12.67, 12.68, 12.69, 12.70, 12.71, 12.72, 12.73, 12.74, 12.75, 12.76,
        12.77, 12.78, 12.79, 12.80, 12.81, 12.82, 12.83, 12.84, 12.85, 12.86,
        12.87, 12.88, 12.89, 12.90, 12.91, 12.92, 12.93, 12.94, 12.95, 12.96,
        12.97, 12.98, 12.99, 13.00, 13.01, 13.02, 13.03, 13.04, 13.05, 13.06,
        13.07
    ]).reshape(-1, 1)

    # Yaw
    yaw = np.array([
        0.5099, 0.5074, 0.5334, 0.5611, 0.5361, 0.5623, 0.5537, 0.5858, 0.5841, 0.5996,
        0.6131, 0.6249, 0.6375, 0.6217, 0.6528, 0.6566, 0.6738, 0.6729, 0.6829, 0.701,
        0.7033, 0.7241, 0.7383, 0.723,  0.7384, 0.7476, 0.7667, 0.7607, 0.7867, 0.7747,
        0.8024, 0.7904, 0.8158, 0.8181, 0.8264, 0.8381, 0.8423, 0.8428, 0.8642, 0.8581,
        0.8766, 0.8747, 0.8937, 0.9002, 0.9083, 0.9106, 0.9267, 0.9315, 0.9353, 0.9583,
        0.9713
    ]).reshape(-1, 1)

    measurements = np.hstack([time, yaw])
    states = np.loadtxt('/home/yogesh/Documents/Arduino/plot_mpu_9250/states.txt')
    plt.plot(measurements[:,0], measurements[:,1])
    plt.plot(states[:,0], states[:,1])
    plt.show()

if __name__ == "__main__":
    main()
        


        




