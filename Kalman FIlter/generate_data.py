import numpy as np
import matplotlib.pyplot as plt
import time


class generateData():
    def __init__(self):

        self.no_of_samples = 100

        self.stateDim = 9
        self.states = np.zeros(self.stateDim)
        # self.states[0,] = 1
        # self.states[1,] = 1
        # self.states[2,] = 1
        self.states[3,] = 2
        self.states[4,] = 4
        self.states[5,] = 1.5
        # self.states[6,] = 1
        # self.states[7,] = 1
        # self.states[8,] = 1

        self.dt = 0.02

        self.pred_states  = []
        self.measurements = []

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

    def wrapYaw(self, states):
        if states[2,0]>np.pi:
            states[2,0] = states[2,0] - 2*np.pi
        if states[2,0]<-np.pi:
            states[2,0] = states[2,0] + 2*np.pi
        
        return states

    def get_prediction(self):
        for i in range(self.no_of_samples):
            self.states = self.wrapYaw(self.stateTransModel(self.states))

            self.pred_states.append(np.append(time.time(), self.states.ravel()))

        np.savetxt("predicted_states.txt", self.pred_states)

        print("file has been saved successfully...")

    def get_measurements(self):

        pred_states = np.loadtxt('/home/yogesh/Documents/Kalman FIlter/predicted_states.txt')
        
        for i in range(self.no_of_samples):
            x     = pred_states[i,1]
            y     = pred_states[i,2]
            theta = pred_states[i,3]

            x_noise     = x     + 0.1*np.random.randn()
            y_noise     = y     + 0.1*np.random.randn()
            theta_noise = theta + 0.1*np.random.randn()

            self.measurements.append(np.append(pred_states[i,0], [x_noise, y_noise, theta_noise]))
        
        np.savetxt('measurements.txt', self.measurements)
        
class car():
    def __init__(self):
        self.c1 = np.array([0.5, 0.25])
        self.c2 = np.array([-0.5, 0.25])
        self.c3 = np.array([-0.5, -0.25])
        self.c4 = np.array([0.5, -0.25])

    def rotate(self, theta, point):
        R = np.array([[np.cos(theta), -np.sin(theta)], 
                      [np.sin(theta),  np.cos(theta)]])
        
        return np.matmul(R,point.reshape(-1,1)).ravel()
    
    def body(self, pose):
        pose = pose.ravel()

        C1 = pose[:2] + self.rotate(pose[2], self.c1)
        C2 = pose[:2] + self.rotate(pose[2], self.c2)
        C3 = pose[:2] + self.rotate(pose[2], self.c3)
        C4 = pose[:2] + self.rotate(pose[2], self.c4)

        return np.array([C1, C2, C3, C4])
    
    def show_trajectory(self, pose):
        pose = pose.ravel()
        plt.scatter(pose[0], pose[1], color = 'b')
        # plt.plot(pose[0], pose[1], color = 'b')
    
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
        self.show_trajectory(pose)

def main():
    generate_data = generateData()
    car_obj = car()

    generate_data.get_prediction()
    generate_data.get_measurements()

    #For car simulation
    # pred_states = np.array(generate_data.pred_states)

    # for i in range(no_of_samples):
    #     pose = np.array([pred_states[i,1], pred_states[i,2], pred_states[i,3]])
    #     car_obj.plot(pose)

if __name__ == '__main__':
    main()

