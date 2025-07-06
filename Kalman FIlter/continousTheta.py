import numpy as np
import matplotlib.pyplot as plt

class getThetaMeasured():
    def __init__(self):
        self.omega = 1.0
        self.theta = 0
        self.r = 2.0
        self.dt = 0.02

        self.x_axis = []
        self.theta_list = []
        self.theta_m_list = []

        self.getThetaM()

    def getThetaM(self, visualize=True):
        for i in range(500):
            self.theta = self.theta + self.omega * self.dt
            self.theta_m = self.theta + np.random.randn()
            x = self.r * np.cos(self.theta_m)
            y = self.r * np.sin(self.theta_m)
            self.theta_m = np.arctan2(y,x) #wrapped theta
            self.theta_m = self.unwrap_theta(self.theta, self.theta_m) #unwrapped theta
            if visualize:
                self.x_axis.append(i)
                self.theta_list.append(self.theta)
                self.theta_m_list.append(self.theta_m)

        if visualize:
            plt.plot(self.x_axis, self.theta_list, color='r')
            plt.scatter(self.x_axis, self.theta_m_list, color='b')
            plt.show()

    def unwrap_theta(self, theta, theta_m_wrap):
        diff = theta - theta_m_wrap
        while (diff > np.pi):
            diff -= 2*np.pi
        while(diff < -np.pi):
            diff += 2*np.pi
    
        return theta + diff

def main():
    getThetaMeasuredObj = getThetaMeasured()

if __name__ == '__main__':
    main()

# import numpy as np
# import matplotlib.pyplot as plt

# # Generate data
# uniform_data = np.random.rand(10000)      # Uniform between 0 and 1
# normal_data = np.random.randn(10000)      # Normal distribution with mean=0, std=1

# # Create subplots
# plt.figure(figsize=(12, 5))

# # Uniform Distribution Histogram
# plt.subplot(1, 2, 1)
# plt.hist(uniform_data, bins=50, color='skyblue', edgecolor='black')
# plt.title('Uniform Distribution (np.random.rand)')
# plt.xlabel('Value')
# plt.ylabel('Frequency')

# # Normal Distribution Histogram
# plt.subplot(1, 2, 2)
# plt.hist(normal_data, bins=50, color='salmon', edgecolor='black')
# plt.title('Normal Distribution (np.random.randn)')
# plt.xlabel('Value')
# plt.ylabel('Frequency')

# # Show the plots
# plt.tight_layout()
# plt.show()
