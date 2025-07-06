import numpy as np

class kfImu():
    def __init__(self):
        self.stateDim  = 2
        self.measurDim = 1
        self.dt        = 0.02

        self.states = np.zeros(self.stateDim)

        self.Qk = 0.0001*np.eye(self.stateDim)
        self.Pk = 100*np.eye(self.stateDim)

        self.Rk = 0.8*np.eye(self.measurDim)

    def stateTransModel(self, states):
        states = states.ravel()
        states[0] = states[0] + states[1]*self.dt

    def filter(self, measur):



