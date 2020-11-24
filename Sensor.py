if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import math

class Sensor(object):
    def __init__(self, robot):
        self.robot = robot

class IMU(Sensor):
    ### could return x, y, and theta
    def __init__(self, robot):
        Sensor.__init__(self, robot)
        self.Q = array([[4.87e-1, -5.86e-3, -5.86e-4], 
                        [-5.86e-3, 4.87e-1, -5.86e-4],
                        [-5.86e-4, -5.86e-4, 4.87e-2]])
        self.C = array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
        def h(x):
            ## Sensor function
            return array([x[0], 
                          x[1], 
                          x[2]])
        self.h = h
        ## Linearized sensor function
        self.H = lambda x : array([[1, 0, 0],
                                   [0, 1, 0],
                                   [0, 0, 1]])
    def observe(self):
        T = self.robot.GetTransform()
        observation_noise = random.multivariate_normal((0, 0, 0), self.Q)
        observation = array([T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0]) ]) + observation_noise
        return observation

class GPS(Sensor):
    ### could return x, y
    def __init__(self, robot):
        Sensor.__init__(self, robot)
        self.Q = array([[4.87e-1, -5.86e-3, 0],
                        [-5.86e-3, 4.87e-1, 0],
                        [0, 0, 1]])
        self.C = array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
        def h(x):
            ## Sensor function
            return array([x[0], 
                          x[1], 
                          x[2]])
        self.h = h
        ## Linearized sensor function
        self.H = lambda x : array([[1, 0, 0],
                                   [0, 1, 0],
                                   [0, 0, 1]])
    def observe(self):
        T = self.robot.GetTransform()
        observation_noise = random.multivariate_normal((0, 0, 0), self.Q)
        observation = array([T[0, 3], T[1, 3], 0]) + observation_noise
        return observation