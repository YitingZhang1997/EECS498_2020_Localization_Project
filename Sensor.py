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
    def h(self, x):
        ## Sensor function
        return array([x[0],
                      x[1],
                      x[2]])

    def H(self, x):
        ## Linearized sensor function
        return array([[1, 0, 0],
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
        # self.Q = array([[4.87e-1, -5.86e-3, 0],
        #                 [-5.86e-3, 4.87e-1, 0],
        #                 [0, 0, 1]])
        # self.C = array([[1, 0, 0],
        #                 [0, 1, 0],
        #                 [0, 0, 1]])
        self.Q = array([[4.87e-1, -5.86e-3],
                        [-5.86e-3, 4.87e-1]])
        self.C = array([[1, 0, 0],
                        [0, 1, 0]])
    def h(self, x):
        ## Sensor function
        # return array([x[0],
        #               x[1],
        #               x[2]])
        return array([x[0],
                      x[1]])

    def H(self, x):
        ## Linearized sensor function
        # return array([[1, 0, 0],
        #               [0, 1, 0],
        #               [0, 0, 1]])
        return array([[1, 0, 0],
                      [0, 1, 0]])
    def observe(self):
        T = self.robot.GetTransform()
        observation_noise = random.multivariate_normal((0, 0), self.Q)
        observation = array([T[0, 3], T[1, 3]]) + observation_noise
        return observation

class LandMark(Sensor):
    '''
    LandMark Sensor, return the direction and distance between robot and landmarks
    '''
    def __init__(self, robot):
        Sensor.__init__(self, robot)

        self.LandMarkLocation = array([[0, 2.0],
                                       [4.0, 2.0],
                                       [4.0, -2.0],
                                       [0.0, -2.0],
                                       [-4.0, -2.0],
                                       [-4.0, 2.0]])
        # self.LandMarkLocation = array([[4.0, 2.0]])
        self.n = self.LandMarkLocation.shape[0]
        self.Q = eye(2 * self.n)
        for i in range(self.n):
            self.Q[2 * i, 2 * i] = 4.87e-2
            self.Q[2 * i, 2 * i + 1] = -5.86e-3
            self.Q[2 * i + 1, 2 * i] = -5.86e-3
            self.Q[2 * i + 1, 2 * i + 1] = 4.87e-1
        # ### define which LandMark to use
        # self.index = 0
    def h(self, x):
        ## Sensor function
        result = zeros((2 * self.n,))
        for i in range(self.n):
            xl = self.LandMarkLocation[i, 0]
            yl = self.LandMarkLocation[i, 1]
            result[2 * i] = sqrt((xl - x[0]) ** 2 + (yl - x[1]) ** 2)
            result[2 * i + 1] = math.atan2(x[1] - yl, x[0] - xl)
        return result

        # xl = self.LandMarkLocation[self.index, 0]
        # yl = self.LandMarkLocation[self.index, 1]
        # return array([sqrt((xl - x[0])**2 + (yl - x[1])**2),
        #               math.atan2(x[1] - yl, x[0] - xl)])

    def H(self, x):
        ## Linearized sensor function
        result = zeros((2 * self.n, 3))
        for i in range(self.n):
            xl = self.LandMarkLocation[i, 0]
            yl = self.LandMarkLocation[i, 1]
            result[2 * i, 0] = (x[0] - xl)/sqrt((xl - x[0])**2 + (yl - x[1])**2)
            result[2 * i, 1] = (x[1] - yl)/sqrt((xl - x[0])**2 + (yl - x[1])**2)
            result[2 * i + 1, 0] = -(x[1] - yl)/((xl - x[0])**2 + (yl - x[1])**2)
            result[2 * i + 1, 1] = (x[0] - xl)/((xl - x[0])**2 + (yl - x[1])**2)
        return result

        # xl = self.LandMarkLocation[self.index, 0]
        # yl = self.LandMarkLocation[self.index, 1]
        # return array([[(x[0] - xl)/sqrt((xl - x[0])**2 + (yl - x[1])**2), (x[1] - yl)/sqrt((xl - x[0])**2 + (yl - x[1])**2), 0],
        #               [-(x[1] - yl)/((xl - x[0])**2 + (yl - x[1])**2), (x[0] - xl)/((xl - x[0])**2 + (yl - x[1])**2), 0]])

    def observe(self):
        T = self.robot.GetTransform()
        mean = zeros((2 * self.n,))
        observation_noise = random.multivariate_normal(mean, self.Q)
        observation = self.h(array([T[0, 3], T[1, 3]])) + observation_noise
        # self.index += 1
        # if self.index == self.LandMarkLocation.shape[0]:
        #     self.index = 0
        return observation