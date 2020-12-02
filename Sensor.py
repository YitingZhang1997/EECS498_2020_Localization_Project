
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import math
import time
class MySensor(object):
    def __init__(self, robot):
        self.robot = robot

class IMU(MySensor):
    ### could return x, y, and theta
    def __init__(self, robot, env):
        MySensor.__init__(self, robot)
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

class GPS(MySensor):
    ### could return x, y
    def __init__(self, robot, env):
        MySensor.__init__(self, robot)
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

class LandMark(MySensor):
    '''
    LandMark Sensor, return the direction and distance between robot and landmarks
    '''
    def __init__(self, robot, env):
        MySensor.__init__(self, robot)

        # self.LandMarkLocation = array([[0, 2.0],
        #                                [4.0, 2.0],
        #                                [4.0, -2.0],
        #                                [0.0, -2.0],
        #                                [-4.0, -2.0],
        #                                [-4.0, 2.0]])
        self.LandMarkLocation = array([[4.0, 2.0]])
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

class Laser(MySensor):
    '''
    define a Laser Sensor class
    Variables:
        self.n: the number of the laser beams
        self.Q: the coevariance matrix of sensor, size [n, n]
    '''
    def __init__(self, robot, env):
        MySensor.__init__(self, robot)
        self.env = env
        self.sensor = env.GetSensors()[0]
        self.sensor.Configure(Sensor.ConfigureCommand.PowerOn)
        self.sensor.Configure(Sensor.ConfigureCommand.RenderDataOn)
        self.n = self.sensor.GetSensorData(Sensor.Type.Laser).ranges.shape[0]
        self.Q = eye(self.n)
        self.olddata = 0
        for i in range(self.n):
            self.Q[i, i] = 1e-2

    def h(self, x):
        '''
        return the data given by the laser sensor
        Input:
            x: the state of the robot, [x, y, theta]
        Output:
            output: distance from the obstacle returned by laser beams in to a verctor
                    shape [n], [sqrt(x1**2+y1**2), sqrt(x2**2+y2**2) ... sqrt(xn**2+yn**2)]
        '''
        T = self.robot.GetTransform()
        T_temp = array([[cos(x[2]), -sin(x[2]), 0, x[0]],
                        [sin(x[2]), cos(x[2]), 0, x[1]],
                        [0, 0, 1, 0.05],
                        [0, 0, 0, 1]])
        self.robot.SetTransform(T_temp)
        ## GetSensorData could not written in self.env, unless it won't update
        while True:
            data = self.sensor.GetSensorData(Sensor.Type.Laser)
            if self.olddata == 0 or self.olddata.stamp != data.stamp or (T == T_temp).all():
                break
            # time.sleep(0.01)
        self.olddata = data
        output = sqrt(data.ranges[:, 0] ** 2 + data.ranges[:, 1] ** 2)
        # self.robot.SetTransform(T)
        return output

    def observe(self):
        T = self.robot.GetTransform()
        mean = zeros((self.n,))
        observation_noise = random.multivariate_normal(mean, self.Q)
        # observation = self.h(array([T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0]) ])) + observation_noise
        observation = self.h(array([T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])]))
        # self.index += 1
        # if self.index == self.LandMarkLocation.shape[0]:
        #     self.index = 0
        return observation


