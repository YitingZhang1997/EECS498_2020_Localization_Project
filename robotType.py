if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from Sensor import *

class Robot(object):
    def __init__(self, robot, state, sensor = GPS):
        ###
        # Class Robot
        # robot is the robot object in openrave
        # state is the x, y, theta(orientation) of the robot
        # sensor is a Sensor type, could be GPS, IMU....
        ###
        self.robot = robot
        self.state = state 
        self.sensor = sensor(self.robot)
        self.observation = self.sensor.observe()

    def update(self, env, handles):
        ###
        # update robot's location in the environment
        ###
        T = array([[cos(self.state[2]), -sin(self.state[2]), 0, self.state[0]],
                   [sin(self.state[2]), cos(self.state[2]), 0, self.state[1]],
                   [0, 0, 1, 0.05],
                   [0, 0, 0, 1]])
        self.robot.SetTransform(T)
        ## draw robot true trajectory
        handles.append(env.plot3(points=array([self.state[0], self.state[1], 0.05]),
                                pointsize=4.0,
                                colors=array(((0,0,1)))))
        ## draw sensor observation trajectory
        self.observation = self.sensor.observe()
        handles.append(env.plot3(points=array([self.observation[0], self.observation[1], 0.05]),
                                pointsize=1.0,
                                colors=array(((1,0,0)))))



class SimpleDynamicRobot(Robot):
    ###
    # A simple dynamic robot class
    ###
    def __init__(self, robot, state, sensor = GPS):
        Robot.__init__(self, robot, state, sensor)
        self.input = array([0, 0, 0])
        self.R = array([[2.5e-3, 1.8e-5, 1.8e-6], 
                        [1.8e-5, 2.5e-3, 1.8e-6], 
                        [1.8e-6, 1.8e-6, 2.5e-4]])
        self.A = eye(3)
        self.B = eye(3)
    def predict(self):
        prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
        self.state = self.state + self.input + prediction_noise
    
class GoForwardDynamicRobot(Robot):
    ###
    # A robot class, could only go directly
    ###
    def __init__(self, robot, state, sensor = GPS):
        Robot.__init__(self, robot, state, sensor)
        self.input = array([0, 0])
        self.R = array([[2.5e-3, 1.8e-5, 1.8e-6], 
                        [1.8e-5, 2.5e-3, 1.8e-6], 
                        [1.8e-6, 1.8e-6, 2.5e-4]])
        def g(u, x):
            ## Dynamic function
            return array([x[0] + u[0] * cos(x[2]), 
                          x[1] + u[0] * sin(x[2]), 
                          x[2] + u[1]])
        self.g = g

        def G(u ,x):
            ## Linearized dynamic function
            return array([[1, 0, - sin(x[2]) * u[0]],
                          [0, 1,   cos(x[2]) * u[0]],
                          [0, 0,          1        ]])
        self.G = G

    def predict(self):
        prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
        self.state = self.g(self.input, self.state) + prediction_noise
