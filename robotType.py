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
    def predict(self, env):
        prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
        # add check collision
        # self.state = self.state + self.input + prediction_noise
        temp_input = self.input.copy()
        temp_state = self.state + temp_input + prediction_noise
        T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                       [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])
        self.robot.SetTransform(T)
        collision = env.CheckCollision(self.robot)
        temp_turn_theta = 0
        while collision:
            temp_turn_theta += pi/2
            Rot = array([[cos(temp_turn_theta), sin(temp_turn_theta), 0],
                        [-sin(temp_turn_theta), cos(temp_turn_theta), 0],
                        [0, 0, 1]])
            temp_input = dot(Rot, self.input)
            temp_state = self.state + temp_input + prediction_noise
            T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                       [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])
            self.robot.SetTransform(T)
            collision = env.CheckCollision(self.robot)
        self.state = temp_state
        self.input = temp_input

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

    def predict(self, env):
        prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
        # add check collision
        # self.state = self.g(self.input, self.state) + prediction_noise
        temp_input = self.input.copy()
        temp_state = self.g(temp_input, self.state) + prediction_noise
        T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                   [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                   [0, 0, 1, 0.05],
                   [0, 0, 0, 1]])
        self.robot.SetTransform(T)
        collision = env.CheckCollision(self.robot)
        temp_turn_theta = 0

        while collision:
            temp_turn_theta += pi/2
            temp_input = array([-4*self.input[0], self.input[1]-temp_turn_theta])
            temp_state = self.g(temp_input, self.state) + prediction_noise
            T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                       [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])
            self.robot.SetTransform(T)
            collision = env.CheckCollision(self.robot)
        self.state = temp_state
        self.input = temp_input

