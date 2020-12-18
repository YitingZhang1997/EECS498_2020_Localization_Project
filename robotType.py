if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
from Sensor import *
from Astar import Astar

class Robot(object):
    def __init__(self, robot, state, env, sensor = GPS, search_alg = Astar):
        ###
        # Class Robot
        # robot is the robot object in openrave
        # state is the x, y, theta(orientation) of the robot
        # sensor is a Sensor type, could be GPS, IMU....
        ###
        self.robot = robot
        self.state = array(state)
        self.sensor = sensor(self.robot, env)
        self.observation = self.sensor.observe()
        self.search_alg = search_alg
        self.env = env
    def update(self, env, handles, DRAWOB = True, DRAWTR = True):
        ###
        # update robot's location in the environment
        # DRAWTR: "True" draw the robot trajectory on the map
        # DRAWOB: "True" draw the robot obsevation on the map, only for GPS and IMU
        ###
        T = array([[cos(self.state[2]), -sin(self.state[2]), 0, self.state[0]],
                   [sin(self.state[2]), cos(self.state[2]), 0, self.state[1]],
                   [0, 0, 1, 0.05],
                   [0, 0, 0, 1]])
        self.robot.SetTransform(T)
        ## draw robot true trajectory
        if DRAWTR:
            handles.append(env.plot3(points=array([self.state[0], self.state[1], 0.05]),
                                    pointsize=4.0,
                                    colors=array(((0,0,1)))))
        ## draw sensor observation trajectory
        self.observation = self.sensor.observe()
        if (type(self.sensor) == GPS or type(self.sensor) == IMU) and DRAWOB:
            handles.append(env.plot3(points=array([self.observation[0], self.observation[1], 0.05]),
                                    pointsize=2.0,
                                    colors=array(((1,0,0)))))
        ## update which landMark is working
        if type(self.sensor) == LandMark:
            for i in range(self.sensor.n):
                xl = self.sensor.LandMarkLocation[i, 0]
                yl = self.sensor.LandMarkLocation[i, 1]
                handles.append(env.plot3(points=array([xl, yl, 1]),
                                        pointsize=10.0,
                                        colors=array(((1,0,0)))))
    def search_path(self, handles, goalconfig, type = "4-connected"):
        path = Astar(self.robot,goalconfig,self.env,handles, type)
        path = flip(path, 0)
        return path





class SimpleDynamicRobot(Robot):
    ###
    # A simple dynamic robot class
    ###
    def __init__(self, robot, state, env, sensor = GPS, search_alg = Astar):
        Robot.__init__(self, robot, state, env, sensor, search_alg)
        self.input = array([0, 0, 0])
        self.R = array([[2.5e-3, 1.8e-5, 1.8e-6],
                        [1.8e-5, 2.5e-3, 1.8e-6],
                        [1.8e-6, 1.8e-6, 2.5e-4]])
        self.A = eye(3)
        self.B = eye(3)
    def g(self, u, x):
        ## Dynamic function
        return array([x[0] + u[0],
                      x[1] + u[1],
                      x[2] + u[2]])

    def G(self, u ,x):
        ## Linearized dynamic function
        return array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

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
        # temp_turn_theta = 0
        while collision:
            # temp_turn_theta += pi/2
            # Rot = array([[cos(temp_turn_theta), sin(temp_turn_theta), 0],
            #             [-sin(temp_turn_theta), cos(temp_turn_theta), 0],
            #             [0, 0, 1]])
            # temp_input = 4 * dot(Rot, self.input)
            # temp_state = self.state + temp_input + prediction_noise
            # T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
            #            [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
            #            [0, 0, 1, 0.05],
            #            [0, 0, 0, 1]])
            # self.robot.SetTransform(T)
            # collision = env.CheckCollision(self.robot)
            '''
            when collision, just try the input again. 
            since the input is calculated by A*, in theory the robot would not in collsion.
            So just try another time, and for the next time, the noise would be different.
            There would be a noise that could let the robot collision-free
            '''
            prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
            temp_state = self.state + temp_input + prediction_noise
            T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                       [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])
            self.robot.SetTransform(T)
            collision = env.CheckCollision(self.robot)
        self.state = temp_state
        self.input = temp_input
    def path2input(self, path):
        input = path[1:,:] - path[0:-1,:]
        return input


class GoForwardDynamicRobot(Robot):
    ###
    # A robot class, could only go directly
    ###
    def __init__(self, robot, state, env, sensor = GPS, search_alg = Astar):
        Robot.__init__(self, robot, state, env, sensor, search_alg)
        self.input = array([0, 0])
        self.R = array([[2.5e-3, 1.8e-5, 1.8e-6],
                        [1.8e-5, 2.5e-3, 1.8e-6],
                        [1.8e-6, 1.8e-6, 2.5e-4]])
    def g(self, u, x):
        ## Dynamic function
        return array([x[0] + u[0] * cos(x[2]),
                      x[1] + u[0] * sin(x[2]),
                      x[2] + u[1]])

    def G(self, u ,x):
        ## Linearized dynamic function
        return array([[1, 0, - sin(x[2]) * u[0]],
                      [0, 1,   cos(x[2]) * u[0]],
                      [0, 0,          1        ]])

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
            prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
            temp_turn_theta += pi/16
            temp_input = array([-self.input[0], self.input[1]-temp_turn_theta])
            temp_state = self.g(temp_input, self.state) + prediction_noise
            T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
                       [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])
            # prediction_noise = random.multivariate_normal((0, 0, 0), self.R)
            # temp_state = self.g(temp_input, self.state) + prediction_noise
            # T = array([[cos(temp_state[2]), -sin(temp_state[2]), 0, temp_state[0]],
            #        [sin(temp_state[2]), cos(temp_state[2]), 0, temp_state[1]],
            #        [0, 0, 1, 0.05],
            #        [0, 0, 0, 1]])
            self.robot.SetTransform(T)
            collision = env.CheckCollision(self.robot)
        self.state = temp_state
        self.input = temp_input
