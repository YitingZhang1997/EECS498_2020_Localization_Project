import time
import os
from init import initRobot
from init import simpleDynamicinputs, goforwardDynamicinputs
import robotType
from KalmanFilter import KF, EKF
from ParticleFilter import PF
from Sensor import *
from Astar import Astar
from PID import *
import math

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
if __name__ == "__main__":
    path =  os.getcwd()

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()

    handles = []
    ##############################        CASE0.1         ##############################
    #### show the system has some noise
    env_path = os.path.join(path, "map/testScene.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("##############################        CASE0.1         ##############################")
    print("###################        show the noise of the system         ####################")
    print("blue point is the real state, black point is the desire state, red point is the sensor data")
    print("Robot Dynamic: simpledynamicrobot")
    print("Sensor: self-defined sensor(GPS-like sensor, but could also sense robot's facing direction)")
    print("Map: empty map")
    simplerobot = robotType.SimpleDynamicRobot(robot, [-6, 0, 0], env, sensor=GPS)
    simplerobot.update(env, handles)
    current_point = array([-6., 0., 0.])
    for ii in range(simpleDynamicinputs.shape[0]):
        handles.append(env.plot3(points=array([current_point[0], current_point[1], 0.05]),
                                 pointsize=4.0,
                                 colors=array(((0, 0, 0)))))
        current_point += simpleDynamicinputs[ii, :]
        simplerobot.input = simpleDynamicinputs[ii, :]
        simplerobot.predict(env)
        simplerobot.update(env, handles)
        time.sleep(0.05)
    print("Finish case0.1!")
    print("\n\n\n")
    time.sleep(3)
    ##############################        CASE1.1         ##############################
    ######## robot dynamic: goforwardrobot
    ######## sensor: self-defined sensor
    ######## map: obstacles
    ######## estimation: EKF
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("##############################        CASE1.1         ##############################")
    print("#################        Extended Kalman Filter Localization       #################")
    print("Robot Dynamic: goforwardrobot(the robot could only go along its face direction)")
    print("Sensor: self-defined sensor(GPS-like sensor, but could also sense robot's facing direction)")
    print("Map: obstacles map")
    print("Estimation Algorithm: Extended Kalman Filter")
    print("blue point is the real state, green point is the predict state, red point is the sensor data")
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=IMU)
    goforwardrobot.update(env, handles)
    ekf = EKF(goforwardrobot)
    for ii in range(goforwardDynamicinputs.shape[0]):
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        ekf.update(env, handles)
        time.sleep(0.025)
    print("Finish case1.1!")
    print("\n\n\n")
    time.sleep(3)
    ##############################        CASE1.2         ##############################
    ######## robot dynamic: goforwardrobot
    ######## sensor: self-defined sensor
    ######## map: obstacles
    ######## estimation: PF
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("##############################        CASE1.2         ################################")
    print("####################          Particle Filter Localization        ####################")
    print("Robot Dynamic: goforwardrobot(the robot could only go along its face direction)")
    print("Sensor: self-defined sensor(GPS-like sensor, but could also sense robot's facing direction)")
    print("Map: obstacles map")
    print("Estimation Algorithm: Particle Filter")
    print("blue point is the real state, brown point is the particle state,reen point is the predict state, red point is the sensor data")
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, 0, 0], env, sensor=IMU)
    goforwardrobot.update(env, handles)
    pf = PF(goforwardrobot,
            M = 400, env = env, handles = handles,
            boundary = array([[-12, 12], [-12, 12], [-math.pi, math.pi]]))
    for ii in range(goforwardDynamicinputs.shape[0]):
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        pf.update(env, handles)
    print("Finish case1.2!")
    print("\n\n\n")

    time.sleep(3)
    ##############################        CASE2.1         ##############################
    ######## robot dynamic: goforwardrobot
    ######## sensor: landmark sensor
    ######## map: obstacles
    ######## estimation: EKF
    pf.clearParticles()
    pf.handles = []
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("#################                            CASE2.1                          #################")
    print("#################        Scenarios Extended Kalman Filter cannot solve        #################")
    print("Robot Dynamic: goforwardrobot(the robot could only go along its face direction)")
    print("Sensor: One Landmark sensor(the landmark which has red light)")
    print("Map: obstacles map")
    print("Estimation Algorithm: Extended Kalman Filter")
    print("This is the scene EKF could not work properly, because of highly nonlinearity of only one landmark")
    print("blue point is the real state, green point is the predict state")
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, 0, 0], env, sensor=LandMark)
    goforwardrobot.update(env, handles)
    ekf = EKF(goforwardrobot)

    for ii in range(goforwardDynamicinputs.shape[0]):
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        ekf.update(env, handles)
        time.sleep(0.05)
    print("Finish case2.1!")
    print("\n\n\n")
    time.sleep(3)

    ##############################        CASE2.2         ##############################
    ######## robot dynamic: goforwardrobot
    ######## sensor: landmark sensor
    ######## map: obstacles
    ######## estimation: EKF
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("#################                      CASE2.2                      #################")
    print("#################        Scenarios Particle Filter can solve        #################")
    print("Robot Dynamic: goforwardrobot(the robot could only go along its face direction)")
    print("Sensor: One Landmark sensor(the landmark which has red light)")
    print("Map: obstacles map")
    print("Estimation Algorithm: Particle Filter")
    print("This is the scene EKF could not work properly but Particle Filter could")
    print("blue point is the real state, green point is the predict state, brown point is the particle state")
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, 0, 0], env, sensor=LandMark)
    goforwardrobot.update(env, handles)
    pf = PF(goforwardrobot,
            M = 800, env = env, handles = handles,
            boundary = array([[-12, 12], [-12, 12], [-math.pi, math.pi]]))
    for ii in range(goforwardDynamicinputs.shape[0]):
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        pf.update(env, handles)
        # time.sleep(0.05)
    print("Finish case2.2!")
    print("\n\n\n")
    time.sleep(4)

    ##############################        CASE3.1         ##############################
    ######## robot dynamic: SimpleDynamicRobot
    ######## sensor: self-defined sensor
    ######## map: room
    ######## estimation: KF
    pf.clearParticles()
    pf.handles = []
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/roomScene.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("#################                        CASE3.1                  #################")
    print("#################    robot localization and navigation in a room  #################")
    print("Robot Dynamic: SimpleDynamicRobot")
    print("Sensor: self-defined sensor(GPS-like sensor, but could also sense robot's facing direction)")
    print("Map: room map")
    print("Estimation Algorithm: Kalman Filter")
    print("This is a demo for the implementaion of Kalman Filter")
    print("First, we use greedy search algorithm to find the path and generate the input")
    print("Then, we use PID control to let the prediction state follow the path")
    print("blue point is the real state, green point is the predict state, red point is the sensor data, black point is target path")
    goalconfig = [24, 5, pi / 2]
    simplerobot = robotType.SimpleDynamicRobot(robot, [1, 0, 0], env, sensor=GPS, search_alg=Astar)
    print("searching the path......")
    searchpath = simplerobot.search_path(handles, goalconfig)
    print("Find the path!")
    targetinput = simplerobot.path2input(searchpath)

    simplerobot.update(env, handles)
    kf = KF(simplerobot)
    pid = PID(searchpath, kp=np.array([5e-1, 5e-1, 0]), kd=np.array([5e-2, 5e-2, 0]), ki=np.array([5e-2, 5e-2, 0]))
    for ii in range(targetinput.shape[0]):
        simplerobot.input = targetinput[ii, :] + pid.FeedBack(kf.mu)
        simplerobot.predict(env)
        simplerobot.update(env, handles)
        kf.update(env, handles)
        time.sleep(0.05)
    print("Finish case3.1!")
    print("\n\n\n")
    time.sleep(4)
    ##############################        CASE3.2         ##############################
    ######## robot dynamic: SimpleDynamicRobot
    ######## sensor: self-defined sensor
    ######## map: room
    ######## estimation: KF
    pf.clearParticles()
    pf.handles = []
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/roomScene.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    print("#################                                  CASE3.2                           #################")
    print("#################    robot localization and navigation in a room(different target)   #################")
    print("Robot Dynamic: SimpleDynamicRobot")
    print("Sensor: self-defined sensor(GPS-like sensor, but could also sense robot's facing direction)")
    print("Map: room map")
    print("Estimation Algorithm: Kalman Filter")
    print("This is a demo for the implementaion of Kalman Filter")
    print("First, we use greedy search algorithm to find the path and generate the input")
    print("Then, we use PID control to let the prediction state follow the path")
    print("blue point is the real state, green point is the predict state, red point is the sensor data, black point is target path")
    goalconfig = [24, -8, pi / 2]
    simplerobot = robotType.SimpleDynamicRobot(robot, [1, 0, 0], env, sensor=GPS, search_alg=Astar)
    print("searching the path......")
    searchpath = simplerobot.search_path(handles, goalconfig)
    print("Find the path!")
    targetinput = simplerobot.path2input(searchpath)

    simplerobot.update(env, handles)
    kf = KF(simplerobot)
    pid = PID(searchpath, kp=np.array([5e-1, 5e-1, 0]), kd=np.array([5e-2, 5e-2, 0]), ki=np.array([5e-2, 5e-2, 0]))
    for ii in range(targetinput.shape[0]):
        simplerobot.input = targetinput[ii, :] + pid.FeedBack(kf.mu)
        simplerobot.predict(env)
        simplerobot.update(env, handles)
        kf.update(env, handles)
        time.sleep(0.05)
    print("Finish case3.2!")
    print("\n\n\n")

    raw_input("Press enter to exit...")
    handles = None
    env.Destroy()