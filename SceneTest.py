import time
import openravepy
import os
from init import initRobot
from init import simpleDynamicinputs, goforwardDynamicinputs
import robotType
from KalmanFilter import KF, EKF
from Sensor import *


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
    # load a scene from ProjectRoom environment XML file
    # env_path = os.path.join(path, "data/testScene.env.xml")

    # test a scene with barriers
    env_path = os.path.join(path, "data/testSceneWithBarrier.env.xml")
    env.Load(env_path)

    robot = env.GetRobots()[0]
    initRobot(env, robot)

    ######################## Load a simpleDynamicRobot with a GPS sensor ########################
    ########################          Use KalmanFilter to predict        ########################
    simplerobot = robotType.SimpleDynamicRobot(robot, [-6, -2, 0], sensor=GPS)
    simplerobot.update(env, handles)
    kf = KF(simplerobot)

    for ii in range(simpleDynamicinputs.shape[0]):
        simplerobot.input = simpleDynamicinputs[ii, :]
        simplerobot.predict(env)
        simplerobot.update(env, handles)
        kf.update(env, handles)
        time.sleep(0.05)

    ######################## Load a GoForwardDynamicRobot with a IMU sensor ########################
    ########################               Use EKF to predict               ########################
    # goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-3, -5, 0], sensor=IMU)
    # goforwardrobot.update(env, handles)
    # ekf = EKF(goforwardrobot)
    # for ii in range(goforwardDynamicinputs.shape[0]):
    #     goforwardrobot.input = goforwardDynamicinputs[ii, :]
    #     goforwardrobot.predict()
    #     goforwardrobot.update(env, handles)
    #     ekf.update(env, handles)
    #     time.sleep(0.05)


    raw_input("Press enter to exit...")
    handles = None
    env.Destroy()