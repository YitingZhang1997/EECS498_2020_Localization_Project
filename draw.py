import time
import os
from init import initRobot
from init import simpleDynamicinputs, goforwardDynamicinputs
import robotType
from KalmanFilter import KF, EKF
from ParticleFilter import PF
from Sensor import *
import math
import matplotlib.pyplot as plt

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
###################################### case 1 ################################################
    # handles = []
    # env.Reset()
    #
    # env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    # env.Load(env_path)
    # robot = env.GetRobots()[0]
    # initRobot(env, robot)
    # goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=IMU)
    # goforwardrobot.update(env, handles)
    # ekf = EKF(goforwardrobot)
    # error = linalg.norm(goforwardrobot.state[0:2] - ekf.mu[0:2])
    # t = arange(0, goforwardDynamicinputs.shape[0]+1, 1)
    # error_EKF = array([error])
    # update_t_EKF = array([0])
    # for ii in range(goforwardDynamicinputs.shape[0]):
    #     current_t = time.time()
    #     goforwardrobot.input = goforwardDynamicinputs[ii, :]
    #     goforwardrobot.predict(env)
    #     goforwardrobot.update(env, handles)
    #     ekf.update(env, handles)
    #     dt = time.time() - current_t
    #     error = linalg.norm(goforwardrobot.state[0:2] - ekf.mu[0:2])
    #     error_EKF = concatenate((error_EKF, array([error])))
    #     update_t_EKF = concatenate((update_t_EKF, array([dt])))
    #     time.sleep(0.025)
    #
    # handles = []
    # env.Reset()
    #
    # env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    # env.Load(env_path)
    # robot = env.GetRobots()[0]
    # initRobot(env, robot)
    # goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=IMU)
    # goforwardrobot.update(env, handles)
    # pf = PF(goforwardrobot,
    #         M = 400, env = env, handles = handles,
    #         boundary = array([[-12, 12], [-12, 12], [-math.pi, math.pi]]))
    # error = linalg.norm(goforwardrobot.state[0:2] - pf.mean[0:2])
    # error_PF = array([error])
    # update_t_PF = array([0])
    # for ii in range(goforwardDynamicinputs.shape[0]):
    #     current_t = time.time()
    #     goforwardrobot.input = goforwardDynamicinputs[ii, :]
    #     goforwardrobot.predict(env)
    #     goforwardrobot.update(env, handles)
    #     pf.update(env, handles)
    #     dt = time.time() - current_t
    #     error = linalg.norm(goforwardrobot.state[0:2] -  pf.mean[0:2])
    #     error_PF = concatenate((error_PF, array([error])))
    #     update_t_PF = concatenate((update_t_PF, array([dt])))
    #
    # fig, ax = plt.subplots()
    #
    # l1 = ax.scatter(t, update_t_PF, s=1)
    # l2 = ax.scatter(t, update_t_EKF, s=1)
    # ax.set_ylim([0, 0.2])
    # ax.set_xlabel('Iters')
    # ax.set_ylabel('Calculation time per iter(s)')
    # ax.set_title('Comparison in computation time between PF and EKF for case1')
    # ax.legend((l1, l2), ('PF', 'EKF'), loc='upper right')
    #
    # fig1, ax1 = plt.subplots()
    # l3 = ax1.scatter(t, error_PF, s=1)
    # l4 = ax1.scatter(t, error_EKF, s=1)
    # ax1.set_ylim([0, 5])
    # ax1.set_xlabel('Iters')
    # ax1.set_ylabel('Error')
    # ax1.set_title('Comparison in accuracy between PF and EKF for case1')
    # ax1.legend((l3, l4), ('PF', 'EKF'), loc='upper right')
    #
    # plt.show()

###################################### case 2 ################################################
    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=LandMark)
    goforwardrobot.update(env, handles)
    ekf = EKF(goforwardrobot)
    error = linalg.norm(goforwardrobot.state[0:2] - ekf.mu[0:2])
    t = arange(0, goforwardDynamicinputs.shape[0]+1, 1)
    error_EKF = array([error])
    update_t_EKF = array([0])
    for ii in range(goforwardDynamicinputs.shape[0]):
        current_t = time.time()
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        ekf.update(env, handles)
        dt = time.time() - current_t
        error = linalg.norm(goforwardrobot.state[0:2] - ekf.mu[0:2])
        error_EKF = concatenate((error_EKF, array([error])))
        update_t_EKF = concatenate((update_t_EKF, array([dt])))
        time.sleep(0.025)

    handles = []
    env.Reset()

    env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    env.Load(env_path)
    robot = env.GetRobots()[0]
    initRobot(env, robot)
    goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=LandMark)
    goforwardrobot.update(env, handles)
    pf = PF(goforwardrobot,
            M = 400, env = env, handles = handles,
            boundary = array([[-12, 12], [-12, 12], [-math.pi, math.pi]]))
    error = linalg.norm(goforwardrobot.state[0:2] - pf.mean[0:2])
    error_PF = array([error])
    update_t_PF = array([0])
    for ii in range(goforwardDynamicinputs.shape[0]):
        current_t = time.time()
        goforwardrobot.input = goforwardDynamicinputs[ii, :]
        goforwardrobot.predict(env)
        goforwardrobot.update(env, handles)
        pf.update(env, handles)
        dt = time.time() - current_t
        error = linalg.norm(goforwardrobot.state[0:2] -  pf.mean[0:2])
        error_PF = concatenate((error_PF, array([error])))
        update_t_PF = concatenate((update_t_PF, array([dt])))

    fig, ax = plt.subplots()

    l1 = ax.scatter(t, update_t_PF, s=1)
    l2 = ax.scatter(t, update_t_EKF, s=1)
    ax.set_ylim([0, 0.2])
    ax.set_xlabel('Iters')
    ax.set_ylabel('Calculation time per iter(s)')
    ax.set_title('Comparison in computation time between PF and EKF for case1')
    ax.legend((l1, l2), ('PF', 'EKF'), loc='upper right')

    fig1, ax1 = plt.subplots()
    l3 = ax1.scatter(t, error_PF, s=1)
    l4 = ax1.scatter(t, error_EKF, s=1)
    ax1.set_ylim([0, 10])
    ax1.set_xlabel('Iters')
    ax1.set_ylabel('Error')
    ax1.set_title('Comparison in accuracy between PF and EKF for case1')
    ax1.legend((l3, l4), ('PF', 'EKF'), loc='upper right')

    plt.show()
    tt = 0