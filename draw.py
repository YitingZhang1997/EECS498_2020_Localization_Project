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
    # raw_input("Press enter to exit...")
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
    # raw_input("Press enter to exit...")
###################################### case 2 ################################################
    # handles = []
    # env.Reset()
    #
    # env_path = os.path.join(path, "map/testSceneWithLandMark.env.xml")
    # env.Load(env_path)
    # robot = env.GetRobots()[0]
    # initRobot(env, robot)
    # goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=LandMark)
    # goforwardrobot.update(env, handles)
    # ekf = EKF(goforwardrobot)
    # error = linalg.norm(goforwardrobot.state[0:2] - ekf.mu[0:2])
    # t = arange(0, goforwardDynamicinputs.shape[0]+1, 1)
    # errorOB = linalg.norm(goforwardrobot.state[0:2] - goforwardrobot.observation[0:2])
    # error_observation = array([errorOB])
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
    #     errorOB = linalg.norm(goforwardrobot.state[0:2] - goforwardrobot.observation[0:2])
    #     error_observation = concatenate((error_observation, array([errorOB])))
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
    # goforwardrobot = robotType.GoForwardDynamicRobot(robot, [-6, -4, 0], env, sensor=LandMark)
    # goforwardrobot.update(env, handles)
    # pf = PF(goforwardrobot,
    #         M = 800, env = env, handles = handles,
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
    # ax1.set_ylim([0, 10])
    # ax1.set_xlabel('Iters')
    # ax1.set_ylabel('Error')
    # ax1.set_title('Comparison in accuracy between PF and EKF for case1')
    # ax1.legend((l3, l4), ('PF', 'EKF'), loc='upper right')
    #
    # plt.show()
    # tt = 0

    ############################### noise #############################
    # env_path = os.path.join(path, "map/testScene.env.xml")
    # env.Load(env_path)
    # robot = env.GetRobots()[0]
    # initRobot(env, robot)
    # handles = []
    # simplerobot = robotType.SimpleDynamicRobot(robot, [-6, 0, 0], env, sensor=GPS)
    # simplerobot.update(env, handles)
    # current_point = array([-6., 0., 0.])
    #
    # errorOB = linalg.norm(simplerobot.state[0:2] - simplerobot.observation[0:2])
    # error_observation = array([errorOB])
    # error_mean = array([errorOB])
    # t = arange(0, simpleDynamicinputs.shape[0] + 1, 1)
    #
    # for ii in range(simpleDynamicinputs.shape[0]):
    #     handles.append(env.plot3(points=array([current_point[0], current_point[1], 0.05]),
    #                              pointsize=4.0,
    #                              colors=array(((0, 0, 0)))))
    #     current_point += simpleDynamicinputs[ii, :]
    #     simplerobot.input = simpleDynamicinputs[ii, :]
    #     simplerobot.predict(env)
    #     simplerobot.update(env, handles)
    #
    #     errorOB = linalg.norm(simplerobot.state[0:2] - simplerobot.observation[0:2])
    #     error_observation = concatenate((error_observation, array([errorOB])))
    #     error_mean = concatenate((error_mean, array([mean(error_observation)])))
    #     time.sleep(0.05)
    # fig, ax = plt.subplots()
    # l1 = ax.scatter(t, error_observation, s=1)
    # l2, = ax.plot(t, error_mean, color = "r")
    # ax.set_xlabel('Iters')
    # ax.set_ylabel('Observation Error')
    # ax.legend((l1, l2), ('observation_error', 'mean_observation_error'), loc='upper right')
    # plt.show()
    # print("Finish case0.1!")
    # print("\n\n\n")
    raw_input("Press enter to exit...")