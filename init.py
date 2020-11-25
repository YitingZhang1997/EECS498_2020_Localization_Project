import time
import openravepy
import robotType

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def initRobot(env, robot):
    tuckarms(env,robot)
    T = array([[1, 0, 0, -5.5],
               [0, 1, 0, -7.5],
               [0, 0, 1, 0.05],
               [0, 0, 0, 1]])
    robot.SetTransform(T)
    

simpleDynamicinputs1 = repeat(array([[0.1, 0, 0]]), 60, axis = 0)
simpleDynamicinputs2 = repeat(array([[0, 0.1, 0]]), 100, axis = 0)
simpleDynamicinputs3 = repeat(array([[-0.1, 0, 0]]), 60, axis = 0)
simpleDynamicinputs4 = repeat(array([[0, -0.1, 0]]), 100, axis = 0)
simpleDynamicinputs = simpleDynamicinputs1
for _ in range(5):
    simpleDynamicinputs = numpy.append(simpleDynamicinputs, simpleDynamicinputs2, axis = 0)
    simpleDynamicinputs = numpy.append(simpleDynamicinputs, simpleDynamicinputs3, axis = 0)
    simpleDynamicinputs = numpy.append(simpleDynamicinputs, simpleDynamicinputs4, axis = 0)
    simpleDynamicinputs = numpy.append(simpleDynamicinputs, simpleDynamicinputs1, axis = 0)



goforwardDynamicinputs1 = repeat(array([[0.1, 0]]), 40, axis = 0)
goforwardDynamicinputs2 = repeat(array([[0.0628319, 0.0314159]]), 50, axis = 0)
goforwardDynamicinputs3 = repeat(array([[0.1, 0]]), 60, axis = 0)
goforwardDynamicinputs4 = repeat(array([[0.1, 0]]), 20, axis = 0)
goforwardDynamicinputs5 = repeat(array([[0.1, 0]]), 80, axis = 0)
goforwardDynamicinputs = numpy.append(goforwardDynamicinputs1, goforwardDynamicinputs2, axis = 0)
for _ in range(15):
    goforwardDynamicinputs = numpy.append(goforwardDynamicinputs, goforwardDynamicinputs3, axis = 0)
    goforwardDynamicinputs = numpy.append(goforwardDynamicinputs, goforwardDynamicinputs2, axis = 0)
    goforwardDynamicinputs = numpy.append(goforwardDynamicinputs, goforwardDynamicinputs4, axis = 0)
    goforwardDynamicinputs = numpy.append(goforwardDynamicinputs, goforwardDynamicinputs2, axis = 0)
goforwardDynamicinputs = numpy.append(goforwardDynamicinputs, goforwardDynamicinputs5, axis = 0)