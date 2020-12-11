import time
import openravepy
from Queue import PriorityQueue
from math import *
from numpy import *

class Node:
    def __init__(self,x_in,y_in,theta_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = inf
        ## store the parent node
        self.parent = None

    def __call__(self, *args, **kwargs):
        print self.Configuration()

    def __hash__(self):
        c =self.Configuration()
        return int(c[0]*1000000 + c[1]*10000 + c[2]*100)

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid

    def Configuration(self):
        return [self.x, self.y, self.theta]

    def f(self, goalconfig):
        # return self.g + _Dis(self, Node(*goalconfig))
        return _Dis(self, Node(*goalconfig))

def _Dis(node1, node2):
    [nx, ny, ntheta] = node1.Configuration()
    [mx, my, mtheta] = node2.Configuration()
    c = sqrt( (nx - mx)**2 + (ny - my)**2 + min(abs(ntheta - mtheta), 2 * pi - abs(ntheta - mtheta))**2 )
    return c

def getNeighbour(N ,type = "4-connected"):
    DX = 0.1
    DY = 0.1
    DTHETA = pi/12
    [x, y, theta] = N.Configuration()
    neighbours = []
    if type == "4-connected":
        neighbours.append(Node(x - DX, y, theta))
        neighbours.append(Node(x + DX, y, theta))
        neighbours.append(Node(x, y - DY, theta))
        neighbours.append(Node(x, y + DY, theta))
        if theta - DTHETA < 0:
            thetaDown = theta - DTHETA + 2*pi
        else:
            thetaDown = theta - DTHETA

        if theta + DTHETA >= 2 * pi:
            thetaUp = theta + DTHETA - 2 * pi
        else:
            thetaUp = theta + DTHETA
        neighbours.append(Node(x, y, thetaUp))
        neighbours.append(Node(x, y, thetaDown))
    if type == "8-connected":
        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                for k in range(-1, 2, 1):
                    if i != 0 or j != 0 or k != 0:
                        if theta + k * DTHETA < 0:
                            thetaNear = theta + k*DTHETA + 2 * pi
                        elif theta + k * DTHETA >= 2 * pi:
                            thetaNear = theta + k * DTHETA - 2 * pi
                        else:
                            thetaNear = theta + k * DTHETA
                        neighbours.append(Node(x + i * DX, y + j * DY, thetaNear))
    return neighbours

def CheckGoal(node,goalconfig):
    [x, y, theta] = node.Configuration()
    return abs(x - goalconfig[0]) < 0.0001 and\
           abs(y - goalconfig[1]) < 0.0001 and\
           abs(min(abs(theta - goalconfig[2]), 2 * pi - abs(theta - goalconfig[2]))) < 0.0001


def isVisited(Visited_Node, node):
    return Visited_Node.has_key(node.__hash__())


def testCollision(robot, env, node):
    [x, y, theta] = node.Configuration()
    ca = cos(theta)
    sa = sin(theta)
    T = array([[ca, -sa, 0, x],
               [sa, ca, 0, y],
               [0, 0, 1, 0.05],
               [0, 0, 0, 1]])
    robot.SetTransform(T)
    return env.CheckCollision(robot)

def getPath(node):
    current_Node = node
    path = array([current_Node.Configuration()])
    while current_Node.parent != None:
        current_Node = current_Node.parent
        path = concatenate((path, array([current_Node.Configuration()])), axis=0)
    return path

def DrawPath(handles,env,node):
    current_Node = node
    while current_Node.parent != None:
        [x1, y1, theta1] = current_Node.Configuration()
        [x2, y2, theta2] = current_Node.parent.Configuration()
        line = array([[x1, y1, 0],
                      [x2, y2, 0]])
        handles.append(env.drawlinestrip(points=line,
                                         linewidth=10.0,
                                         colors=array(((0, 0, 0)))))
        current_Node = current_Node.parent


def DrawVisitedNode(robot, handles, env, node):
    [x1, y1, theta1] = node.Configuration()
    if testCollision(robot, env, node):
        handles.append(env.plot3(points=[x1, y1, 0.05],
                                 pointsize=6.0,
                                 colors=array((1, 0, 0))))
    else:
        handles.append(env.plot3(points=[x1, y1, 0.05],
                                 pointsize=6.0,
                                 colors=array((0, 0, 1))))

def Astar(robot,goalconfig,env,handles, type = "4-connected"):
    ###draw goal node
    handles.append(env.plot3(points=[goalconfig[0], goalconfig[1], 0.2],
                             pointsize=10.0,
                             colors=array((0, 1, 0))))
    q = PriorityQueue()
    T_init = robot.GetTransform()
    startconfig = [T_init[0,3], T_init[1,3], atan2(T_init[1,0], T_init[0,0])]
    startNode = Node(*startconfig)
    startNode.g = 0
    q.put((startNode.f(goalconfig), startNode))
    ## Stored the node that has been visited
    Visited_Node = {startNode.__hash__(): startNode}
    ##A* Search
    current_Node = startNode
    start_index = len(handles)
    handles.append(1)
    with env:
        while not CheckGoal(current_Node,goalconfig):
            if q.empty():
                print "No solution Found"
                break
            else:
                current_Node = q.get()[1]
                if not testCollision(robot,env,current_Node):
                    neighbour = getNeighbour(current_Node, type)
                    for near_Node in neighbour:
                        if not isVisited(Visited_Node, near_Node):
                            if near_Node.g > current_Node.g + _Dis(current_Node, near_Node):
                                near_Node.g = current_Node.g + _Dis(current_Node, near_Node)
                                near_Node.parent = current_Node
                            q.put((near_Node.f(goalconfig), near_Node))
                            Visited_Node[near_Node.__hash__()] = near_Node
                            handles[-1] = []
                            DrawVisitedNode(robot, handles, env, near_Node)
                        else:
                            near_Node = Visited_Node[near_Node.__hash__()]
                            if near_Node.g > current_Node.g + _Dis(current_Node, near_Node):
                                near_Node.g = current_Node.g + _Dis(current_Node, near_Node)
                                near_Node.parent = current_Node
    handles[start_index: ] = []
    DrawPath(handles, env, current_Node)
    path = getPath(current_Node)
    robot.SetTransform(T_init)
    return path
