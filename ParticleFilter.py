import robotType
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import time

class PF(object):
    def __init__(self, robotType, M, env, handles, boundary, DRAWPARTILE = True):
        '''
        Input:
        robotType: a type of robot, e.x SimpleDynamicRobot
        M: M inital particle for Particle Filter
        env: the env for robot
        handles: use to draw points in env 
        boundary:the boundary for PF to sample
                size = (3, 2)
                [[x_low_boundary, x_up_boundary],
                 [y_low_boundary, y_up_boundary],
                 [theta_low_boundary, theta_high_boundary]]
        DRAWPARTILE: "Ture", draw particles

        Object variable:
        X: used to stored particles
           size = (M, 4) 
           X[m, 0], X[m, 1] is the coordiante of robot, X[m ,2] is the oriantation of robot, X[m, 3] is the weight of particle
        handelDrawIndex: use to memory the location of particles drawing in handles
        '''
        self.robotType = robotType
        self.env = env
        self.handles = handles
        self.M = M
        self.boundary = boundary
        self.DRAWPARTILE = DRAWPARTILE
        self.X = self._genParticles(M = self.M, boundary = self.boundary)
        self.handelDrawIndex = 0
        self._drawParticles()
        self.mean = self.updatemean()

    def update(self, env, handles):
        self._applyU()
        self._updateWeight()
        self._reinit()
        self._resample()
        self.clearParticles()
        self.mean = self.updatemean()
        self.drawpredict()
        self._drawParticles()

    def _genParticles(self, M, boundary):
        with self.env:
            num = 0
            X = zeros((M, 4))
            while num < M:
                x = random.rand()*(boundary[0, 1] - boundary[0, 0]) + boundary[0, 0]
                y = random.rand()*(boundary[1, 1] - boundary[1, 0]) + boundary[1, 0]
                theta = random.rand()*(boundary[2, 1] - boundary[2, 0]) + boundary[2, 0]
                T = array([[cos(theta), -sin(theta), 0, x],
                        [sin(theta), cos(theta), 0, y],
                        [0, 0, 1, 0.05],
                        [0, 0, 0, 1]])
                self.robotType.robot.SetTransform(T)
                collision = self.env.CheckCollision(self.robotType.robot)
                if not collision:
                    X[num, :] = array([x, y, theta, 1])
                    num += 1
        return X
    
    def _drawParticles(self):
        if self.DRAWPARTILE:
            M = self.X.shape[0]

            self.handelDrawIndex = len(self.handles)
            for ii in range(M):
                self.handles.append(self.env.plot3(points=array([self.X[ii,0], self.X[ii, 1], 0.4]),
                                    pointsize=4.0,
                                    colors=array(((0.4, 0, 0)))))

    def clearParticles(self):
        ## clear particles drawing commands in handles
        self.handles[self.handelDrawIndex : self.handelDrawIndex+self.M] = []

    def _applyU(self):
        M = self.X.shape[0]
        for ii in range(M):
            ## add noise could add variety of the sample
            prediction_noise = random.multivariate_normal((0, 0, 0), self.robotType.R)
            new_particle_state = self.robotType.g(self.robotType.input,self.X[ii,0:3]) + prediction_noise
            self.X[ii, 0:3] = new_particle_state

    def _updateWeight(self):
         M = self.X.shape[0]
         true_observe = self.robotType.observation
         k = true_observe.shape[0]
         Sigma = self.robotType.sensor.Q
         T = self.robotType.robot.GetTransform()
         for ii in range(M):
             particle_observe = self.robotType.sensor.h(self.X[ii,0:3])
             delta = true_observe - particle_observe
             ## the weight should be the probability for particle_observe being oberved as true_observe, giveoise coevariance
             # self.X[ii, 3] = round(1/(sqrt(((2*pi)**k)*linalg.det(Sigma)))*exp(-0.5 * dot(delta.T, linalg.inv(Sigma)).dot(delta)), 8)
             self.X[ii, 3] = exp(-(linalg.norm(delta))**2)
            #  print(particle_observe)
            #  print(exp(-(linalg.norm(delta))))
            #  print()
         self.robotType.robot.SetTransform(T)

    def _resample(self):
        ## Low-cariance sampling
        total_weight = sum(self.X[:, 3])
        interval = total_weight/self.M
        r = random.random()*interval
        num = 0
        current_weight_index = 0
        accumulate_weight = 0
        new_X = zeros((self.M, 4))
        while num < self.M:
            if r >  accumulate_weight:
                accumulate_weight += self.X[current_weight_index, 3]
                current_weight_index += 1
            else:
                r += interval
                new_X[num, :] = self.X[current_weight_index - 1, :]
                new_X[num, 3] = 1
                num += 1
        self.X = new_X

    def _reinit(self):
        if max(self.X[:,3]) < 1e-3:
            self.X = self._genParticles(M=self.M, boundary=self.boundary)
    def updatemean(self):
        x = mean(self.X[:,0])
        y = mean(self.X[:,1])
        theta = mean(self.X[:,2])
        return array([x, y, theta])
    def drawpredict(self):
        self.handles.append(self.env.plot3(points=array([self.mean[0], self.mean[1], 0.4]),
                                           pointsize=4.0,
                                           colors=array(((0, 1, 0)))))




