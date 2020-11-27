import numpy as np
import robotType
class KF(object):
    def __init__(self, robotType):
        self.robotType = robotType
        self.R = self.robotType.R
        self.Q = self.robotType.sensor.Q
        self.A = self.robotType.A
        self.B = self.robotType.B
        self.C = self.robotType.sensor.C
        # self.mu = self.robotType.state
        self.mu = np.random.rand(3)
        self.Sigma = np.eye(self.mu.size)

    def update(self, env, handles, DRAWCOR = True):
        '''
        DRAWCOR: "True" draw sensor correction
        '''
        self.mu, self.Sigma = _KalmanFilter(self.mu, self.Sigma,
                                            self.robotType.observation, self.robotType.input,
                                            self.A, self.B, self.C, self.Q, self.R)
        if DRAWCOR:
            handles.append(env.plot3(points=np.array([self.mu[0], self.mu[1], 0.05]),
                                    pointsize=4.0,
                                    colors=np.array(((0,1,0)))))

def _KalmanFilter(mu, Sigma, z, u, A, B, C, Q, R):

    #prediction step
    mu_bar = np.dot(A, mu) + np.dot(B, u)
    Sigma_bar = np.dot(A, Sigma).dot(A.T) + R
    #correction step
    Kalman_gain = (C.dot(Sigma_bar)).dot(C.T) + Q
    K_t = (Sigma_bar.dot(C.T)).dot(np.linalg.inv(Kalman_gain ))
    mu = mu_bar + K_t.dot(z - C.dot(mu_bar))
    Sigma = (np.eye(K_t.shape[0]) - K_t.dot(C)).dot(Sigma_bar)
    mu_new = mu; Sigma_new = Sigma
    return mu_new, Sigma_new

class EKF(object):
    def __init__(self, robotType):
        self.robotType = robotType
        self.R = self.robotType.R
        self.Q = self.robotType.sensor.Q
        self.g = self.robotType.g
        self.G = self.robotType.G
        self.h = self.robotType.sensor.h
        self.H = self.robotType.sensor.H
        # self.mu = self.robotType.state
        self.mu = np.random.rand(3)
        self.Sigma = np.eye(self.mu.size)

    def update(self, env, handles, DRAWCOR = True):
        '''
        DRAWCOR: "True" draw sensor correction
        '''
        self.mu, self.Sigma = _ExtendedKalmanFilter(self.mu, self.Sigma,
                                            self.robotType.observation, self.robotType.input,
                                            self.g, self.G, self.h, self.H, self.Q, self.R)
        if DRAWCOR:
            handles.append(env.plot3(points=np.array([self.mu[0], self.mu[1], 0.05]),
                                    pointsize=4.0,
                                    colors=np.array(((0,1,0)))))

def _ExtendedKalmanFilter(mu, Sigma, z, u, g, G, h, H, Q, R):

    #prediction step
    mu_bar = g(u, mu)
    Sigma_bar = np.dot(G(u, mu), Sigma).dot(G(u, mu).T) + R
    #correction step
    Kalman_gain = (H(mu).dot(Sigma_bar)).dot(H(mu).T) + Q
    K_t = (Sigma_bar.dot(H(mu).T)).dot(np.linalg.inv(Kalman_gain))
    mu = mu_bar + K_t.dot(z - h(mu_bar))
    Sigma = (np.eye(K_t.shape[0]) - K_t.dot(H(mu))).dot(Sigma_bar)
    mu_new = mu; Sigma_new = Sigma
    return mu_new, Sigma_new
