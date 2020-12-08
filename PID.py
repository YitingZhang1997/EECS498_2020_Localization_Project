import numpy as np

class PID(object):
    def __init__(self, desire, kp = np.array([0, 0, 0]), ki =np.array([0, 0, 0]), kd = np.array([0, 0, 0])):
        '''
        Input:
        desire: a series of desire output
        kp: propotional gain for PID, should be size 3X1 for x, y, theta
        ki: integration gain for PID, should be size 3X1 for x, y, theta
        kd: derivative gain for PID, should be size 3X1 for x, y, theta

        Object variable:
        self.accerror: used to stored accumulate error, used for integration part in PID
        self.preerror: used to stored previous error(one step before), used for derivative part in PID
        self.index: used to track the motion process, desire[self.index] is the current desire ouput
        '''
        self.desire = desire
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.accerror = np.array([0.0, 0.0, 0.])
        self.preerror = np.array([0., 0., 0.])
        self.index = 0

    def FeedBack(self, observe):
        '''
        Input:
        observe: the sensor observation for current step
        
        Output:
        feedback: the PID output feedback term        
        '''
        DROP = False
        if observe.size == 2:
            observe = np.concatenate((observe, np.array([0])), axis=0)
            DROP = True
        self.index += 1
        error = self.desire[self.index, :] - observe
        self.accerror += error
        feedback = self.kp * error + self.kd * ( error - self.preerror) + self.ki * self.accerror
        self.preerror = error
        if DROP:
            feedback[2] = 0
        return feedback