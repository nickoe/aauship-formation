#!/usr/bin/env python2
# Kalman filter example demo in Python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *
from math import sin, cos

import numpy
import pylab
import scipy.io as sio
import scipy.linalg as linalg


class KF(object):
    def __init__(self):
        self.ssmat = sio.loadmat('/home/nickoe/aauship-formation/code/matlab/ssaauship.mat')
        self.no_of_states = 17

    def aaushipsimmodel(self, x, u):
        # Linear simulation step
        xn = self.ssmat['Ad'].dot(x[2:12]) + self.ssmat['Bd'].dot(u)

        eta   = numpy.zeros(5)
        nu    = numpy.zeros(5)
        nudot = numpy.zeros(5)
        xs    = numpy.zeros(self.no_of_states)

        # Calculate positions with euler integration
        xn[4] = xn[9]*self.ssmat['ts'] + x[6]
        Rz    = numpy.matrix([[cos(xn[4]), -sin(xn[4])], [sin(xn[4]), cos(xn[4])]])
        eta[0:2] = x[0:2] + Rz.dot(xn[5:7])*float(self.ssmat['ts'])

        # Compute Fossen vectors
        eta[2:5] = xn[7:10]*self.ssmat['ts'] + x[4:7]
        nu       = xn[5:10]
        nudot    = xn[5:10]-x[7:12]

        # Full state simulaiton vector
        xs[0:2] = eta[0:2]
        xs[2:4] = xn[0:2]
        xs[4:7] = eta[2:5]
        xs[7:12] = nu
        xs[12:17] = nudot

        return xs


    def KalmanF(self, x, u, z, P_plus, R):
        # System matrix
        PHI = numpy.zeros([17,17])
        PHI[0:2,0:2] = numpy.matrix([[1,0],[0,1]])
        PHI[2:12,2:12] = self.ssmat['Ad']
        PHI[12:17,12:17] = numpy.diag([1,1,1,1,1])
        PHI[12:17,7:12] = self.ssmat['Ad'][5:10,5:10]

        # Measurement matrix
        h = numpy.zeros([7,17])
        h[0:2,0:2] = numpy.diag([1,1])
        h[2:5,6:9] = numpy.diag([1,1,1])
        h[5:7,12:14] = numpy.diag([1,1])
        H = h;

        Q = numpy.diag(numpy.ones([17]))
        # Prediction
        x_hat_minus = self.aaushipsimmodel(x,u);
        P_minus = PHI*P_plus*PHI + Q;
        
        print(x_hat_minus.T)
        print(h)
        print(z.T)
        # Update
        z_bar = z - h.dot(x_hat_minus);
        S = H.dot(P_minus).dot(H.T) + R;
        K = P_minus.dot(H.T).dot(linalg.inv(S));
         

        '''
        % if mod(k,10)
        %     K(1:2,:,k) = zeros(2,7);
        %     K(:,1:2,k) = zeros(17,2);
        %     K(8:9,:,k) = zeros(2,7);
        %     K(:,4:5,k) = zeros(17,2);
        % else
        %     gpsc(jj) = k;
        %     jj=jj+1;
        % end
        '''

        
        x_hat_plus = x_hat_minus + K.dot(z_bar);
        P_plus = (numpy.eye(17) - K.dot(H)).dot(P_minus).dot( (numpy.eye(17) - K.dot(H)).T ) + K.dot(R).dot(K.T);
        
        return x_hat_plus
        

    def run(self):
        x = numpy.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]) # state vector
        u = numpy.array([8,0,0,0,0]) # input vector
        z = numpy.array([1,1,1,1,1,1,1]) # measurement vector
        #xs = self.aaushipsimmodel(x,u)
        #print(xs)

        P_plus = numpy.zeros([17,17])
        R = numpy.diag([3.0, 3.0, 13.5969, 0.1, 0.1, 0.0524, 0.0524])
        xest = self.KalmanF(x, u, z, P_plus, R)
        print(xest)

        # intial parameters
        n_iter = 50
        sz = (n_iter,) # size of array
        x = -0.37727 # truth value (typo in example at top of p. 13 calls this z)
        z = numpy.random.normal(x,0.1,size=sz) # observations (normal about x, sigma=0.1)

        Q = 1e-5 # process variance

        # allocate space for arrays
        xhat=numpy.zeros(sz)      # a posteri estimate of x
        P=numpy.zeros(sz)         # a posteri error estimate
        xhatminus=numpy.zeros(sz) # a priori estimate of x
        Pminus=numpy.zeros(sz)    # a priori error estimate
        K=numpy.zeros(sz)         # gain or blending factor

        R = 0.1**2 # estimate of measurement variance, change to see effect

        # intial guesses
        xhat[0] = 0.0
        P[0] = 1.0

        for k in range(1,n_iter):
            # time update
            xhatminus[k] = xhat[k-1]
            Pminus[k] = P[k-1]+Q

            # measurement update
            K[k] = Pminus[k]/( Pminus[k]+R )
            xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
            P[k] = (1-K[k])*Pminus[k]

        '''
        pylab.figure()
        pylab.plot(z,'k+',label='noisy measurements')
        pylab.plot(xhat,'b-',label='a posteri estimate')
        pylab.axhline(x,color='g',label='truth value')
        pylab.legend()
        pylab.xlabel('Iteration')
        pylab.ylabel('Voltage')

        pylab.figure()
        valid_iter = range(1,n_iter) # Pminus not valid at step 0
        pylab.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
        pylab.xlabel('Iteration')
        pylab.ylabel('$(Voltage)^2$')
        pylab.setp(pylab.gca(),'ylim',[0,.01])
        pylab.show()
        '''

if __name__ == '__main__':
    w = KF()
    w.run()
