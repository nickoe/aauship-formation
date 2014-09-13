#!/usr/bin/env python2
# Kalman filter example demo in Python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf
from math import sin, cos

import numpy as np
import scipy.io as sio
import scipy.linalg as linalg
from math import pi, fmod
import gpsfunctions as geo

class KF(object):
    def __init__(self):
        # Load discretised model constants
        self.ssmat = sio.loadmat('../../../../../matlab/ssaauship.mat')
        
        # Measurement noise vector and covarince matrix
        #self.v = np.array([3.0, 3.0, 13.5969e-006, 0.2, 0.2, 0.00033, 0.00033])
        #self.R = np.diag(self.v)

        # Process noise vector and covariance matrix
        self.w = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01, 0.033, 0.033, 0.033, 0.033, 0.033])
        self.Q = np.diag(self.w)
        self.no_of_states = 17

        # Measurement vector
        self.z = np.zeros(7)

        # Static rotation matrix
        self.klingen = sio.loadmat('klingenberg.mat')
        self.Rn2e = self.RNED2ECEF(self.klingen['rotlon'], self.klingen['rotlat'])
        self.Re2n = self.Rn2e.T
        self.pos_of_ned_in_ecef = geo.wgs842ecef(self.klingen['rotlat'], self.klingen['rotlon'])

        # Topics
        self.subgps1 = rospy.Subscriber('gps1', GPS, self.gps1cb)
        self.subgps2 = rospy.Subscriber('gps2', GPS, self.gps2cb)
        self.subimu  = rospy.Subscriber('imu', ADIS16405, self.imucb)
        self.subahrs = rospy.Subscriber('attitude', Quaternion, self.ahrscb)
        self.pub = rospy.Publisher('kf_statesnew', Float64MultiArray, queue_size=3)

        rospy.init_node('klamanfilter_node')


    def aaushipsimmodel(self, x, u):
        # Linear simulation step
        xn = self.ssmat['Ad'].dot(x[2:12]) + self.ssmat['Bd'].dot(u)

        eta   = np.zeros(5)
        nu    = np.zeros(5)
        nudot = np.zeros(5)
        xs    = np.zeros(self.no_of_states)

        # Calculate positions with euler integration
        xn[4] = xn[9]*self.ssmat['ts'] + x[6]
        Rz    = np.matrix([[cos(xn[4]), -sin(xn[4])], [sin(xn[4]), cos(xn[4])]])
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


    # Seems like the KalmanF does not return the same as the matlab implementation at the moment!!!
    def KalmanF(self, x, u, z, P_plus, R):
        # System matrix
        PHI = np.zeros([17,17])
        PHI[0:2,0:2] = np.matrix([[1,0],[0,1]])
        PHI[2:12,2:12] = self.ssmat['Ad']
        PHI[12:17,12:17] = np.diag([1,1,1,1,1])
        PHI[12:17,7:12] = self.ssmat['Ad'][5:10,5:10]

        # Measurement matrix
        h = np.zeros([7,17])
        h[0:2,0:2] = np.diag([1,1])
        h[2:5,6:9] = np.diag([1,1,1])
        h[5:7,12:14] = np.diag([1,1])
        H = h;

        # The nonlinear contribution to the system
        PHI[0:2,7:9] = np.matrix([ [float(self.ssmat['ts'])*cos(x[6]),-float(self.ssmat['ts'])*sin(x[6])], 
                       [float(self.ssmat['ts'])*sin(x[6]), float(self.ssmat['ts'])*cos(x[6])] ])
        # PHI(1:2,8:9) = [ts*cos(x(7)) -ts*sin(x(7)); ts*sin(x(7)) ts*cos(x(7))]; 
        # print(PHI[0:2,7:9]) # seems to be ok now

        #Q = np.diag(np.ones([17]))
        Q = np.diag([0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.01,0.01,0.01,0.01,0.01,0.033,0.033,0.033,0.033,0.033])
        # Prediction
        x_hat_minus = self.aaushipsimmodel(x,u);
        P_minus = PHI*P_plus*PHI + Q;
        
        # Update
        z_bar = z - h.dot(x_hat_minus);
        #print(R)
        S = H.dot(P_minus.dot(H.T)) + R; # test the individual vars and operators in this line
        #print(S) # Something is wrong with S
        # S = H*P_minus*H' + R

        K = P_minus.dot(H.T).dot(linalg.inv(S));
        x_hat_plus = x_hat_minus + K.dot(z_bar);
        P_plus = (np.eye(17) - K.dot(H)).dot(P_minus).dot( (np.eye(17) - K.dot(H)).T ) + K.dot(R).dot(K.T);
       
        # Return estimated state vector
        return (x_hat_plus,P_plus)

    # Rotation matrix from NED to BODY frame
    # Rotation order is zyx
    def RNED2BODY(self, phi, theta, psi):
        cphi = cos(phi)
        sphi = sin(phi)
        cth  = cos(theta)
        sth  = sin(theta)
        cpsi = cos(psi)
        spsi = sin(psi)
         
        R = np.matrix([ [cpsi*cth, -spsi*cphi+cpsi*sth*sphi,  spsi*sphi+cpsi*cphi*sth],
                        [spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi],
                        [    -sth,                 cth*sphi,                 cth*cphi] ])
        return R

    # Rotation matrix from NED to ECEF frame
    # Using the eq. (2.84) from Fossen
    def RNED2ECEF(self, lon, lat):
        clon = cos(lon)
        slon = sin(lon)
        clat = cos(lat)
        slat = sin(lat)

        R = np.matrix([ [-clon*slat,  -slon,   -clon*clat],
                        [-slon*slat,   clon,   -slon*clat],
                        [      clat,      0,        -slat] ])

        return R

    # Angle in rad to the interval (-pi pi]
    def rad2pipi(self, rad):
        r = fmod((rad+np.sign(rad)*pi) , 2*pi) # remainder
        s = np.sign(np.sign(rad) + 2*(np.sign(abs( fmod((rad+pi), (2*pi)) /(2*pi)))-1));
        pipi = r - s*pi;
        return pipi


    # GPS1 callback
    def gps1cb(self, data):
        #z = [N E psi u v udot vdot]

        # Positoin in NED
        # This seems to only be approximate
        pos_ecef = geo.wgs842ecef(data.latitude, data.longitude, 0.0)
        pos_ned = self.Re2n.dot( pos_ecef - self.pos_of_ned_in_ecef )
        self.z[0:2] =  np.squeeze( pos_ned[0:2] )
        
        # Body velocities
        #print('')
        #print('track angle: ' + str(data.track_angle))
        #print('heading ang: ' + str(self.z[2]))
        beta = data.track_angle - self.z[2] # sideslip angle = track_angle - heading
        #print(beta)
        U_b = np.array([cos(beta), sin(beta)]) * data.SOG
        self.z[3:5] = np.squeeze( U_b )
        #print(self.z[3:5])
   
    # GPS2 callback
    def gps2cb(self, data):
        print('GPS2 is not implemented yet')

    # IMU callback
    # We just use the IMU callback to call the Kalman filter, becaue
    # it is easier, and it has a much higher sample rate than the
    # kalman filter. We should make sure we only use a GPS measurement
    # from a GPS only once. We can do that by checking the sequence
    # number of the GPS header.
    def imucb(self, data):
        #self.KalmanF()
        #print(data)

        # TODO calculate the measurement vector z and compare with the simulation node
        Rn2b = self.RNED2BODY(self.roll, self.pitch, self.yaw)
        a_imu = np.array([data.xaccl, data.yaccl, data.zaccl])
        a_b = a_imu - Rn2b.dot(np.array([0,0,-9.82]))
        #print('a_xb ' + str(a_b[0,0]))
        #print('a_yb ' + str(a_b[0,2]))
        #print('')
        self.z[5:7] = np.array([a_b[0,0], a_b[0,1]]) # TODO is the entirely correct? Maybe the sign is opposite?
        print(self.z[5:7])

        # TODO move the KF stuff from the simulation node in here, now it should still work
        pass

    def ahrscb(self, data):
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([data.x, data.y, data.z, data.w])
        #Rm2n  = self.RNED2BODY(pi/2, 0, pi )
        #att = Rm2n.dot(np.array([self.roll, self.pitch, self.yaw]))
        #print(att)
        #print( (self.roll, self.pitch, self.yaw))
        
        # Hax for attitude in euler angles
        self.roll = -self.roll
        #print(self.roll)
        self.pitch = -self.pitch
        #print(self.pitch)
        self.yaw = fmod( -self.yaw+2*pi+pi/2, 2*pi)
        #print(self.yaw)
        self.z[2] = self.yaw 

    def run(self):
        '''
        # Testing code for the KF
        x = np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]) # state vector
        u = np.array([8,0,0,0,0]) # input vector
        z = np.array([1,1,1,1,1,1,1]) # measurement vector

        self.P_plus = np.zeros([17,17])
        R = np.diag([3.0, 3.0, 13.5969, 0.1, 0.1, 0.0524, 0.0524])
        (xest,self.P_plus) = self.KalmanF(x, u, z, self.P_plus, R)
        print(self.P_plus)
        '''

        rospy.spin()

        print("Exiting simulaiton node")
        exit()



if __name__ == '__main__':
    w = KF()
    w.run()
