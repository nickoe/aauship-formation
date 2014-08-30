#!/usr/bin/env python2

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import Float64MultiArray, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from aauship.msg import *
import tf
import scipy.io as sio

# temporary way to simulate aauship
import kalmanfilterfoo as kfoo
import numpy as np
from math import pi, sqrt, atan2, acos, sin, fmod
import scipy.linalg as linalg

import time
import os 

## This is the simulaiton node, basically kind of the same as simaauship.m
class Simulator(object):
    def __init__(self):
        self.sub = rospy.Subscriber('lli_input', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('kf_states', Float64MultiArray, queue_size=1)
        self.trackpath = rospy.Publisher('track', Path, queue_size=3)
        self.refpath = rospy.Publisher('refpath', Path, queue_size=3)
        self.keepoutpath = rospy.Publisher('keepout', Path, queue_size=3)
        rospy.init_node('simulation_node')
        self.r = rospy.Rate(30) # Hz

        self.v = np.array([0.3,0.3,13.5969e-006,0.2,0.2,0.00033,0.00033])#Measurement,noise
        self.z = np.zeros(7)

        self.P_plus = np.zeros([17,17])
        self.R = np.diag(self.v)
        self.R_i = np.diag(self.v)

        self.thrustdiff = 0
        self.u = np.zeros(5) # input vector
        self.x = np.zeros(17) # state vector
        self.pubmsg = Float64MultiArray()
        self.refmsg = Path()
        self.refmsg.header.frame_id = "ned"
        self.keepoutmsg = Path()
        self.keepoutmsg.header.frame_id = "ned"
        self.klingen = sio.loadmat('klingenberg.mat')
        self.path = sio.loadmat('../../../../../matlab/2mmargintrack.mat')
        self.trackmsg = Path()
        self.trackmsg.header.frame_id = "ned"

        h = Header()
        q = Quaternion(0,0,0,1)

        # klingen['inner'][0]  is northing
        # klingen['inner'][1]  is easting
        for i in self.klingen['outer']:
            p = Point(i[0],i[1],0)
            self.refmsg.poses.append(PoseStamped(h, Pose(p, q)))

        for i in self.klingen['inner']:
            p = Point(i[0],i[1],0)
            self.keepoutmsg.poses.append(PoseStamped(h, Pose(p, q)))
        
    def callback(self, data):
        print(data.data[0])
        self.thrustdiff = data.data[0]      
        

    def run(self):
        # SAve a csv log with the control inputs
        #BUFSIZE = 1024
        #self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        ##self.ctllog = open("logs/ctl.log",'w',BUFSIZE)
        ##print(self.ctllog.name)

        # Construct the Kalman-filter
        f = kfoo.KF()

        # Wait a bit for the node to settle, such that Rviz does not miss the publishing
        time.sleep(3)
        self.refpath.publish(self.refmsg)
        self.keepoutpath.publish(self.keepoutmsg)

        # Initialize an poses array for the trackmsg
        h = Header()
        p = Point(0,0,0)
        q = Quaternion(0,0,0,1)
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))

        # GPS frequencey counter
        #gpsc = []
        #gpsc.append(0)
        jj = 0

        # Counter for figuring out when to add the GPS sample
        k = 0

        x_hat = self.x
        self.path['track'] = np.append([[self.x[0],self.x[1]]], self.path['track'], axis=0)
        # Main loop
        while not rospy.is_shutdown():
            # Headpoint of trail track
            #p = Point(self.x[0],self.x[1],0.0)
            p = Point(x_hat[0],x_hat[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[0] = PoseStamped(h, Pose(p, q))

            # Her skal u subscripe til lli input
            self.u = np.array([8,0,0,0,self.thrustdiff])

            # Simulation
            self.x = f.aaushipsimmodel(self.x,self.u)
            #self.pubmsg.data = self.x
            
            # Call AHRS node either Mahoney or Madgwick

            # Generate noise vector
            self.z[0:2] = self.x[0:2] + np.array([self.v[0],self.v[1]])*np.random.randn(1,2)
            self.z[2]   = self.x[6] + self.v[2]*np.random.randn(1,1)
            self.z[3:5] = self.x[7:9] + np.array([self.v[3],self.v[4]])*np.random.randn(1,2)
            self.z[5:7] = self.x[12:14] + np.array([self.v[5],self.v[6]])*np.random.randn(1,2)

            if k%20 != 0:
                self.R[0,0] = 10*10**10;
                self.R[1,1] = 10*10**10;
            else:
                self.R[0,0] = self.R_i[0,0]
                self.R[1,1] = self.R_i[1,1]
                #gpsc[jj] = k;
                jj = jj+1;
            
            (x_hat,self.P_plus) = f.KalmanF(x_hat, self.u, self.z, self.P_plus, self.R)
            
            self.pubmsg = Float64MultiArray()
            for a in x_hat:
                self.pubmsg.data.append(a)
                #print(a)
            self.pub.publish(self.pubmsg)

            # Send tf for the robot model visualisation
            br = tf.TransformBroadcaster()
            br.sendTransform((self.x[0],self.x[1], 0),
                             #tf.transformations.quaternion_from_euler(self.x[4], self.x[5], headingdesired),
                             tf.transformations.quaternion_from_euler(self.x[4], self.x[5], self.x[6]),
                             rospy.Time.now(),
                             "boat_link",
                             "ned")

            # Endpoint of trail track
            #p = Point(self.x[0],self.x[1],0.0)
            p = Point(x_hat[0],x_hat[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[1] = PoseStamped(h, Pose(p, q))
            self.trackpath.publish(self.trackmsg)
            #print(self.pathmsg)

            #rospy.signal_shutdown("testing")

            k = k+1
            print(time.time())

            self.r.sleep()
        print("\nClosing log file")
        ##self.ctllog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Simulator()
    w.run()
