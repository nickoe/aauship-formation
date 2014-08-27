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

import time
import os 

## This is the simulaiton node, basically kind of the same as simaauship.m
class Simulator(object):
    def __init__(self):
        #self.sub = rospy.Subscriber('lli_input', Faps, self.callback)
        self.pub = rospy.Publisher('kf_states', Float64MultiArray, queue_size=3)
        self.pubpath = rospy.Publisher('path', Path, queue_size=3)
        self.trackpath = rospy.Publisher('track', Path, queue_size=3)
        rospy.init_node('simulation_node')
        self.r = rospy.Rate(10) # Hz


        self.u = np.zeros(5) # input vector
        self.x = np.zeros(17) # state vector
        self.pubmsg = Float64MultiArray()
        self.pathmsg = Path()
        self.pathmsg.header.frame_id = "map"
        self.path = sio.loadmat('../../../../../matlab/track.mat')

        self.trackmsg = Path()
        self.trackmsg.header.frame_id = "map"

        h = Header()
        q = Quaternion(0,0,0,1)
        for i in self.path['allwps']:
            p = Point(i[1],i[0],0)
            self.pathmsg.poses.append(PoseStamped(h, Pose(p, q)))

        
    def callback(self, data):
        # Set the input according to the control signal.
        print "lli input callback" + str(time.time())
        print os.getcwd()
        print os.environ
        self.u = np.array([8,0,0,0,0])
        pass

    def rad2pipi(self, rad):
        return rad

    def run(self):
        BUFSIZE = 1024
        #self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        ##self.ctllog = open("logs/ctl.log",'w',BUFSIZE)
        ##print(self.ctllog.name)
        self.u = np.array([4,0,0,0,0.1])
        f = kfoo.KF()
        time.sleep(1)
        self.pubpath.publish(self.pathmsg)

        error = []
        integral = []
        integral.append(0)
        derivative = []
        derivative.append(0)
        thrustdiff =[]
        thrustdiff.append(0)
        Kp = 5.0
        Ki = 0.051
        Kd = 50.0

        headingdesired = 2
        k = 0
        h = Header()
        p = Point(0,0,0)
        q = Quaternion(0,0,0,1)
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        while not rospy.is_shutdown():
            p = Point(self.x[0],self.x[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[0] = PoseStamped(h, Pose(p, q))


            self.u = np.array([4,0,0,0,thrustdiff[k]])

            self.x = f.aaushipsimmodel(self.x,self.u)
            self.pubmsg.data = self.x
            
            self.pathmsg.header.frame_id = "map"
            br = tf.TransformBroadcaster()

            br.sendTransform((self.x[0],self.x[1], 0),
                             tf.transformations.quaternion_from_euler(self.x[4], self.x[5], self.x[6]),
                             rospy.Time.now(),
                             "boat_link",
                             "map")

            


            p = Point(self.x[0],self.x[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[1] = PoseStamped(h, Pose(p, q))
            self.trackpath.publish(self.trackmsg)
            #print(self.pathmsg)


            # PID
            error.append(self.rad2pipi(headingdesired  - self.x[6]))
            integral.append(integral[k] + error[k])
            if k!=1:
                derivative.append(error[k] - error[k-1])

            thrustdiff.append(Kp*error[k] + Ki*integral[k] + Kd*derivative[k])

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
