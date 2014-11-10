#!/usr/bin/env python2

import roslib; roslib.load_manifest('aauship')

from aauship.msg import *
import rospy
from std_msgs.msg import Float64MultiArray, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from aauship.msg import *
import tf
import numpy as np
from math import pi, sqrt, atan2, acos, sin, fmod, cos
import scipy.linalg as linalg
import time
import os


# This file is supposed to replay measurement data from a real
# seatrail as it is presented to ROS. So this should act as a fake
# sensor-decode-node.py (publishes IMU through the /imu topic) and
# the lli-node.py (publishes GPS data thorugh the /gps1 topic).
# 
# You need to run process.m first to save the test data to a mat file
# that is to be read by this script for the playback.
class Playback(object):
    def __init__(self):
        rospy.init_node('playback_node')
        self.r = rospy.Rate(20) # Hz
        
        # Define the topics
        self.pub_imu = rospy.Publisher('imu', ADIS16405, queue_size=1)


    def run(self):
        # Main loop
        while not rospy.is_shutdown():

            # Call AHRS node either Mahony or Madgwick
            # Publish the calculated measurements for /imu
            #self.pub_imu.publish(self.imu_msg)


            self.r.sleep()
        
        print("Exiting  node")
        exit()

if __name__ == '__main__':
    w = Playback()
    w.run()

