#!/usr/bin/env python

# This is the esitimator and sensor node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import time
import os 

def callback(data):
#    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    for i, c in enumerate(data.data):
        if c == '$' and ord(data.data[i+3]) == 9:
            print 'Data is', ord(data.data[i+2])
            print 'BUILD INFO\n', data.data[i+4:]

def estimator():
    imulog = open(os.getcwd() + "/../meas/imu.log",'w')
    print(imulog.name)
    rospy.Subscriber('samples', String, callback)
    rospy.init_node('estimator')
    rospy.spin() # Keeps the node running untill stopped
    print("\nClosing log file")
    imulog.close()
    print("Exiting")
    exit()

if __name__ == '__main__':
    estimator()
