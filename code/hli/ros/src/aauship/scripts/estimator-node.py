#!/usr/bin/env python

# This is the esitimator and sensor node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import time
import os 

class Estimator(object):
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        for i, c in enumerate(data.data):
#            if c == '$' and ord(data.data[i+3]) == 9:
            if ord(data.data[i-3]) == '$' and ord(data.data[i]) == 9:
                print 'Data is', ord(data.data[i+2])
                print 'BUILD INFO\n', data.data[i+4:]
        self.imulog.write(data.data)

    def run(self):
        self.imulog = open(os.getcwd() + "/../meas/imu.log",'w')
        print(self.imulog.name)
        rospy.Subscriber('samples', String, self.callback)
        rospy.init_node('estimator')
        rospy.spin() # Keeps the node running untill stopped
        print("\nClosing log file")
        self.imulog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Estimator()
    w.run()
