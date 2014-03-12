#!/usr/bin/env python

# This is the contorl node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import time
import os 

class Control(object):
    def callback(self, data):
#        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
#        self.imulog.write(data.data)
        pass

    def run(self):
        self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        print(self.ctllog.name)
#        rospy.Subscriber('samples', String, self.callback)
        rospy.init_node('control')
        rospy.spin() # Keeps the node running untill stopped
        print("\nClosing log file")
        self.ctllog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Control()
    w.run()
