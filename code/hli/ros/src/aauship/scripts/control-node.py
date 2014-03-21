#!/usr/bin/env python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import time
import os 

## This is the control node
class Control(object):
    def callback(self, data):
        # send data to lli_input topic
#        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
#        self.ctllog.write(data.data)
        # Publish data to lli_input
        print "control callback" + time.time()
        pass

    def run(self):
	BUFSIZE = 1024
        #self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        self.ctllog = open("logs/ctl.log",'w',BUFSIZE)
        print(self.ctllog.name)
        sub = rospy.Subscriber('control_input', Faps, self.callback)
        pub = rospy.Publisher('lli_input', Faps)
        rospy.init_node('control')
        r = rospy.Rate(0.5) # Hz
        #rospy.spin() # Keeps the node running untill stopped
        while not rospy.is_shutdown():
            pub.publish("control signals should be sent here")
            r.sleep()
        print("\nClosing log file")
        self.ctllog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Control()
    w.run()
