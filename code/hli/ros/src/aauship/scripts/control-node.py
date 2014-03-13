#!/usr/bin/env python

# This is the contorl node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import time
import os 

class Control(object):
    def callback(self, data):
        # send data to lli_input topic
#        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
#        self.ctllog.write(data.data)
        # Publish data to lli_input
        print "control callback" + time.time()
        pass

    def run(self):

        self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        print(self.ctllog.name)
        pub = rospy.Publisher('lli_input', String)
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
