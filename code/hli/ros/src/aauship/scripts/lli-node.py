#!/usr/bin/env python

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import serial
import struct
import time

def talker():
    print("Starting talker in lli-node.py")
    ser = serial.Serial('/dev/lli', 57600)
    pub = rospy.Publisher('samples', String)
    rospy.init_node('lli')
    r = rospy.Rate(1) # sleep time
    while not rospy.is_shutdown():
#        data = ser.readline()
        data = ser.read(100)
        pub.publish(data)
#        r.sleep()
#        rospy.loginfo(data)
        try:
            ser.write(struct.pack('>bbbbbb', 0x24,0x00,0x00,0x09,0x13,0x37))
        except Exception, e1:
            print "error communicating...: " + str(e1)
        print 'Writing' , time.time()
        r.sleep()
    ser.close()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
