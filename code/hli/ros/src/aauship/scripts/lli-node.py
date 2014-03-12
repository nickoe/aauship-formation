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
    ser = serial.Serial('/dev/lli', 57600, timeout = 0.02)
    time.sleep(5)
    pub = rospy.Publisher('samples', String)
    rospy.init_node('lli')
    r = rospy.Rate(1) # sleep time
    while not rospy.is_shutdown():
#        data = ser.readline()
        data = ser.readline()
        pub.publish(data)
#        r.sleep()
#        rospy.loginfo(data)
        print 'Trying' , time.time()
        try:
            print 'Writing' , time.time()
            nobw = ser.write(struct.pack('>bbbbbb', 0x24,0x00,0x00,0x09,0x13,0x37))
        except Exception, e1:
            print "error communicating...: " + str(e1)
        print 'Waiting' , time.time() , "Wrote" , nobw, "bytes"
        r.sleep()
    ser.close()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
