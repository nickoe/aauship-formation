#!/usr/bin/env python

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import serial
import struct
import time

import Queue
import fapsParse  # fork of packetparser
import fapsPacket # fork of packetHandler

class LLI(object):
    def callback(self, data):
        # write data to serial
        print "Requesting buildinfo " + str(time.time())
        jeppe = self.packet.package([],0,9)
        self.packet.lli_send(jeppe)
        
        rospy.loginfo(data.data)
        pass

    def run(self):
        self.qu = Queue.Queue()
        self.packet = fapsPacket.Handler('/dev/lli', 57600, 0.02, self.qu)
        # GPS2 and Echo sounder should be opened here, or maybe implemented in the fapsPacket.Handler thread
        time.sleep(5)
        self.packet.start()
        pub = rospy.Publisher('samples', Faps)
        sub = rospy.Subscriber('lli_input', Faps, self.callback)
        rospy.init_node('lli')
        r = rospy.Rate(100) # Rate in Hz

        while not rospy.is_shutdown():
            try:
                data = self.qu.get(False)
                pub.publish(data['DevID'],
                            data['MsgID'],
                            data['Data'],
                            rospy.get_time())
            except Queue.Empty:
                pass
            r.sleep()

        self.packet.close()
        self.packet.join()
        
if __name__ == '__main__':
    w = LLI()
    w.run()
