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
import numpy as np

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
        self.packet = fapsPacket.packetHandler('/dev/lli', 57600, 0.02, self.qu)

#        self.parser = fapsParse.packetParser

        time.sleep(5)
        self.packet.start()
        pub = rospy.Publisher('samples', Faps)
#        pub = rospy.Publisher('samples', String)
        sub = rospy.Subscriber('lli_input', String, self.callback)
        rospy.init_node('lli')
        r = rospy.Rate(100) # Rate in Hz

        while not rospy.is_shutdown():
            try:
                data = self.qu.get(False)
                print "RAW:" + str((data['DevID']))
                try:
                    pub.publish(ord(data['DevID']),ord(data['MsgID']),''.join(data['Data']))
                except:
                    print "expected RAW" + str(type(data['DevID']))
                    print "publishing failed miserably or something"
                #pub.publish(np.uint8(data['DevID']),np.uint8(data['MsgID']),''.join(data['Data']))
#                self.parser.parse(data)
#                if ord(data['DevID']) == 0:
#                    print ''.join(data['Data'])
#                    pub.publish(''.join(data['Data']))
#                pub.publish(''.join(data))
            except Queue.Empty:
                pass
            r.sleep()

        self.packet.close()
        self.packet.join()
        
if __name__ == '__main__':
    w = LLI()
    w.run()
