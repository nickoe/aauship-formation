#!/usr/bin/env python2

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import serial
import struct
import time

import Queue
import fapsPacket # fork of packetHandler

class MB100(object):
    def __init__(self):
	pass

    def run(self):
        self.qu = Queue.Queue()
        self.packet = fapsPacket.Handler('/dev/ttyUSB0', 115200, 0.02, self.qu)

        BUFSIZE = 1024
        mb100log = open("logs/gps2.log",'wb',BUFSIZE)


	#$PASHR,POS,1,10,134035.35,5700.8745400,N,00959.1551112,E,059.318,10.4,077.3,000.459,+000.069,1.7,0.9,1.4,0.9,Hp23*30
        time.sleep(5)
        self.packet.start()
        pub = rospy.Publisher('', Faps, queue_size=10)
        rospy.init_node('mb100')
        r = rospy.Rate(100) # Rate in Hz, maybe try a faster rate than
        # 100 to fix the buffering issue

        while not rospy.is_shutdown():
            # Grabbing the GPS2 data (tempory implementation)
            try:
                gps2 = gps2rcv.readline()
                if gps2 != "":
                    gps2 = gps2.rstrip()
                    gps2log.write(gps2 + ',' +str(time.time()) + "\r\n")
            except Exception:
                pass



            # End of loop, wait to keep the rate
            r.sleep()

        mb100log.close()
        
if __name__ == '__main__':
    w = MB100()
    w.run()
