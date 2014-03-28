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
        self.packet.lli_send(self.packet.package(data.Data,data.DevID,data.MsgID))
        
        #rospy.loginfo(data.data)
        pass

    def run(self):
        self.qu = Queue.Queue()
        self.packet = fapsPacket.Handler('/dev/lli', 57600, 0.02, self.qu)
        # GPS2 and Echo sounder should be opened here, or maybe implemented in the fapsPacket.Handler thread
        gps2rcv = serial.Serial("/dev/gps2",115200,timeout=0.04)
        echorcv = serial.Serial("/dev/echosounder",4800,timeout=0.04)


        echolog = open("logs/echolog.log",'w')
        gps2log = open("logs/gps2log.log",'wb')
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
                            str(data['Data']),
                            rospy.get_time())
            except Queue.Empty:
                pass


            # Grabbing the GPS2 data (tempory implementation)
            try:
                gps2 = gps2rcv.readline()
                if gps2 != "":
                    gps2 = gps2.rstrip()
                    gps2log.write(gps2 + ',' +str(time.time()) + "\r\n")
            except Exception:
                pass

            # Grabbing the echosounder data (tempory implementation)
            try:
                echosounder = echorcv.readline()
                echosounder = echosounder.rstrip()
                if echosounder != "":
                    echolog.write(echosounder + ',' + str(time.time()) + "\r\n")
            except Exception:
                pass


            # End of loop, wait to keep the rate
            r.sleep()

        echolog.close()
        gps2log.close()
        self.packet.close()
        self.packet.join()
        
if __name__ == '__main__':
    w = LLI()
    w.run()
