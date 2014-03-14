#!/usr/bin/env python

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String

import serial
import struct
import time

import Queue
import fapsParse  # fork of packetparser
import fapsPacket # fork of packetHandler

class LLI(object):
    def callback(self, data):
        # write data to serial
#        print 'Trying' , time.time()
#        try:
#            print 'Writing' , time.time()
#        #    nobw = ser.write(struct.pack('>bbbbbb', 0x24,0x00,0x00,0x09,0x13,0x37))
#        except Exception, e1:
#            print "error communicating...: " + str(e1)
#        print 'Waiting' , time.time() , "Wrote" , nobw, "bytes"
        print "Requesting buildinfo " + str(time.time())
        #nobw = self.ser.write(struct.pack('>bbbbbb', 0x24,0x00,0x00,0x09,0x13,0x37))
        #print "Wrote" + str(nobw) + "bytes"
        rospy.loginfo(data.data)
        pass

    def run(self):
        #self.ser = serial.Serial('/dev/lli', 57600, timeout = 0.02)
        self.qu = Queue.Queue()
        self.packet = fapsPacket.packetHandler('/dev/lli', 57600, 0.02, self.qu)

        #self.parser = fapsParse.packetParser


        time.sleep(5)
        self.packet.start()
        pub = rospy.Publisher('samples', String)
        sub = rospy.Subscriber('lli_input', String, self.callback)
        rospy.init_node('lli')
        r = rospy.Rate(100) # Hz

        while not rospy.is_shutdown():
            #data = self.ser.readline()
            #self.parser.parse(data)
          ##  self.packet

            try:
                print "fem"
                data = self.qu.get(False)
                print data
                #pub.publish(str(data))

            except Queue.Empty:
                print "qu empty" + str(time.time())
                pass
    #        rospy.loginfo(data)
            r.sleep()

        #self.ser.close()
        self.packet.close()
        self.packet.join()
        
if __name__ == '__main__':
    w = LLI()
    w.run()
