#!/usr/bin/env python2

# This is the LLI node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import serial
import struct
import time
from math import pi

import Queue
import gpsfunctions

class MB100(object):
    def __init__(self):
        BUFSIZE = 1024
        self.mb100log = open("logs/mb100.log",'wb',BUFSIZE)
        self.mb100rcv = serial.Serial("/dev/ttyUSB0",115200,timeout=0.04)

    def parse(self,line):
        self.mb100log.write(line + ',' + str(time.time()) + "\r\n")
        line = line.split(',')
        if line[0] == "$PASHR" and line[1] == "POS":
            mb100 = {}
            mb100['posmode'] = int(line[2])
            mb100['sats'] = int(line[3])
            mb100['utctime'] = float(line[4])
            [latdec, londec] = (gpsfunctions.nmea2decimal(float(line[5]),line[6],float(line[7]),line[8]))
            mb100['lat'] = latdec*pi/180.0
            mb100['lon'] = londec*pi/180.0
            mb100['alt'] = float(line[9])
            mb100['ageofdiff'] = float(line[10])
            mb100['cog'] = float(line[11])*pi/180.0
            mb100['sog'] = float(line[12])*0.514444444
            mb100['verticalvel'] = float(line[13])
            mb100['pdop'] = float(line[14])
            mb100['hdop'] = float(line[15])
            mb100['vdop'] = float(line[16])
            mb100['tdop'] = float(line[17])
            mb100['crc'] = line[18]
            return mb100
        else:
            return None

    def run(self):
        # Example of data
        #line = "$PASHR,POS,1,10,134035.35,5700.8745400,N,00959.1551112,E,059.318,10.4,077.3,000.459,+000.069,1.7,0.9,1.4,0.9,Hp23*30"
        #print(self.parse(line))

        pub = rospy.Publisher('kf_states', Float64MultiArray, queue_size=1)
        rospy.init_node('mb100')
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # Grabbing the data
            try:
                data = selfmb100rcv.readline()
                if data != "":
                    data = data.rstrip()
                    self.parse(data)
            except Exception:
                pass

            # End of loop, wait to keep the rate
            r.sleep()
        
        print("Closing MB100 node")
        self.mb100log.close()
        self.mb100rcv.close()

        
if __name__ == '__main__':
    w = MB100()
    w.run()
