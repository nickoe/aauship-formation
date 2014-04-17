#!/usr/bin/env python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *
from rospy.numpy_msg import numpy_msg

import time
import os 

import fapsParse
import numpy
from math import atan2

## This is the esitimator and sensor node
#
#  It takes the raw data published from the LLI node and interprets
#  them with the fapsParse.packetParser.parser(). 
class Estimator(object):
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+" I heard %s",data.Data)

        self.stat = self.stat + 1 # Used for callback debugging
        if self.stat > 1:     
            print "WARNING: This could be bad; Estimator gets a callback before it has finished the last one."
            print "self.stat = " + str(self.stat)

        #print "Running parser"
        tmp = {'DevID':str(data.DevID), 'MsgID':str(data.MsgID),'Data': (data.Data)}
        self.parser.parse(tmp)
        self.pub_imu.publish(numpy.asscalar(self.samples[0,0]),
                             numpy.asscalar(self.samples[1,0]),
                             numpy.asscalar(self.samples[2,0]),
                             numpy.asscalar(self.samples[3,0]),
                             numpy.asscalar(self.samples[4,0]),
                             numpy.asscalar(self.samples[5,0]),
                             numpy.asscalar(self.samples[6,0]),
                             numpy.asscalar(self.samples[7,0]),
                             numpy.asscalar(self.samples[8,0]),
                             numpy.asscalar(self.samples[9,0]),
                             numpy.asscalar(self.samples[10,0]),
                             numpy.asscalar(self.samples[11,0]))
        print atan2(numpy.asscalar(self.samples[8,0]),numpy.asscalar(self.samples[7,0]))*180/3.14158+180

        self.stat = 0 # Used for callback debugging

    def run(self):
        self.stat = 0 # Used for callback debugging
	BUFSIZE = 1024
        self.imulog   = open("logs/imu.log",'w',BUFSIZE)   # was acclog
        self.mixedlog = open("logs/mixed.log",'w',BUFSIZE) # was recieved
        self.gpslog   = open("logs/gps1.log",'w',BUFSIZE)  # was gpslog
        self.plog     = open("logs/p.log",'w',BUFSIZE)     # was plog
        #self.echolog = open("logs/echo.log",'w',BUFSIZE)  # Currently logged in lli-node.py
        #self.gps2log = open("logs/gps2.log",'wb',BUFSIZE) # Currently logged in lli-node.py

        self.samples = numpy.zeros((12,2))
        self.parser = fapsParse.packetParser(
                self.imulog,
                self.gpslog,
                self.samples,
                self.mixedlog,
                self.plog)

        print(self.imulog.name)

        rospy.init_node('estimator')
        rospy.Subscriber('samples', Faps, self.callback)
        self.pub_imu = rospy.Publisher('imu', ADIS16405)
        rospy.spin() # Keeps the node running untill stopped
        print("\nClosing log files")
        self.imulog.close()
        self.mixedlog.close()
        self.gpslog.close()
        self.plog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Estimator()
    w.run()
