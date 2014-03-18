#!/usr/bin/env python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import time
import os 

import fapsParse

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
        tmp = {'DevID':str(data.DevID), 'MsgID':str(data.MsgID),'Data': data.Data}
        self.parser.parse(tmp)

        self.stat = 0 # Used for callback debugging

    def run(self):
        self.stat = 0 # Used for callback debugging

        self.imulog   = open("logs/imu.log",'w')   # was acclog
        self.mixedlog = open("logs/mixed.log",'w') # was recieved
        self.gpslog   = open("logs/gps.log",'w')   # was gpslog
        self.plog     = open("logs/p.log",'w')     # was plog
        #self.echolog = open("meas/echolog.txt",'w')
        #self.gps2log = open("meas/gps2log.txt",'wb')

        self.parser = fapsParse.packetParser(
                self.imulog,
                self.gpslog,
                self.mixedlog,
                self.plog)

        print(self.imulog.name)
        rospy.Subscriber('samples', Faps, self.callback)
        rospy.init_node('estimator')
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
