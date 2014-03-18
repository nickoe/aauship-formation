#!/usr/bin/env python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import time
import os 

import fapsParse

## This is the esitimator and sensor node
class Estimator(object):
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+" I heard %s",data.Data)

        self.stat = self.stat + 1 # If this is 2, then it gets a callback before it has finished the last one
        print self.stat

        #print "Running parser"
        tmp = {'DevID':str(data.DevID), 'MsgID':str(data.MsgID),'Data': data.Data}
        self.parser.parse(tmp)

        self.stat = 0

    def run(self):
        self.stat = 0

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
