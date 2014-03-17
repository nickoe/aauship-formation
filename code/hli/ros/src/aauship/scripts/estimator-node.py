#!/usr/bin/env python

# This is the esitimator and sensor node

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import time
import os 

import fapsParse

class Estimator(object):
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id()+" I heard %s",data.Data)
###        self.stat = self.stat + 1
###        print self.stat
        print "running parser"
        tmp = {'DevID':str(data.DevID), 'MsgID':str(data.MsgID),'Data': data.Data, 'Time': time.time()}
        self.parser.parse(tmp)

##        self.imulog.write(data.Data)

        self.stat = 0

    def run(self):
        self.stat = 0
        self.imulog = open(os.getcwd() + "/../meas/imu.log",'w')
        self.receivinglog = open("meas/received.txt",'w')
        self.acclog = open("meas/acc.txt",'w')
        self.gpslog = open("meas/gps.txt",'w')
        self.plog = open("meas/plog.txt",'w')
        #echolog = open("meas/echolog.txt",'w')
        #gps2log = open("meas/gps2log.txt",'wb')


        self.parser = fapsParse.packetParser(
                self.acclog,
                self.gpslog,
                self.receivinglog,
                self.plog)

        print(self.imulog.name)
        rospy.Subscriber('samples', Faps, self.callback)
        rospy.init_node('estimator')
        rospy.spin() # Keeps the node running untill stopped
        print("\nClosing log file")
        self.imulog.close()
        self.receivinglog.close()
        self.acclog.close()
        self.gpslog.close()
        self.plog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Estimator()
    w.run()
