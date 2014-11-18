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
        pass

    def run(self):
        #self.qu = Queue.Queue()
        #self.packet = fapsPacket.Handler('/dev/ttyUSB0', 115200, 0.02, self.qu)

        BUFSIZE = 1024
        #mb100log = open("logs/gps2.log",'wb',BUFSIZE)

        '''
        try:
            gps2 = gps2rcv.readline()
            if gps2 != "":
                gps2 = gps2.rstrip()
                gps2log.write(gps2 + ',' +str(time.time()) + "\r\n")
        except Exception:
            pass
        '''

        line = "$PASHR,POS,1,10,134035.35,5700.8745400,N,00959.1551112,E,059.318,10.4,077.3,000.459,+000.069,1.7,0.9,1.4,0.9,Hp23*30"
        line = line.split(',')
        posmod = float(line[2])
        sats = float(line[3])
        utctime = float(line[4])
        [latdec, londec] = (gpsfunctions.nmea2decimal(float(line[5]),line[6],float(line[7]),line[8]))
        lat = latdec*pi/180
        lon = londec*pi/180
        alt = float(line[9])
        ageofdiff = float(line[10])
        cog = float(line[11])*pi/180
        sog = float(line[12])*0.514444444
        verticalvel = float(line[13])
        pdop = float(line[14])
        hdop = float(line[15])
        vdop = float(line[16])
        tdop = float(line[17])
        crc = line[18]
        
        print(lat,lon)
        '''
        [latdec, londec] = (gpsfunctions.nmea2decimal(float(line[2]),line[3],float(line[4]),line[5]))
        latdec = latdec*pi/180
        londec = londec*pi/180
        if self.centerlat == 0 and self.centerlon == 0:
            self.rot=gpsfunctions.get_rot_matrix(float(latdec),float(londec))
            self.centerlat = float(latdec)
            self.centerlon = float(londec)
        pos = self.rot * (gpsfunctions.wgs842ecef(float(latdec),float(londec))-gpsfunctions.wgs842ecef(float(self.centerlat),float(self.centerlon)))



        line = "".join(packet['Data']).split(',')
        if line[0] == "$GPRMC" and line[2] == 'A':
        
            #print ",".join("".join(packet['Data']).split(',')[1:8])
            line = line[1:8]


        if line[1] == 'A' :
            self.gpsinvalid += 1
        
        if 42 <= self.gpsinvalid <= 67:
            line[1] = 'V'
            print "Gps invalid!"
        
        if line[1] == 'A' :
        
            self.gpslog.write(",".join(line) + ", " + str(time.time()) + "\r\n")
            self.fulllog.write(",".join(line) + ", " + str(time.time()) + "\r\n")
            #print line
            speed = float(line[6]) * 0.514444444 #* 0 + 100
            #print str(speed) + " m/s"
            [latdec, londec] = (gpsfunctions.nmea2decimal(float(line[2]),line[3],float(line[4]),line[5]))
            latdec = latdec*pi/180
            londec = londec*pi/180
            if self.centerlat == 0 and self.centerlon == 0:
                self.rot=gpsfunctions.get_rot_matrix(float(latdec),float(londec))
                self.centerlat = float(latdec)
                self.centerlon = float(londec)
            
            
            pos = self.rot * (gpsfunctions.wgs842ecef(float(latdec),float(londec))-gpsfunctions.wgs842ecef(float(self.centerlat),float(self.centerlon)))
            #print pos
            #print pos
            
            
            self.state[0] = [float(pos[0,0]),        1]
            #self.state[0] = [10,1]
            self.state[1] = [speed, 1]
            self.state[3] = [float(pos[1,0]),     1]
            #self.state[3] = [5,1]
        '''




        
        '''
        time.sleep(5)
        #self.packet.start()
        pub = rospy.Publisher('', Faps, queue_size=10)
        rospy.init_node('mb100')
        r = rospy.Rate(100) # Rate in Hz, maybe try a faster rate than
        # 100 to fix the buffering issue
        '''

        '''
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
        '''

        #mb100log.close()
        
if __name__ == '__main__':
    w = MB100()
    w.run()
