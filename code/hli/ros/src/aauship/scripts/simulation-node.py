#!/usr/bin/env python2

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import Float64MultiArray, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from aauship.msg import *
import tf
import scipy.io as sio

# temporary way to simulate aauship
import kalmanfilterfoo as kfoo
import numpy as np
from math import pi, sqrt, atan2, acos, sin
import scipy.linalg as linalg

import time
import os 

## This is the simulaiton node, basically kind of the same as simaauship.m
class Simulator(object):
    def __init__(self):
        #self.sub = rospy.Subscriber('lli_input', Faps, self.callback)
        self.pub = rospy.Publisher('kf_states', Float64MultiArray, queue_size=3)
        self.pubpath = rospy.Publisher('path', Path, queue_size=3)
        self.trackpath = rospy.Publisher('track', Path, queue_size=3)
        rospy.init_node('simulation_node')
        self.r = rospy.Rate(30) # Hz


        self.u = np.zeros(5) # input vector
        self.x = np.zeros(17) # state vector
        self.pubmsg = Float64MultiArray()
        self.pathmsg = Path()
        self.pathmsg.header.frame_id = "map"
        self.path = sio.loadmat('../../../../../matlab/track.mat')

        self.trackmsg = Path()
        self.trackmsg.header.frame_id = "map"

        h = Header()
        q = Quaternion(0,0,0,1)
        for i in self.path['allwps']:
            p = Point(i[1],i[0],0)
            self.pathmsg.poses.append(PoseStamped(h, Pose(p, q)))

        
    def callback(self, data):
        # Set the input according to the control signal.
        print "lli input callback" + str(time.time())
        print os.getcwd()
        print os.environ
        self.u = np.array([8,0,0,0,0])
        pass

    # Angle in rad to the interval (-pi pi]
    def rad2pipi(self, rad):
        r = (rad+np.sign(rad)*pi) % (2*pi) # remainder
        s = np.sign(np.sign(rad) + 2*(np.sign(abs( ((rad+pi) % (2*pi)) /(2*pi)))-1));
        pipi = r - s*pi;
        return pipi

    # This calculates reference points for the path follower
    def wp_gen(self, wps, wpe, now):
        
        #P_c = [now, 0]; # [x y angle]
        P_c = now; # [x y]
        wp_r = 1; # Waypoint Radius
        wp_reached = 0; # Waypoint not reached
        v_i_len = 2; # length of intermediate vector
        
        ## Initial calculations
        # track = [wps;wpe];

        ## Track-frame projected point
        v_i = ( (wpe-wps)/linalg.norm(wpe-wps) )*v_i_len; # Intermediate vector
        #P_i = [P_c[0]+v_i[0], P_c[1]+v_i[1]]; # Intermediate projected parallel point
        P_i = P_c+v_i

        ## Calculate projected point onto the track
        A = P_i - wps;
        B = wpe - wps;
        r_p = ((A.dot(B))/((linalg.norm(B)**2)))*B; # Projection of vector on vector
        P_p = r_p + wps;

        ## The vessels predicted position
        v_d = P_p - P_c[0:2];
        v_ref = v_d/linalg.norm(v_d); # normaliserer
        v_ref = v_ref * v_i_len;
        P_ref = v_ref + now;
        # plot(P_ref(2),P_ref(1),'*')

        ## Calculate if waypoint is reached
        dist = sqrt((P_c[0]-wpe[0])**2+(P_c[1]-wpe[1])**2);
        if dist < wp_r:
            wp_reached = 1

        ## Calculate heading
        # heading = 2*pi-asin(( P_ref(1)-P_c[0] ) / sqrt( (P_ref(1)-P_c[0])  ^2 + (P_ref(2)-P_c[1])^2 ));
        # heading = mod(atan2(v_ref(2),v_ref(1))+pi,2*pi)-pi;
        heading = self.rad2pipi(atan2(v_ref[1],v_ref[0]));


        ## CrossTrackError % MOD = HYP*SIN(A) => crosstrack = (pos-wps) * sin(vinkel mellem (pos-wps) og (pos-wps))
        a = linalg.norm(now-wps);
        b = linalg.norm(wpe-wps);
        vinkel = (acos((now-wps).dot((wpe-wps).T)/(a*b)));
        # vinkeligrader = vinkel*180/pi;
        cte = a*sin(vinkel);

        #return {'heading':heading, 'wp_reached':wp_reached, 'cte':cte}
        return (heading, wp_reached, cte)

    def run(self):
        BUFSIZE = 1024
        #self.ctllog = open(os.getcwd() + "/../meas/ctl.log",'w')
        ##self.ctllog = open("logs/ctl.log",'w',BUFSIZE)
        ##print(self.ctllog.name)
        self.u = np.array([4,0,0,0,0.1])
        f = kfoo.KF()
        time.sleep(1)
        self.pubpath.publish(self.pathmsg)

        error = []
        integral = []
        integral.append(0)
        derivative = []
        derivative.append(0)
        thrustdiff =[]
        thrustdiff.append(0)
        Kp = 5.0;
        Ki = 0.0;
        Kd = 50.0;
        
        print(self.wp_gen(np.array([0,0]),np.array([10,10]),np.array([10,1])))


        headingdesired = 2
        k = 0
        n = 0 # used for wp gen logic
        h = Header()
        p = Point(0,0,0)
        q = Quaternion(0,0,0,1)
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        while not rospy.is_shutdown():
            p = Point(self.x[0],self.x[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[0] = PoseStamped(h, Pose(p, q))

            # GNC
            (headingdesired, wp_reached, cte) = self.wp_gen(self.path['allwps'][n],self.path['allwps'][n+1],self.x[0:2]); # WP Gen
            if (wp_reached == 1):
                n = n+1;
                if n >= len(self.path['allwps']):
                    es = k;
                    print('Trying to break')
                    break


            self.u = np.array([4,0,0,0,thrustdiff[k]])

            self.x = f.aaushipsimmodel(self.x,self.u)
            self.pubmsg.data = self.x
            
            self.pathmsg.header.frame_id = "map"
            br = tf.TransformBroadcaster()

            br.sendTransform((self.x[0],self.x[1], 0),
                             tf.transformations.quaternion_from_euler(self.x[4], self.x[5], headingdesired),
                             rospy.Time.now(),
                             "boat_link",
                             "map")

            


            p = Point(self.x[0],self.x[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[1] = PoseStamped(h, Pose(p, q))
            self.trackpath.publish(self.trackmsg)
            #print(self.pathmsg)


            # PID
            error.append(self.rad2pipi(headingdesired  - self.x[6]))
            integral.append(integral[k] + error[k])
            if k!=1:
                derivative.append(error[k] - error[k-1])

            print("error " + str(error[k]))
            print("integral " + str(integral[k]))
            print("derivative " + str(derivative[k]))
            print("thrustdiff " + str(thrustdiff[k]))
            thrustdiff.append(Kp*error[k] + Ki*integral[k] + Kd*derivative[k])

            k = k+1
            print(time.time())
            self.r.sleep()
        print("\nClosing log file")
        ##self.ctllog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Simulator()
    w.run()
