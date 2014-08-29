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
from math import pi, sqrt, atan2, acos, sin, fmod
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
        self.refpath = rospy.Publisher('refpath', Path, queue_size=3)
        self.keepoutpath = rospy.Publisher('keepout', Path, queue_size=3)
        rospy.init_node('simulation_node')
        self.r = rospy.Rate(30) # Hz


        self.u = np.zeros(5) # input vector
        self.x = np.zeros(17) # state vector
        self.pubmsg = Float64MultiArray()
        self.pathmsg = Path()
        self.pathmsg.header.frame_id = "map"
        self.refmsg = Path()
        self.refmsg.header.frame_id = "map"
        self.keepoutmsg = Path()
        self.keepoutmsg.header.frame_id = "map"
        self.path = sio.loadmat('../../../../../matlab/2mmargintrack.mat')
        self.klingen = sio.loadmat('klingenberg.mat')

        self.trackmsg = Path()
        self.trackmsg.header.frame_id = "map"

        h = Header()
        q = Quaternion(0,0,0,1)
        for i in self.path['track']:
            p = Point(i[0],i[1],0)
            self.pathmsg.poses.append(PoseStamped(h, Pose(p, q)))
	
        # klingen['inner'][0]  is northing
        # klingen['inner'][1]  is easting
        for i in self.klingen['outer']:
            p = Point(i[0],i[1],0)
            self.refmsg.poses.append(PoseStamped(h, Pose(p, q)))

        for i in self.klingen['inner']:
            p = Point(i[0],i[1],0)
            self.keepoutmsg.poses.append(PoseStamped(h, Pose(p, q)))
        
    def callback(self, data):
        # Set the input according to the control signal.
        print "lli input callback" + str(time.time())
        print os.getcwd()
        print os.environ
        self.u = np.array([8,0,0,0,0])
        pass

    # Angle in rad to the interval (-pi pi]
    def rad2pipi(self, rad):
        r = fmod((rad+np.sign(rad)*pi) , 2*pi) # remainder
        #print(r)
        s = np.sign(np.sign(rad) + 2*(np.sign(abs( fmod((rad+pi), (2*pi)) /(2*pi)))-1));
        #print(s)
        pipi = r - s*pi;
        return pipi

        '''
        -2.6416 = rem( (-2*pi+0.5) + sign(-2*pi+0.5)*pi , 2*pi)
        r = rem(angle+sign(angle)*pi,2*pi);
        s = sign(sign(angle) + 2*(sign(abs(rem(angle+pi,2*pi)/(2*pi)))-1));
        y = r - s*pi;
        '''

    # This calculates reference points for the path follower
    def wp_gen(self, wps, wpe, now):
        
        #P_c = [now, 0]; # [x y angle]
        P_c = now; # [x y]
        wp_r = 1; # Waypoint Radius
        wp_reached = 0; # Waypoint not reached
        v_i_len = 2; # length of intermediate vector
        
        ## Initial calculations
        #track = [wps;wpe];

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
        heading = self.rad2pipi(atan2(v_ref[1],v_ref[0]))

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

        f = kfoo.KF()
        time.sleep(3)
        self.pubpath.publish(self.pathmsg)
        self.refpath.publish(self.refmsg)
        self.keepoutpath.publish(self.keepoutmsg)

        # Initilaze parapeters for the PID contorller
        error = []
        integral = []
        integral.append(0)
        derivative = []
        derivative.append(0)
        thrustdiff =[]
        thrustdiff.append(0)
        Kp = 5.0;
        Ki = 0.051;
        Kd = 50.0;
        
	    # Testing output of rad2pipi funciton
        '''
        a = np.array([-12.5664,-12.0664,-11.5664,-11.0664,-10.5664,-10.0664, -9.5664, -9.0664, -8.5664, -8.0664, -7.5664, -7.0664, -6.5664, -6.0664, -5.5664, -5.0664, -4.5664, -4.0664, -3.5664, -3.0664, -2.5664, -2.0664, -1.5664, -1.0664, -0.5664, -0.0664,  0.4336,  0.9336,  1.4336,  1.9336,  2.4336,  2.9336,  3.4336,  3.9336,  4.4336,  4.9336,  5.4336,  5.9336,  6.4336,  6.9336,  7.4336,  7.9336,  8.4336,  8.9336,  9.4336,  9.9336, 10.4336, 10.9336, 11.4336, 11.9336, 12.4336])
        for x in a:
            print(self.rad2pipi(x))
        '''
        #(self.rad2pipi(-2*pi+0.5))
        
	
	    # Testing output of the wp_gen funciton
	    # [heading is wrong, compares to the matlab funciton]
        #print(self.wp_gen(np.array([0,-3]),np.array([10,10]),np.array([-10,1])))


        '''
        # Testing output of the simulation model
        # [Seems to work just fine when compared to the matlab function]
        x = np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])
        u = np.array([8,0,0,0,0])
        print(f.aaushipsimmodel(x,u))
        '''
        #rospy.signal_shutdown("testing")


        k = 0
        n = 1 # used for wp gen logic

        # Initialize an poses array for the trackmsg
        h = Header()
        p = Point(0,0,0)
        q = Quaternion(0,0,0,1)
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))
        self.trackmsg.poses.append(PoseStamped(h, Pose(p, q)))


        print(self.path['track'])
        self.path['track'] = np.append([[self.x[0],self.x[1]]], self.path['track'], axis=0)
        print(self.path['track'])
        # Main loop
        while not rospy.is_shutdown():
            # Headpoint of trail track
            p = Point(self.x[0],self.x[1],0.0)
            q = Quaternion(0,0,0,1)
            self.trackmsg.poses[0] = PoseStamped(h, Pose(p, q))

            # GNC
            (headingdesired, wp_reached, cte) = self.wp_gen(self.path['track'][n-1],self.path['track'][n],np.array([self.x[0],self.x[1]])); # WP Gen
            if (wp_reached == 1):
                n = n+1;
                if n >= len(self.path['track']):
                    es = k;
                    print('Trying to break')
                    break

            self.u = np.array([8,0,0,0,thrustdiff[k]])

            # Simulation
            self.x = f.aaushipsimmodel(self.x,self.u)
            self.pubmsg.data = self.x
            
            # Send tf for the robot model visualisation
            br = tf.TransformBroadcaster()
            br.sendTransform((self.x[0],self.x[1], 0),
                             #tf.transformations.quaternion_from_euler(self.x[4], self.x[5], headingdesired),
                             tf.transformations.quaternion_from_euler(self.x[4], self.x[5], self.x[6]),
                             rospy.Time.now(),
                             "boat_link",
                             "map")

            # Endpoint of trail track
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
            thrustdiff.append(Kp*error[k] + Ki*integral[k] + Kd*derivative[k])
            
            print("error " + str(error[k]))
            print("integral " + str(integral[k]))
            print("derivative " + str(derivative[k]))
            print("thrustdiff " + str(thrustdiff[k]))

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
