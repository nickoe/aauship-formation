#!/usr/bin/env python

import roslib; roslib.load_manifest('aauship')

import rospy
from std_msgs.msg import String
from aauship.msg import *

import time
import os 

## This is the control node
#  Its sole purpose is to be the simple PID controller implementation
#  that should be used to make the tests for determining some
#  hydrodynamic derivatives, as described in the appendix Called
#  "Indentification of hydrodynamic coefficients".
#
#  This node gets input from the rqt_mypkg node, which sets reference
#  setpoints and PID coefficients to the controllers, from here it is
#  also possible to select the respective controller.
#
#  This is divided into two topics, one for the setpoints
#  (aauship/testSetpoints.msg) and one for the PID coefficients
#  (aauship/PID.msg).
class Control(object):
    ## Reference callback to update variables
    def ref_cb(self, data):
        print data
        pass

    ## PID callback to update variables
    def pid_cb(self, data):
        print data
        pass

    ## PID controller to calculate actuator input
    def pid(self, Kp, Ki, Kd, desired, currentstate):

        '''
        error = desired - currentstate
        integral = integral + error*dt
        if i~=1
        derivative(i) = (error(i) - prev_error)*dt
        end
        actuatorinput = Kp*error + Ki*integral + Kd*derivative
        prev_error = error
        return actuatorinput
        #sleep(dt)
        '''
        pass

    def run(self):
	    BUFSIZE = 1024
        self.ctllog = open(str(os.environ['ROS_TEST_RESULTS_DIR']) + "/../../src/aauship/scripts/logs/ctl.log",'w',BUFSIZE)
        print(self.ctllog.name)

        subref = rospy.Subscriber('ref_input', testSetpoints, self.pid_cb)
        subpid = rospy.Subscriber('pid_input', PID, self.ref_cb)
        pub = rospy.Publisher('lli_input', Faps)

        rospy.init_node('control')
        r = rospy.Rate(0.5) # Hz
        #rospy.spin() # Keeps the node running untill stopped
        while not rospy.is_shutdown():
            #pub.publish("control signals should be sent here")
            r.sleep()
        print("\nClosing log file")
        self.ctllog.close()
        print("Exiting")
        exit()

if __name__ == '__main__':
    w = Control()
    w.run()
