#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos

from geometry_msgs.msg import Twist
import time
from std_msgs.msg import UInt16
from std_msgs.msg import Int8

from neato_driver.neato_driver import Botvac

class NeatoNode:

    def __init__(self):
    	rospy.init_node('teleop04')

    	self.port = rospy.get_param('~port1 ', "/dev/ttyACM1")
    	rospy.loginfo("Using port: %s"%(self.port))

    	self.robot = Botvac(self.port)
	
	rospy.loginfo("Object init")
    	rospy.Subscriber("twist04", Twist, self.twist_handler, queue_size=10)
    	rospy.Subscriber("cbon04", Int8, self.cbon04, queue_size=10)

        rospy.on_shutdown(self.stopFromSleeping)
	self.latest_twist = Twist()
	
        self.SPEED = 150
        self.DIST = 20
        self.lastx = 0
        self.lasty = 0
        self.xramp = 0
        self.yramp = 0
        self.status = False

    def spin(self):

        # main loop of driver
        r = rospy.Rate(20)
        DIRECTION = 1 # 1 for forward, -1 for backwards
        while not rospy.is_shutdown():
            z = self.latest_twist.angular.z
            x = self.latest_twist.linear.x
            if x == 0 and z == 0: #we were told not to move that all
                SPEED = 0
                dist_l = 0
                dist_r = 0
		print("running keepalive")
		#keepalive action
		self.robot.requestScan()
            else:
                if x != 0:
                    SPEED = abs(x)   
                    if x < 0:
                        DIRECTION = -1 # backwards
                    else:
                        DIRECTION = 1 # forwards
                else:
                    SPEED = 150

                print(self.latest_twist)

                if z != 0:
                    dist_l = -int(self.DIST*z)
                    dist_r = int(self.DIST*z)
                else:
                    dist_l = int(DIRECTION*self.DIST)
                    dist_r = int(DIRECTION*self.DIST)

            if(self.status):
                self.robot.setMotors(dist_l, dist_r, SPEED)
            else:
                print("Chair #4 is OFF")
            
            r.sleep()
            # wait, then do it again

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off")

    def twist_handler(self, twist_msg):
        print('twist handler')
        self.latest_twist = twist_msg

    def cbon04(self, on):
        if on.data == 1:
            self.status = True
        elif on.data == 0:
            self.status = False
        print("Your are driving chair #4")

    def stopFromSleeping(self):
	'''
	This will just act like a keepalive packet mechanism.
	'''
	self.robot.setMotors(1,1,0) #do nothing actually
	print("Trying to move 1,1 at the speed of 0! keepalive!")
	

if __name__ == "__main__":
    robot = NeatoNode()
    robot.spin()

