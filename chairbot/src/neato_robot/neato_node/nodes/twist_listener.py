#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos
from what_is_my_name import chairbot_number

from geometry_msgs.msg import Twist
import time
from std_msgs.msg import UInt16
from std_msgs.msg import Int8

from neato_driver.neato_driver import Botvac

class NeatoNode:

    def __init__(self):
	self.chairbot_number = chairbot_number()
        self.teleop_node_name = 'teleop' + self.chairbot_number
	self.twist_topic_name = 'twist' + self.chairbot_number
	self.cbon_topic_name = 'cbon' + self.chairbot_number

    	rospy.init_node(self.teleop_node_name)

    	self.port = rospy.get_param('~port1 ', "/dev/ttyACM1")
    	rospy.loginfo("Using port: %s"%(self.port))

    	self.robot = Botvac(self.port)
	
	rospy.loginfo("Object init")
    	rospy.Subscriber(self.twist_topic_name, Twist, self.twist_handler, queue_size=10)
    	rospy.Subscriber(self.cbon_topic_name, Int8, self.cbon04, queue_size=10)

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
                print("Chair #" + self.chairbot_number + "is OFF")
            
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
        print("You are driving chair " + self.chairbot_number)


if __name__ == "__main__":
    robot = NeatoNode()
    robot.spin()

