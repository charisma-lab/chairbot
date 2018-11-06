#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

msg = """
Reading from the Dualshock4 controller ~!! And Publishing to neato!
---------------------------
Move around:
"""

class Joystick:

	def __init__(self, joystick_number):
		self.last_1 = 0
		self.last_2 = 0
		self.joystick_number = joystick_number
		self.Flag = False

	def joy_handler(self, ps):
		Joy_commands =  ps
		Axess = ps.axes
		Buttons = ps.buttons
		now_2 = Axess[7]
		allB = Buttons[4]
		if (self.joystick_number == 0):
			if ((self.last_2==(-0.0) or self.last_2==1) and now_2==1):
				self.Flag = True
			elif ((self.last_2==(-0.0) or self.last_2==-1) and now_2==-1):
				self.Flag = False
		elif (self.joystick_number == 1):
			if ((self.last_2==(-0.0) or self.last_2==1) and now_2==1):
				self.Flag = True
			elif ((self.last_2==(-0.0) or self.last_2==-1) and now_2==-1):
				self.Flag = False
		elif (self.joystick_number == 2):
			if ((self.last_2==(-0.0) or self.last_2==1) and now_2==1):
				self.Flag = True
			elif ((self.last_2==(-0.0) or self.last_2==-1) and now_2==-1):
				self.Flag = False
		elif (self.joystick_number == 3):
			if ((self.last_2==(-0.0) or self.last_2==1) and now_2==1):
				self.Flag = True
			elif ((self.last_2==(-0.0) or self.last_2==-1) and now_2==-1):
				self.Flag = False
		self.last_2=now_2
		self.send_commands(Joy_commands)

	def send_commands(self, ps):
	  	if (self.Flag):
			if (self.joystick_number == 0):
				pubjoy01.publish(ps)
				print "Publishing to 1"
			elif (self.joystick_number == 1):
				pubjoy02.publish(ps)
				print "Publishing to 2"
			elif (self.joystick_number == 2):
				pubjoy03.publish(ps)
				print "Publishing to 3"
			elif (self.joystick_number == 3):
				pubjoy04.publish(ps)
				print "Publishing to 4"
	  	elif (not self.Flag):
			if (self.joystick_number == 0):
				print "Not publishing to 1"
			elif (self.joystick_number == 1):
				print "Not publishing to 2"
			elif (self.joystick_number == 2):
				print "Not publishing to 3"
			elif (self.joystick_number == 3):
				print "Not publishing to 4"

def listener(js0,js1,js2,js3):
  rospy.Subscriber("/joystick0", Joy , js0.joy_handler)
  rospy.Subscriber("/joystick1", Joy , js1.joy_handler)
  rospy.Subscriber("/joystick2", Joy , js2.joy_handler)
  rospy.Subscriber("/joystick3", Joy , js3.joy_handler)
  rospy.spin()

if __name__ == '__main__':
  joystick0 = Joystick(joystick_number=0)
  joystick1 = Joystick(joystick_number=1)
  joystick2 = Joystick(joystick_number=2)
  joystick3 = Joystick(joystick_number=3)
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('joystick_all', anonymous=True)
  pubjoy01 = rospy.Publisher("/joy01", Joy, queue_size=10)
  pubjoy02 = rospy.Publisher("/joy02", Joy, queue_size=10)
  pubjoy03 = rospy.Publisher("/joy03", Joy, queue_size=10)
  pubjoy04 = rospy.Publisher("/joy04", Joy, queue_size=10)
  
  try:
	print msg
	listener(joystick0,joystick1,joystick2,joystick3)
  except rospy.ROSInterruptException:
	pass