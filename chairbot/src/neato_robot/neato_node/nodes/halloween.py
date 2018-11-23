#!/usr/bin/env python

import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos

from geometry_msgs.msg import Twist
import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

from neato_driver.neato_driver import Botvac

class NeatoNode:

	def __init__(self):
		""" Start up connection to the Neato Robot. """
		rospy.init_node('teleop04', anonymous=True)

		self.port = rospy.get_param('~port1 ', "/dev/ttyACM1")
		rospy.loginfo("Using port: %s"%(self.port))

		self.robot = Botvac(self.port)

		rospy.Subscriber("/joy04", Joy, self.joy_handler, queue_size=10)
		rospy.Subscriber("/cbon04", Int8, self.cbon04, queue_size=10)
		# rospy.Subscriber('/touches02', Int8, self.touch_handler, queue_size=10)
		self.Axess = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
		self.Butt = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
		self.SPEED = 0
		self.DIST = 20
		self.SPEEDSET = 0
		self.SPEEDRAMP = 0
		self.lastx = 0
		self.lasty = 0
		self.xramp = 0
		self.yramp = 0
		self.touch_number = -1
		self.touch0 = False
		self.touch1 = False
		self.touch2 = False
		self.touch3 = False
		self.touch4 = False
		self.touch5 = False
		self.touch6 = False
		self.touch7 = False
		self.touch8 = False

	def spin(self):        
		
		# main loop of driver
		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			Lft_t = self.Axess[0]
			Lft_d = self.Axess[1]
			Rgh_t = self.Axess[3]
			Rgh_d = self.Axess[4]
			AageL = self.Axess[2]
			AageR = self.Axess[5]
			L_R = self.Axess[6]
			F_B = self.Axess[7]
			sq = self.Butt[0]
			xx = self.Butt[1]
			ci = self.Butt[2]
			tr = self.Butt[3]
			self.SPEED_s = self.Butt[4]
			self.SPEED_f = self.Butt[5]
			AageL_Button = self.Butt[6]
			AageR_Button = self.Butt[7]
			share = self.Butt[8]
			options = self.Butt[9]
			pressL = self.Butt[10]
			pressR = self.Butt[11]
			power = self.Butt[12]
			self.SPEED -= ((AageR-1)*10)
			self.SPEED += ((AageL-1)*10)
			self.SPEED = int(self.SPEED)
			if (self.SPEED<0):
				self.SPEED=0
			elif (self.SPEED>330):
				self.SPEED=330
			
			self.SPEEDSET = self.SPEED

			ll = (Lft_d*self.DIST)
			rr = (Rgh_t*self.DIST)
			if (rr>=0):
				x = (-ll - rr)
				y = (-ll + rr)
			else:
				x = (-ll - rr)
				y = (-ll + rr) 

			x=int(x)
			y=int(y)

			speeddif = abs(self.SPEEDRAMP - self.SPEEDSET)

			if (self.SPEEDRAMP<self.SPEEDSET):
				self.SPEEDRAMP += (speeddif/20)
			else:
				self.SPEEDRAMP -= (speeddif/20)

			if (self.SPEEDRAMP<0):
				self.SPEEDRAMP=0
			elif (self.SPEEDRAMP>330):
				self.SPEEDRAMP=330

			if (self.SPEEDSET > 150):
				if (0<x<10):
					x=10
					if (self.SPEEDRAMP>150):
						self.SPEEDRAMP = 150
				elif (-10<x<0):
					x=-10
					if (self.SPEEDRAMP>150):
						self.SPEEDRAMP = 150

				if (0<y<10):
					y=10
					if (self.SPEEDRAMP>150):
						self.SPEEDRAMP = 150
				elif (-10<y<0):
					y=-10
					if (self.SPEEDRAMP>150):
						self.SPEEDRAMP = 150
			else:
				if (0<x<5):
					x=5
				elif (-5<x<0):
					x=-5

				if (0<y<5):
					y=5
				elif (-5<y<0):
					y=-5
			
			#self.xramp = x 
			#self.yramp = y

			if (self.xramp < self.lastx):
				self.xramp += 1
			elif (self.xramp == self.lastx):
				pass
			else:
				self.xramp -= 1

			if (self.yramp < self.lasty):
				self.yramp += 1
			elif (self.yramp == self.lasty):
				pass
			else:
				self.yramp -= 1


			if (x==0 and y==0):
				self.SPEEDRAMP -= (self.SPEEDSET/10)
			else:
				if ((abs(self.xramp-x)>20) or (abs(self.yramp-y)>20)):
					self.SPEEDRAMP = 50
			
			if (self.SPEEDRAMP<0):
				self.SPEEDRAMP=0

			self.lastx = x
			self.lasty = y
			print (self.xramp, x, self.lastx, self.yramp, y, self.lasty, self.SPEEDRAMP, self.SPEEDSET)
			# if (self.touch6):
			self.robot.setMotors(self.xramp, self.yramp, self.SPEEDRAMP)
			self.robot.flushing()

			if tr == 1:
				self.optiona()
			if ci == 1:
				self.optionb()
			if xx == 1:
				self.optionc()
			if sq == 1:
				self.optiond()

			# if (self.touch6):
			# 	pub_LED.publish(1)
			# 	if (self.touch0 or self.touch1):
			# 		self.left()
			# 	if (self.touch3 or self.touch2):
			# 		self.right()
			# 	if (self.touch4):
			# 		self.back()
			# 	if (self.touch5):
			# 		self.fwd()
			# elif (self.touch7):
			# 	pub_LED.publish(1)
			# 	if (self.touch0 or self.touch1):
			# 		self.leftFast()
			# 	if (self.touch3 or self.touch2):
			# 		self.rightFast()
			# 	if (self.touch4):
			# 		self.backFast()
			# 	if (self.touch5):
			# 		self.fwdFast()
			# else:
			# 	pub_LED.publish(0)

			# wait, then do it again
			r.sleep()

		# shut down
		self.robot.setLDS("off")
		self.robot.setTestMode("off") 

	# SQUARE
	def optiona(self):
		self.lightup(1,3)
		self.lightup(0,1)
		self.lightup(1,1)
		self.lightup(0,1)
		self.lightup(1,1)
		self.lightup(0,1)
		self.lightup(1,1)

		self.lightup(0,1)


	# TRIANGLE
	def optionb(self):
		# self.right(325,300)# almost 120 degree 
		# self.fwd()
		BEET=.98
		#Enter
		

		self.fwd(1000,100,BEET)
		self.right(300,130,BEET)
		self.fwd(1000,100,BEET)
		self.right(300,130,BEET)

		#03 forward
		self.fwd(0,0,7*BEET)
		self.fwd(0,0,BEET)

		# shake
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)

		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.left(100,50,0.5*BEET)
		self.right(100,50,0.5*BEET)
		self.fwd(0,0,0.5*BEET)

		#Turn pause, turn pause
		self.left(400,220,3*BEET)
		self.fwd(0,0,0.05*BEET)
		self.left(600,250,BEET)
		self.fwd(0,0,0.1*BEET)

		# self.lightup(1,BEET)
		# self.lightup(0,0.5*BEET)
		# self.lightup(1,BEET)

		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)

		self.right(300,120,3*BEET)
		self.fwd(0,0,BEET)

		self.right(300,150,0.55*BEET)
		self.left(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)
		self.left(300,150,0.55*BEET)
		self.right(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)

		self.right(300,150,0.55*BEET)
		self.left(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)
		self.left(300,150,0.55*BEET)
		self.right(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)

		self.fwd(0,0,0.5*BEET)

		self.back(1000,100,0.5*BEET)
		self.back(0,0,0.5*BEET)
		self.back(1000,100,0.5*BEET)
		self.back(0,0,0.5*BEET)
		self.back(1000,100,0.5*BEET)
		self.back(0,0,0.5*BEET)
		self.back(1000,100,0.5*BEET)
		self.back(0,0,0.5*BEET)
		self.left(300,100,2*BEET)

		self.left(600,250,BEET)
		self.fwd(0,0,BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)

		self.right(600,250,2*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)

		self.left(2200,150,12*BEET)
		self.fwd(0,0,2*BEET)

	#Thriller 
		self.right(300,150,0.55*BEET)
		self.left(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)
		self.left(300,150,0.55*BEET)
		self.right(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)

		self.left(600,250,BEET)
		self.fwd(0,0,BEET)
		self.right(50,50,0.5*BEET)
		self.left(50,50,0.5*BEET)
		self.right(50,50,0.5*BEET)
		self.left(50,50,0.5*BEET)

		#Thriller
		self.right(300,150,0.55*BEET)
		self.left(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)
		self.left(300,150,0.55*BEET)
		self.right(300,150,0.55*BEET)
		self.fwd(1000,100,BEET)

		self.right(100,200,0.5*BEET)

		self.left(600,250,BEET)
		self.fwd(0,0,2*BEET)

		self.right(50,50,0.5*BEET)
		self.left(50,50,0.5*BEET)

		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)

		self.fwd(0,0,2*BEET)
		self.left(600,250,BEET)

		self.right(50,50,0.5*BEET)
		self.left(50,50,0.5*BEET)
		
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)
		self.fwd(1000,100,0.5*BEET)
		self.fwd(0,0,0.5*BEET)


	# CIRCLE IS FOR TESTING
	def optionc(self):
		BEET=.98
		#Enter
		self.fwd(1000,100,BEET)
		self.right(300,130,BEET)
		self.fwd(1000,100,BEET)
		self.right(300,130,BEET)
		

	# X
	def optiond(self):
		self.lightup()
		
	def fwd(self,DISTANCE,SPEED,SLEEP):
		self.robot.setMotors(-DISTANCE,-DISTANCE,SPEED)
		rospy.sleep(SLEEP)

	def back(self,DISTANCE,SPEED,SLEEP):
		self.robot.setMotors(DISTANCE,DISTANCE,SPEED)
		rospy.sleep(SLEEP)


	def right(self,ANGLE,SPEED,SLEEP):
		self.robot.setMotors(ANGLE,-ANGLE,SPEED)
		rospy.sleep(SLEEP)

	def left(self,ANGLE,SPEED,SLEEP):
		self.robot.setMotors(-ANGLE,ANGLE,SPEED)
		rospy.sleep(SLEEP)

	def stop(self):
		SPEED=00
		self.robot.setMotors(00,00,SPEED)
		rospy.sleep(1)
		self.robot.setMotors(00,00,SPEED)

	def lightup(self,SWITCH,BEET):
	 	pub_LED.publish(SWITCH)
		rospy.sleep(BEET)

	def joy_handler(self, ps):
		self.Butt =  ps.buttons
		self.Axess = ps.axes

	def cbon04(self, on):
	 	pub_LED.publish(on.data)
	 	print(on.data)
	 	if on.data == 1:
	 		self.touch6=True
	 	elif on.data == 0:
	 		self.touch6=False

if __name__ == "__main__":    
	robot = NeatoNode()
	pub_LED = rospy.Publisher("/led04", Int8, queue_size=10)
	robot.spin()