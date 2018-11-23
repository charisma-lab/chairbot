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
        rospy.Subscriber("/cbon04", Int8, self.cbon03, queue_size=10)
        rospy.Subscriber('/touches04', Int8, self.touch_handler, queue_size=10)

        self.Axess = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
        self.Butt = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.SPEED = 150
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
            leftButt = self.Butt[4]
            rightButt = self.Butt[5]
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
            if leftButt == 1:
                self.optione()
            if rightButt == 1:
                self.optionf()

            if (self.touch6):
                pub_LED.publish(1)
            else:
                pub_LED.publish(0)
            if (self.touch0):
                self.right()
            if (self.touch1):
                self.fwd()
            if (self.touch2):
                self.left()
            if (self.touch3):
                self.back()

            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off")

    # SQUARE
    def optiona(self):
        self.walk1ForwardSlow()

    # TRIANGLE
    def optionb(self):
        self.walk1ForwardMed()

    # CIRCLE
    def optionc(self):
        self.walk1ForwardFast()
    # CROSS
    def optiond(self):
        self.shakeNo()
    # LEFT BUTTON
    def optione(self):
        self.lightup()
    # RIGHT BUTTON
    def optionf(self):
        self.turn90Right()
        self.walk1Forward()
        self.turn450Left()
        self.walk1Backward()
        self.shakeNo()
        self.turn90Right()

    # programmed routine
    def SideToSide(self):
        SPEED=150
        self.robot.setMotors(-105,105,SPEED)
        rospy.sleep(2)
        self.robot.setMotors(210,-210,SPEED)
        rospy.sleep(2)
        self.robot.setMotors(-210,210,SPEED)
        rospy.sleep(2)

    def walk1ForwardSlow(self):
        SPEED=100
        self.robot.setMotors(-300,-300,SPEED)
        rospy.sleep(3)
 
    def walk1ForwardMed(self):
        SPEED=200
        self.robot.setMotors(-300,-300,SPEED)
        rospy.sleep(3)

    def walk1ForwardFast(self):
        SPEED=300
        self.robot.setMotors(-300,-300,SPEED)
        rospy.sleep(3)

    def shakeNo(self):
        SPEED=275
        self.robot.setMotors(-120,120,SPEED)
        rospy.sleep(1.15)
        self.robot.setMotors(235,-235,SPEED)
        rospy.sleep(1.50)
        self.robot.setMotors(-210,210,SPEED)
        rospy.sleep(1.25)
        self.robot.setMotors(120,-120,SPEED)
        rospy.sleep(1.15)

    def walk3Forward(self):
        SPEED=150
        self.robot.setMotors(-300,-300,SPEED)
        rospy.sleep(2.5)
        
    def walk8Forward(self):
        SPEED=300
        self.robot.setMotors(-800,-800,SPEED)
        rospy.sleep(3.5)

    def walk1Backward(self):
        SPEED=150
        self.robot.setMotors(100,100,SPEED)
        rospy.sleep(1)

    def turn90Left(self):
        SPEED=150
        self.robot.setMotors(-210,210,SPEED)
        rospy.sleep(3)

    def turn90Right(self):
        SPEED=150
        self.robot.setMotors(235,-235,SPEED)
        rospy.sleep(1.75)

    def turn360Left(self):
        SPEED=150
        self.robot.setMotors(-815,815,SPEED)
        rospy.sleep(7)

    def turn450Left(self):
        SPEED=250
        self.robot.setMotors(-1000,1000,SPEED)
        rospy.sleep(4.25)
        SPEED=330

    def fwdFast(self):
        SPEED=175
        self.robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

    # TOUCH SENSOR FUNCTIONS
    def fwd(self):
        SPEED=150
        self.robot.setMotors(-500,-500,SPEED)
        rospy.sleep(1.75)

    def back(self):
        SPEED=150
        self.robot.setMotors(500,500,SPEED)
        rospy.sleep(1.75)

    def right(self):
        SPEED=150
        self.robot.setMotors(200,-200,SPEED)
        rospy.sleep(1)

    def left(self):
        SPEED=150
        self.robot.setMotors(-200,200,SPEED)
        rospy.sleep(1)

    def lightup(self):
        pub_LED.publish(1)
        rospy.sleep(0.5)
        pub_LED.publish(0)
        rospy.sleep(0.5)
        pub_LED.publish(1)
        rospy.sleep(0.5)
        pub_LED.publish(0)
        rospy.sleep(0.5)
        pub_LED.publish(1)
        rospy.sleep(20)

    def joy_handler(self, ps):
        self.Butt =  ps.buttons
        self.Axess = ps.axes

    def cbon03(self, on):
        pub_LED.publish(on.data)
        #print(on.data)
        if on.data == 1:
            self.touch6=True
        elif on.data == 0:
            self.touch6=False

    def touch_handler(self, msg):
        self.touch_number = msg.data
        if self.touch_number == 0:
            self.touch0 = True
        if self.touch_number == 1:
            self.touch1 = True
        if self.touch_number == 2:
            self.touch2 = True
        if self.touch_number == 3:
            self.touch3 = True
        if self.touch_number == 4:
            self.touch4 = True
        if self.touch_number == 5:
            self.touch5 = True
        if self.touch_number == 6:
            self.touch6 = not self.touch6
        if self.touch_number == 7:
            self.touch7 = True
        if self.touch_number == 8:
            self.touch8 = True

        if self.touch_number == 12:
            self.touch0 = False
        if self.touch_number == 13:
            self.touch1 = False
        if self.touch_number == 14:
            self.touch2 = False
        if self.touch_number == 15:
            self.touch3 = False
        if self.touch_number == 16:
            self.touch4 = False
        if self.touch_number == 17:
            self.touch5 = False
        if self.touch_number == 19:
            self.touch7 = False
        if self.touch_number == 20:
            self.touch8 = False

if __name__ == "__main__":
    robot = NeatoNode()
    pub_LED = rospy.Publisher("/led04", Int8, queue_size=10)
    robot.spin()
