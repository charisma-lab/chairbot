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
        self.touch9 = False
        self.touch10 = False
        self.touch11 = False

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

            # CHECK IF CONTROLLER BUTTONS ARE PRESSED
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

            # CHECK IF TOUCH SENSORS ARE PRESSED
            if (self.touch0 or self.touch1):
                pass
            else:
                if (self.touch2 or self.touch3):
                    self.fwd()
                if (self.touch4 or self.touch5):
                    self.back()
                if (self.touch6 or self.touch7):
                    self.right()
                if (self.touch8 or self.touch9):
                    self.left()
                if (self.touch10 or self.touch11):
                    self.slowFwd()  

            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off")

    # SQUARE
    def optiona(self):
        self.walk1Forward()

    # TRIANGLE
    def optionb(self):
        self.walk1Forward()

    # CIRCLE
    def optionc(self):
        self.walk1Forward()

    # CROSS
    def optiond(self):
        self.walk1Forward()

    # LEFT BUTTON
    def optione(self):
        self.walk1Forward()

    # RIGHT BUTTON
    def optionf(self):
        self.walk1Forward()

    # PROGRAMMED FUNCTIONS


    # TOUCH SENSOR FUNCTIONS
    def fwd(self):
        SPEED=150
        self.robot.setMotors(-500,-500,SPEED)
        rospy.sleep(1.75)

    def slowFwd(self):
        SPEED=75
        self.robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

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


    def joy_handler(self, ps):
        self.Butt =  ps.buttons
        self.Axess = ps.axes

    def cbon04(self, on):
        if on.data == 1:
            pass
        elif on.data == 0:
            pass

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
            self.touch6 = True
        if self.touch_number == 7:
            self.touch7 = True
        if self.touch_number == 8:
            self.touch8 = True
        if self.touch_number == 9:
            self.touch9 = True
        if self.touch_number == 10:
            self.touch10 = True
        if self.touch_number == 11:
            self.touch11 = True

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
        if self.touch_number == 18:
            self.touch5 = False
        if self.touch_number == 19:
            self.touch7 = False
        if self.touch_number == 20:
            self.touch8 = False
        if self.touch_number == 21:
            self.touch9 = False
        if self.touch_number == 22:
            self.touch10 = False
        if self.touch_number == 23:
            self.touch11 = False

if __name__ == "__main__":
    robot = NeatoNode()
    robot.spin()
