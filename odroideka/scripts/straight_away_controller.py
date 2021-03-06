#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
import message_filters

# Controller constants
MAX_SPEED  = 0.3
NUM_MEASUREMENTS = 255 # For calculating width of corridor
TURN_THRESH = .5 # Cross-track error
SAFE_D = 100 # Derivative threshold for detecting turns
MAX_CORD_WIDTH = 290 #??? Might be easier to hard-code this in than try and estimate on the fly
i = 0

class PIDController():
    def __init__(self,_P,_I,_D,safe=.17):
        #Gains
        self.P = _P
        self.I = _I
        self.D = _D

        #To try and make it robust, though might just want to hard-code these in
        self.corridor_width = 286
        self.corridor_measurements = [286]
        self.target_cte = 0.5

        #Controller variables
        self.prevtime = rospy.get_time()
        self.prevdist = 0
        self.err_I = 0
        self.prevCmd = Command()

        #Turn params
        self.prev_err_r = 0
        self.err_rD = 0

    def pidcontroller(self,dist):
	global i
        #Update what we think the corridor looks like, hoping that there are
        #more good measurments than false ones
        if dist.right + dist.left < MAX_CORD_WIDTH:
            if len(self.corridor_measurements) < NUM_MEASUREMENTS:
                self.corridor_measurements.append(dist.left + dist.right)

            else:
                self.corridor_measurements[i] = dist.left + dist.right
                i = (i+1) % NUM_MEASUREMENTS

       #Use the median here to fend off outliers
        self.corridor_width = np.median(self.corridor_measurements)

        #Calculate cross-track error from both sensors
        err_l = dist.left/self.corridor_width - self.target_cte #If dist.left is small this is negative --> turn right
        err_r = (1-self.target_cte) - dist.right/self.corridor_width  #If dist.right is large this is negative --> turn right

        #Always pick the one which errs on the side of less control input to filter out doorways and such
        if abs(err_l) > abs(err_r):
            err = err_r
        else:
            err = err_l

        # We need the time for obvious reasons
        t1 = rospy.get_time()

        #Compute Integral Term
        self.err_I += err*(t1-self.prevtime)

        #Compute Derivative Term, both for general error and for right sensor alone.
        dt = 1/(t1-self.prevtime)
        err_D = err-self.prevdist * dt
        err_rD = err_r - self.prev_err_r * dt
        self.prevdist = err
        self.prev_err_r = err_r
        self.prevtime = t1

        #'gnostics
        print("L: %f R: %f AVG: %f D: %f" % (err_l, err_r, (err_r - err_l)/2, err_rD))

        #Detect the turn, try to filter out falsies while we're unsure of the width of the corridor
        if (abs(err_r) > TURN_THRESH) and len(self.corridor_measurements) > NUM_MEASUREMENTS/1.25:
            self.release()
            return self.prevCmd
        #Set desired steering angle, constant speed
        des_steer = -( (err * self.P) + (self.err_I * self.I) + (err_D * self.D) )
        des_speed = MAX_SPEED
	des_steer = np.clip(des_steer,-1,1)
        #Set message
        msg = Command()
        msg.header.stamp = rospy.Time.now()
        msg.speed = des_speed
        msg.turn = des_steer
        self.prevCmd = msg
	#print(des_steer)
        #return to controller
        return msg

    def adjust_for_camera(self,camx,camr):
        if camx >= .5: #Ball on right
            safe = camx - 1.1*camr
            if safe <= .5 + self.safe_zone: #Left side of ball in center of frame
                self.target_cte -= 0.05
                self.target_cte = np.clip(self.target_cte,.35,.65)
        else: #Ball on left
            safe = camx + 1.1*camr
            if safe >= .5 - self.safe_zone: #Right side of ball in center of frame
                self.target_cte += 0.05
                self.target_cte = np.clip(self.target_cte,.35,.65)

    def release(self):
        #Give up control to IMU, reset corridor measurments because we might
        #be in a new corridor when control returns. We don't need to reset i to
        #0 because the order measurments are taken in the corridor isn't relevant
        rospy.set_param("car_state","turn")
        self.corridor_measurements = [286]
