#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
import message_filters 

# Controller constants
dist_prop = 0.5
max_dist = 1024
steer_prop = 0.15/40
max_speed  = 0.3
max_deriv = 80000
class PIDController():
    def __init__(self):
            self.prevtime = rospy.get_time()
            self.prevdist = 0

    def pidcontroller(self,dist, cmd):
        global dist_prop
        global max_dist
        global steer_prop
        global max_speed
        
        # Desired distance from right and left walls 
        des_dist_r = 130
        des_dist_l = 100
    
        # Compute error terms
        err_r = dist.right - des_dist_r
        err_l = dist.left - des_dist_l
        
        # Decide error
        if err_l > 100: # Default to right error if left is very high
            err = err_r
        elif err_r > 100: # Use left error if right is unreasonably high
            err = err_l
        else: # Average the errors
            err = (err_r - err_l)/2
        #print(err)
    
        # steer_angle = cmd.turn
    
        # Make desired speed based on derivative of distance from wall
        # High derivative -> slow down, small derivative -> speed up
        # Need to compute dervative somewhere
        # deriv_prop = deriv / max_deriv
        des_speed = max_speed #* (1.01 - deriv_prop) 
    
        # Set desired steer proportional to error
        des_steer = err * steer_prop
       # print("Distance: {}, Steering Angle: {}, Desired Speed: {}, Desired Steering Angle: {}".format(dist, steer_angle, des_speed, des_steer))
    
        #Set message
        msg = Command()
        msg.header.stamp = rospy.Time.now()
        msg.speed = des_speed
        msg.turn = des_steer
        #return to controller 
        return msg

