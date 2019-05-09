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
max_speed  = 0.5 #0.5
max_deriv = 3500
class PIDController():
    def __init__(self,_P,_I,_D):
	self.P = _P
        self.I = _I
        self.D = _D
        self.prevtime = rospy.get_time()
        self.prevdist = 0
	self.err_I = 0
	self.prevCmd = Command()

    def pidcontroller(self,dist):
        global dist_prop
        global max_dist
        global steer_prop
        global max_speed
        global max_deriv
        # Desired distance from right and left walls 
        des_dist_r = 130
        des_dist_l = 95 
    
        # Compute error terms
        err_r = dist.right - des_dist_r
        err_l = dist.left - des_dist_l
        
        # Decide error
        #if abs(err_l) > 50: # Default to right error if left is very high
        err = err_r
        #elif abs(err_r) > 100: # Use left error if right is unreasonably high
        #    err = -err_l
        #else: # Average the errors
        #    err = (err_r - err_l)/2
         
   
	t1 = rospy.get_time()

	#Compute Integral Term
        self.err_I += err*(t1-self.prevtime)
    
        #Compute Derivative Term
        err_D = err-self.prevdist/(t1-self.prevtime)
        self.prevdist = err
	self.prevtime = t1	
	
	print("L: %f Q: %f AVG: %f D: %f" % (err_l, err_r, (err_r - err_l)/2, err_D))
	#Throw away if we can't trust it
	if abs(err_D) > max_deriv:
            msg = Command()
            msg.header.stamp = rospy.Time.now()
            msg.speed = max_speed
            msg.turn = 0
	    return msg
	
	elif err_r > 250 and err_r < 400:
	    rospy.set_param("car_state","turn")
	    return self.prevCmd	
	#Set desired steering angle
        des_steer = err * self.P + self.err_I * self.I + err_D * self.D

        deriv_prop = err_D / max_deriv
        des_speed = max_speed #* (1.01 - deriv_prop) 
    
        #Set message
        msg = Command()
        msg.header.stamp = rospy.Time.now()
        msg.speed = des_speed
        msg.turn = des_steer
	self.prevCmd = msg

        #return to controller 
        return msg

    def release(self):
	rospy.set_param("car_state", "turn")
	return
