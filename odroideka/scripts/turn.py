#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
import message_filters 

command_pub = rospy.Publisher('command', Command, queue_size=10)

# Will need to update these
dist_prop = 0.5
max_dist = 1024
max_steer = 40
steer_prop = 0.25 
max_speed  = 0.3
integral_func = 0
integral_prop = 0.54*steer_prop
prevtime = None

wr = .8
wl = 1 - wr

def turncontroller(dist, cmd):
    global dist_prop
    global max_dist
    global max_steer
    global steer_prop
    global max_speed
    global commmand_pub
    global integral_func
    global integral_prop
    global prevtime
    global wr
    global wl

    # Possibly update later if we want to change the desired distanced 
    des_dist_r = 130 
    des_dist_l = 120
    err_r = dist.right - des_dist_right
    err_l = -(dist.left - des_dist_left)
    
    if err_l > 100:
        err = err_r
    else:
        err = wr*err_r + wl*err_l

    #print(err)

    #Compute integral term
    if prevtime is not None:
        deltaT = rospy.get_time() - prevtime
        prevtime = rospy.get_time()
        integral_func += deltaT*err
    else:
        prevtime = rospy.get_time()

    steer_angle = cmd.turn * max_steer
    # Set to a constant for now, will change later
    des_speed = .35 #abs((err * dist_prop) / max_dist)
    # des_steer_angle = steer_angle / max_steer
    #des_steer = (((err * steer_prop) * ((max_steer - abs(steer_angle)) / max_steer)))/max_steer
    des_steer = ((err * steer_prop)/max_steer) #+ (integral_prop*integral_func)
    print("Distance: {}, Steering Angle: {}, Desired Speed: {}, Desired Steering Angle: {}".format(dist, steer_angle, des_speed, des_steer))

    #Give to pololu
    msg = Command()
    msg.header.stamp = rospy.Time.now()
    msg.speed = des_speed
    msg.turn = des_steer
    command_pub.publish(msg)

return
