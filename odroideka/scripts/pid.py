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
steer_prop = 0.9
max_speed  = .25


def pidcontroller(dist, cmd):
    global dist_prop
    global max_dist
    global max_steer
    global steer_prop
    global max_speed
    global commmand_pub
    steer_angle = cmd.turn * max_steer
    # Possibly update later if we want to change the desired distance
    des_dist = 146 
    err = dist.right - des_dist
    #print(err)
    # Set to a constant for now, will change later
    des_speed = .25 #abs((err * dist_prop) / max_dist)
    # des_steer_angle = steer_angle / max_steer
    #des_steer = (((err * steer_prop) * ((max_steer - abs(steer_angle)) / max_steer)))/max_steer
    des_steer = (err * steer_prop)/max_steer
    print("Distance: {}, Steering Angle: {}, Desired Speed: {}, Desired Steering Angle: {}".format(dist, steer_angle, des_speed, des_steer))

    #Give to pololu
    msg = Command()
    msg.header.stamp = rospy.Time.now()
    msg.speed = des_speed
    msg.turn = des_steer
    command_pub.publish(msg)

    return

def main():
    #Get filtered distance data and state data
    dist_sub = message_filters.Subscriber('filtered_distance', Distance)
    state    = message_filters.Subscriber('state', Command)
    
    #Time synchronize it
    ts = message_filters.ApproximateTimeSynchronizer([dist_sub, state], 10, .1)
    ts.registerCallback(pidcontroller)
    
    rospy.init_node('pid', anonymous=False)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

