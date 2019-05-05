#!/usr/bin/env python
import rospy
import pid.py
import turn.py
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
import message_filters 

command_pub = rospy.Publisher('command', Command, queue_size=10)

def callback(dist, cmd):
    global command_pub
    # Still need control state to come from somewhere
    if control_state == "straightaway":
        controls = pidcontroller(dist,cmd)
    else:
        controls = turncontroller(dist,cmd)
    command_pub.publish(controls)

def main():
    #Get filtered distance data and state data
    dist_sub = message_filters.Subscriber('filtered_distance', Distance)
    car_state = message_filters.Subscriber('car_state', Command)
    
    rospy.init_node('controller', anonymous=False)
    #Time synchronize it
    ts = message_filters.ApproximateTimeSynchronizer([dist_sub, car_state], 10, .1)
    ts.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
