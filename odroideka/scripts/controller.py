#!/usr/bin/env python
import rospy
import pid
import imu_controller
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
from sensor_msgs.msg import Imu
import message_filters 


rospy.init_node('pid', anonymous=False)
command_pub = rospy.Publisher('command', Command, queue_size=10)
turn = imu_controller.PIDController(1.,0.,0.)
rate = rospy.Rate(60)

def straightawaycallback(dist):
    global command_pub
    global rate
    if rospy.get_param("car_state") == "straightaway":
        controls = pidcontroller(dist,cmd)
        command_pub.publish(controls)
	rate.sleep()

def turncallback(pose):
    global command_pub
    global turn
    global rate
    if rospy.get_param("car_state") == "turn":
        controls = turn.get_controls(pose)
        command_pub.publish(controls)
        rate.sleep()

def main():
    rospy.set_param("car_state","turn")

    #Get filtered distance data 
    dist_sub = rospy.Subscriber('filtered_distance', Distance,
    straightawaycallback)
    
    #IMU subscriber
    pose_sub = rospy.Subscriber('imu/data', Imu, turncallback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass