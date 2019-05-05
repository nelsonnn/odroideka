#!/usr/bin/env python
import rospy
import pid.py
import imu_controller
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
from sensor_msgs import Imu
import message_filters 

command_pub = rospy.Publisher('command', Command, queue_size=10)
turn = imu_controller.PIDController(10.,1.,0.)

def straightawaycallback(dist, cmd):
    global command_pub
    if rospy.get_param("car_state") == "straightaway":
        controls = pidcontroller(dist,cmd)
        command_pub.publish(controls)

def turncallback(pose):
    global command_pub
    global turn
    if rospy.get_param("car_state") == "turn":
        controls = turn.get_controls(pose)
        command_pub.publish(controls)


def main():
    rospy.init_node('pid', anonymous=False)
    rospy.set_param("car_state","straightaway")

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
