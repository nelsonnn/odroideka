#!/usr/bin/env python
import rospy
import turn_controller
import straight_away_controller
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

rospy.init_node('pid', anonymous=False)
command_pub = rospy.Publisher('command', Command, queue_size=1)
turn = straight_away_controller.PIDController(1.4,0.,0.005)
straight = turn_controller.PIDController(.4,0.0,0.001)
rate = rospy.Rate(60)

def straightawaycallback(dist):
    global command_pub
    global rate
    if rospy.get_param("car_state") == "straightaway":
        controls = straight.pidcontroller(dist)
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

def ballpose_callback(ballpose):
    x, r  = (ballpose.x, ballpose.z)
    straight.adjust_for_camera(x,r)

def main():
    rospy.set_param("car_state","straightaway")

    #Get filtered distance data
    dist_sub = rospy.Subscriber('filtered_distance', Distance,
    straightawaycallback)

    #IMU subscriber
    pose_sub = rospy.Subscriber('imu/data', Imu, turncallback)

    ballpose_sub = rospy.Subscriber('ballpos', Pose, ballpose_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
