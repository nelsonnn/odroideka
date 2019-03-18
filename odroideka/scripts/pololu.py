#!/usr/bin/env python
from sys import version_info
import rospy
import maestro
from std_msgs.msg import String
from  odroideka.msg import Distance
from odroideka.msg import Command


PY2 = version_info[0] == 2   #Running Python 2.x?

board = maestro.Controller()

def send_command(cmd):
    global board
    global throttle_pin
    global steering_pin
    vel = 1000*cmd.speed + 6000
    ang = 1000*cmd.turn  + 6000

    board.setTarget(throttle_pin,vel)
    board.setTarget(steering_pin,ang)

    return


def get_distance():
    global board
    global left_pin
    global right_pin
    left_bytes  = board.getPosition(left_pin)
    right_bytes = board.getPosition(right_pin)
    left = convert2dist(left_bytes)
    right = convert2dist(right_bytes)
    return (left, right)


def convert2dist(bytes):
    #Ryan TODO
    return bytes
 

def main():
    dist_pub = rospy.Publisher('distance', Distance, queue_size=10)
    command_sub = rospy.Subscriber('command', Command, send_command)
    rospy.init_node('pololu', anonymous=True)
    rate = rospy.Rate(60) #10hz, can modify this later
    while not rospy.is_shutdown():
        dist = get_distance()
        dist_pub.publish(dist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Sumeet wasn't here")

