#!/usr/bin/env python
from sys import version_info
import rospy
import maestro
from std_msgs.msg import String
from  odroideka.msg import Distance
from odroideka.msg import Command


PY2 = version_info[0] == 2   #Running Python 2.x?


board = maestro.Controller()
throttle_pin = 1
steering_pin = 5
left_pin = 11
right_pin = 2

def initialize():
    global board
    global throttle_pin
    global steering_pin
    
    board.setTarget(throttle_pin,6000)
    board.setTarget(steering_pin,6000)

    return

    

def send_command(cmd):
    global board
    global throttle_pin
    global steering_pin
    vel = int(1000*cmd.speed + 6000)
    ang = int(3000*cmd.turn  + 6000)

    board.setTarget(throttle_pin,vel)
    board.setTarget(steering_pin,ang)
    
    print "Recieved command %d, %d" % (cmd.speed, cmd.turn)
    return


def get_distance():
    global board
    global left_pin
    global right_pin
    left_bytes  = board.getPosition(left_pin)
    right_bytes = board.getPosition(right_pin)
    left = convert2dist(left_bytes)
    right = convert2dist(right_bytes)
    rospy.loginfo("%d, %d" % (left_bytes, right_bytes))
    #print("%d, %d" % (left_bytes, right_bytes))
    msg = Distance()
    msg.header.stamp = rospy.Time.now()
    msg.left = left
    msg.right = right
    return msg

#Coverts rolling average voltage reading to distance in cm
def convert2dist(bytes):
    bytes = float(bytes)
    distance = 195300/((8*bytes)-1767)
    if (distance > 1000):
        distance = 1000
    elif (distance < 50):
        distance = 50
    return distance
 
def get_state():
    global board
    global throttle_pin
    global steering_pin
    vel = board.getPosition(throttle_pin)
    ang = board.getPosition(steering_pin)
    vel = (vel - 6000)*0.001
    ang = (ang - 6000)*0.001
    msg = Command()
    msg.header.stamp = rospy.Time.now()
    msg.speed = vel
    msg.turn = ang
    return msg

def main():
    initialize()
    dist_pub = rospy.Publisher('distance', Distance, queue_size=10)
    command_sub = rospy.Subscriber('command', Command, send_command)
    command_pub = rospy.Publisher('state', Command, queue_size=10)
    rospy.init_node('pololu', anonymous=True)
    rate = rospy.Rate(60) #10hz, can modify this later
    while not rospy.is_shutdown():
        dist = get_distance()
        state = get_state()
        command_pub.publish(state)
        dist_pub.publish(dist)
        rate.sleep()
    initialize()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        initialize()
        print("Sumeet wasn't here")

