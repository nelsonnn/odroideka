#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command

command_pub = rospy.Publisher('command', Command, queue_size=10)

class PID():
    def __init__(self):
        pass

#distance data comes through here
def callback(data):
    print("Got: ", data.dist) 

def main():
    dist_sub = rospy.Subscriber('distance', Distance, callback)
    rospy.init_node('pid', anonymous=False)
    #test send_command 
    while not rospy.is_shutdown():
        send_command()


#send speed and turn commands b/w [-1, 1]
def send_command():
    msg = Command()
    msg.speed = 0.5
    msg.turn = -0.5
    command_pub.publish(msg)

if __name__ == '__main__':
    pid = PID()
    try:
        main()
    except rospy.ROSInterruptException:
        pass

