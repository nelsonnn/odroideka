#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command

command_pub = rospy.Publisher('command', Command, queue_size=10)

# Will need to update these
dist_prop = 0.5
max_dist = 1024
max_steer = 40
steer_prop = 0.3
max_speed  = .25

class PID():

    def pidcontroller(dist, steer_angle):
        global dist_prop
        global max_dist
        global max_steer
        global steer_prop
        global max_speed

        # Possibly update later if we want to change the desired distance
        des_dist = 200 
        err = dist - des_dist
        # print(err)
        # Set to a constant for now, will change later
        des_speed = .25 #abs((err * dist_prop) / max_dist)
        # des_steer_angle = steer_angle / max_steer
        des_steer = (((err * steer_prop) * ((max_steer - steer_angle) / max_steer)))/40
        print("Distance: {}, Steering Angle: {}, Desired Speed: {}, Desired Steering Angle: {}".format(dist, steer_angle, des_speed, des_steer))
        
        return des_speed, des_steer

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

