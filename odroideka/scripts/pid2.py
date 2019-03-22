#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from odroideka.msg import Distance
from odroideka.msg import Command
import message_filters

command_pub = rospy.Publisher('command', Command, queue_size=10)
pid = PID()

class PID(kp=0.25, ki=0, kd=0.25):
    def __init__(self):
        self.kp = kp 
        self.ki = ki
        self.kd = kd
        self.max_speed = 0.3
        self.prev_error = 0
        self.error = 0
        self.prevtime = None
        self.max_dist = 1024
        self.dist_prop = 0.5
        self.wr = 0.8
        self.wl = 1 - self.wr
        self.des_dist_r = 130
        self.des_dist_l = 120
        self.max_steer = 40

    def run(self, dist, cmd):
        err_r = dist.right -  self.des_dist_r
        err_l = -(dist.left - self.des_dist_l)
        if err_l > 100:
            self.error = err_r
        else:
            self.error = self.wr * err_r + self.wl * err_l

        if self.prevtime is not None:
            dt = rospy.get_time() - self.prevtime
            self.prevtime = rospy.get_time()
        else:
            self.prevtime = rospy.get_time()


        de = self.error - self.prev_error
        self.prev_error = self.error

        dedt = de / dt
        d = self.kd * dedt
        p = ((self.kp * error)/self.max_steer)

        steer_angle = cmd.turn * self.max_steer
        des_speed = 0.35
        des_steer = p + d

        msg = Command()
        msg.header.stamp = rospy.Time.now()
        msg.speed = des_speed
        msg.turn = des_steer
        return msg


def callback(dist, cmd):
    global pid
    global command_pub
    msg = pid.run(dist, cmd)
    command_pub.publish(msg)


def main():
    # Get filtered distance data and state data
    dist_sub = message_filters.Subscriber('filtered_distance', Distance)
    state = message_filters.Subscriber('state', Command)

    rospy.init_node('pid', anonymous=False)
    # Time synchronize it
    ts = message_filters.ApproximateTimeSynchronizer([dist_sub, state], 10, .1)
    ts.registerCallback(pidcontroller)

    rospy.spin()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
