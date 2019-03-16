#!/usr/bin/env python
import argparse
import numpy as np
import rospy
from odroideka.msg import Distance

parser = argparse.ArgumentParser(description="ROS node to average IR distance measurements")
parser.add_argument('-n', '--number-to-avg', type=int, required=False,
default=32,
                    help='Number of readings to take the average of')
args = parser.parse_args()

n = args.number_to_avg
history = np.zeros(n)
i = 0
dist_publisher = rospy.Publisher('filtered_distance', Distance, queue_size = 16)

def time_filter(msg):
    global n
    global history
    global i

    history[i] = msg.dist
    i += 1

    avg = np.mean(history)
    
    if i == n:
        i = 0
        print "Avg distance reading: %f.3" % avg

    #ROS stuff from here
    global dist_publisher
    dist_publisher.publish(avg)
    return

if __name__ == '__main__':
    rospy.init_node('filter', anonymous=True)
    rospy.Subscriber("distance",Distance,time_filter)
    rospy.spin()
    
      
