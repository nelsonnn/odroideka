#!/usr/bin/env python
import argparse
import numpy as np
import rospy
from odroideka.msg import Distance

n = 32
history = np.zeros((n,2))
i = 0
dist_publisher = rospy.Publisher('filtered_distance', Distance, queue_size = 16)

def time_filter(msg):
    global n
    global history
    global i

    history[i,0] = msg.left
    history[i,1] = msg.right
    i += 1

    avg = np.mean(history,axis=0)
    
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
    
      
