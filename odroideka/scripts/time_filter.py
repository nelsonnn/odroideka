#!/usr/bin/env python
import argparse
import numpy as np
import rospy
from odroideka.msg import Distance

n = 3 
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
        print "Avg left: %f.3 \nAvg right: %f.3" % (msg.left,msg.right)

    #ROS stuff from here
    global dist_publisher
    new_msg = Distance()
    new_msg.header.stamp = msg.header.stamp
    new_msg.left = avg[0]
    new_msg.right = avg[1]
    dist_publisher.publish(new_msg)
    return

if __name__ == '__main__':
    rospy.init_node('filter', anonymous=True)
    rospy.Subscriber("distance",Distance,time_filter)
    rospy.spin()
    
      
