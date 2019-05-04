import rospy
import tf
from odroideka.msg import Command

PI = 3.1415

class PIDController():
    def __init__(self,_P,_I,_D):
        self.goal = self.update_goal(0)
        self.P = _P
        self.I = _I
        self.D = _D
        self.t0 = rospy.get_time()
        self.eI = 0
        self.e0 = 0
        return

    def update_goal(self,current):
        self.goal = self.current - PI/2
        return

    def get_controls(self,pose):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        current = euler[2]
        error = self.goal - current
        if abs(error) < self.thresh:
            self.update_goal(current)
            self.release()
            msg = Command()
            msg.header.stamp = rospy.Time.now()
            msg.speed = 0.1
            msg.turn = 0.0
            return msg

        t1 = rospy.get_time()

        #Compute Integral Term
        self.eI += error*(t1-self.t0)

        #Compute Derivative Term
        eD += error-self.e0/(t1-self.t0)
        self.e0 = error

        #Update time
        self.t0 = t1

        #Compose command to return
        steer = self.P*error + self.I*self.eI + self.D * eD

        msg = Command()     
        msg.header.stamp = rospy.Time.now()
        msg.speed = 0.1
        msg.turn = steer
        
        return msg




