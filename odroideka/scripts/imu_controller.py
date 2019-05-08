import rospy
import tf
from odroideka.msg import Command

PI = 3.1415926535

class PIDController():
    def __init__(self,_P,_I,_D,thresh=0.5):
	self.goal = None
        self.P = _P
        self.I = _I
        self.D = _D
	self.thresh = thresh
        self.t0 = rospy.get_time()
        self.eI = 0
        self.e0 = 0
        return

    def update_goal(self,current):
        self.goal = current + .6*(PI/2)
	if self.goal > PI:
	    self.goal -= 2*PI
	elif self.goal < -PI:
	    self.goal += 2*PI
        return

    def release(self):
	rospy.set_param("car_state","straightaway")	
	return

    def get_controls(self,pose):
        quaternion = (
            pose.orientation.y,
            pose.orientation.x,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        current = euler[2]
	if not self.goal:
            self.update_goal(current)
        error = (self.goal - current)
        if abs(error) < self.thresh:
            print("Goal Reached")
            self.release()
	    self.goal = None
	    self.eI = 0
            msg = Command()
            msg.header.stamp = rospy.Time.now()
            msg.speed = 0.4
            msg.turn = 0.0
            return msg

        t1 = rospy.get_time()

        #Compute Integral Term
        self.eI += error*(t1-self.t0)

        #Compute Derivative Term
        eD = error-self.e0/(t1-self.t0)
        self.e0 = error

        #Update time
        self.t0 = t1

        #Compose command to return
        steer = self.P*error + self.I*self.eI + self.D * eD

	print("At: %f   Goal: %f D: %f" % (current,self.goal,eD))
        msg = Command()     
        msg.header.stamp = rospy.Time.now()
        msg.speed = 0.5
        msg.turn = steer
        
        return msg




