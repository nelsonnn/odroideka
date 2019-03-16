import math

dist_prop = 0.5
max_dist = 1024
max_steer = 40
steer_prop = 0.3
max_speed  = .25

#Desired steering [-1, 1]
#Desired speed [-1, 1]

class PidController:

	def pidcontroller(dist, steer_angle):
		global dist_prop
		global max_dist
		global max_steer
		global steer_prop
		global max_speed

		des_dist = 150 
		err = dist - des_dist
		des_speed = (err * dist_prop) / max_dist
		# des_steer_angle = steer_angle / max_steer
		des_steer = (((err * steer_prop) * ((max_steer - steer_angle) / max_steer))%360)/360
		print("Distance: {}, Steering Angle: {}, Desired Speed: {}, Desired Steering Angle: {}".format(dist, steer_angle, des_speed, des_steer))
		
		return des_speed, des_steer

def main():
	dist = 1050
	steer_angle = .5
	PidController.pidcontroller(dist, steer_angle)

if __name__ == '__main__':
	main()

