#!/usr/bin/env python3
import rospy
import rospkg

from geometry_msgs.msg import TransformStamped
# from tf.msg import *
import numpy as np
import math
from syscon_fb5.msg import PwmInput
from tf.transformations import euler_from_quaternion, quaternion_from_euler

dirname = rospkg.RosPack.get_path('syscon_fb5')

class CALIBRATE:
	def __init__(self):
		self.iterations = 3
		self.intervals = 5
		self.pwms = range(0, 256, self.intervals)
		self.output_folder = dirname + '/calibration_files/'
		self.pos_x = 0.
		self.pos_y = 0.
		self.heading = 0.
		self.current_time = 0.
		self.run_duration = 10.
		self.output_file = open(self.output_folder + '/temp.csv', 'w+')
		rospy.Subscriber('/vicon/fb5_10/fb5_10', TransformStamped, s.callback_odom)
		self.execute_pwms()

	

	def callback_odom(self, data):
		self.current_time = data.header.stamp.secs
		self.pos_x = data.transform.translation.x
		self.pos_y = data.transform.translation.y
		orientation_q = data.tranform.rotation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		roll, pitch, yaw = euler_from_quaternion(orientation_list)
		self.heading = (yaw) % (2 * math.pi)
		try:
			self.output_file.write('{}, {}, {}, {}'.format(self.current_time, self.pos_x, self.pos_y, self.heading))
		except:
			print('No Output File Found!')
			pass

	def execute_pwms(self):
		self.output_file.close()
		for pwm in self.pwms:
			for run_id in range(self.iterations):
				start = rospy.get_time()
				self.output_file = open(self.output_folder + '/pos_{}_forward_{}.csv'.format(pwm, run_id), 'w+')
				while self.current_time - start < self.run_duration:
					pwm_msg = PwmInput()
					pwm_msg.rightInput = pwm
					pwm_msg.leftInput = pwm
					pub_pwm.publish(pwm_msg)
					rospy.sleep(0.1)
				self.output_file.close()

				print('balle balle')
				start = rospy.get_time()
				self.output_file = open(self.output_folder + '/pos_{}_backward_{}.csv'.format(pwm, run_id), 'w+')
				while self.current_time - start < self.run_duration:
					pwm_msg = PwmInput()
					pwm_msg.rightInput = -pwm
					pwm_msg.leftInput = -pwm
					pub_pwm.publish(pwm_msg)
					rospy.sleep(0.1)
				self.output_file.close()

				print('shawa shawa')


if __name__ == '__main__':
	try:
		rospy.init_node('auto_calibrate',anonymous=True)
		pub_pwm = rospy.Publisher('/pwmCmd2', PwmInput, queue_size=10)
		s = CALIBRATE()


		rate = rospy.Rate(30) 
		done = False
		while not done:
			rospy.spin()

	except rospy.ROSInterruptException:
		pass
