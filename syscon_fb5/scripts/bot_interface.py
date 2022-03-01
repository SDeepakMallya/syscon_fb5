#!/usr/bin/env python3
import rospy
from serial import Serial 
# import time
import struct
from syscon_fb5.msg import PwmInput
from syscon_fb5.msg import EncoderData
ser=Serial('/dev/ttyUSB0',9600)

ser.flushInput()
ser.flushOutput()
ser.write(bytes('8', 'utf-8'))
bytes_A = bytes('A', 'utf-8')

def callback(data):

	# ~ is \x7E
	ser.write(bytes_A)
	rightPWM = data.rightInput
	leftPWM = data.leftInput
	print(data)

	print("LeftPWM: {}, RightPWM: {}".format(leftPWM, rightPWM))

	#Ensuring the wheels are set to rotate in the correct direction for each scenario.
	if (rightPWM>=0) and (leftPWM>=0):
		ser.write(bytes('8', 'utf-8'))
	elif (rightPWM<0) and (leftPWM>=0):
		ser.write(bytes('6', 'utf-8'))
	elif (rightPWM>=0) and (leftPWM<0):
		ser.write(bytes('4', 'utf-8'))
	elif (rightPWM<0) and (leftPWM<0):
		ser.write(bytes('2', 'utf-8'))
	else:
		ser.write(bytes('5', 'utf-8')) # Should never happen

	#The C code on the bot accepts left motor velocity first.
	ser.write(struct.pack('>B',abs(leftPWM)))
	ser.write(struct.pack('>B',abs(rightPWM)))

def encoderOut():
	ser.write(bytes_A) 
	encoder = EncoderData()
	# encoder.stamp = 0
	# encoder.encoderR = 0
	# encoder.encoderL = 0
	# encoder.chksum = False
	enc = ser.read()

	if enc == bytes_A:
		encoder.stamp = rospy.get_time() - start_time
		encoderRightH = ser.read()
		encoderRightL = ser.read()
		encoderLeftH = ser.read()
		encoderLeftL = ser.read()
		chksum = ser.read()
		encoder.encoderR = ord(encoderRightH) * 256 + ord(encoderRightL)
		encoder.encoderL = ord(encoderLeftH) * 256 + ord(encoderLeftL)
		sum = ord(encoderRightH) + ord(encoderRightL) + ord(encoderLeftH) + ord(encoderLeftL)
		#Byte2 and byte3 are the higher significant and lower significant bits respectively
		encoder.chksum = ord(chksum) == sum%256
		#rospy.loginfo(encoder)
		pub_encoder.publish(encoder)
		sharpH = ser.read()
		sharpL = ser.read()
		print(ord(sharpH) * 256 + ord(sharpL))

if __name__ == '__main__':
	try:
		rospy.init_node('bot_interface', anonymous=True)
		rospy.Subscriber('pwm', PwmInput, callback)
		pub_encoder = rospy.Publisher('encoder', EncoderData, queue_size=10)

		start_time = rospy.get_time()

		rate = rospy.Rate(10) #Since bot sends data at 25hz the publisher will be forced to slow down
		while not rospy.is_shutdown():
			encoderOut()
			# rate.sleep()
	except rospy.ROSInterruptException:
		pass
