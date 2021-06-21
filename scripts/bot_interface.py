#!/usr/bin/env python3
import rospy
import serial 
# import time
import struct
from syscon_fb5.msg import PwmInput
from sycon_fb5.msg import EncoderData
ser=serial.Serial('/dev/ttyUSB0',9600)

ser.flushInput()
ser.flushOutput()
ser.write('8')

def callback(data):
	# ~ is \x7E
	rightPWM = data.rightInput
	leftPWM = data.leftInput

	#Ensuring the wheels are set to rotate in the correct direction for each scenario.
	if (rightPWM>=0) and (leftPWM>=0):
		ser.write('8')
	elif (rightPWM<0) and (leftPWM>=0):
		ser.write('6')
	elif (rightPWM>=0) and (leftPWM<0):
		ser.write('4')
	elif (rightPWM<0) and (leftPWM<0):
		ser.write('2')
	else:
		ser.write('5') #Should never occur

	#The C code on the bot accepts left motor velocity first.
	ser.write(struct.pack('>B',abs(leftPWM)))
	ser.write(struct.pack('>B',abs(rightPWM)))

def encoderOut():
	ser.write('A') 
	encoder = EncoderData()
	encoder.interval = 0
	encoder.encoderR = 0 
	encoder.encoderL = 0 
	encoder.chksum = 0
	enc = ser.read()
	if enc is 'A':
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
		encoder.stamp = rospy.get_time() 
		#rospy.loginfo(encoder)
		pub_encoder.publish(encoder)


if __name__ == '__main__':
	try:
		rospy.init_node('bot_interface',anonymous=True)
		rospy.Subscriber('pwm', PwmInput, callback)
		pub_encoder=rospy.Publisher('encoder',EncoderData,queue_size=10)

		rate = rospy.Rate(30) #Since bot sends data at 25hz the publisher will be forced to slow down
		while not rospy.is_shutdown():
			encoderOut()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
