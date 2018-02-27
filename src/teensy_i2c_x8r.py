#!/usr/bin/env python

from __future__ import division
import logging
import time
import math
import numpy as np
import rospy
import std_msgs.msg
from RC_test.msg import rcData


TEENSY_ADDRESS    = 0x50
NUM_CHANNELS = 16

class Teensy(object):
	
	def __init__(self, address=TEENSY_ADDRESS, i2c=None, **kwargs):
		if i2c is None:
			import Adafruit_GPIO.I2C as I2C
			i2c = I2C
		self._device = i2c.get_i2c_device(address, **kwargs)
	
	def read_x8r(self):
		#~ print("receiving x8r data...")
		raw_i2c_byte = self._device.readList(0x00, 32)
		#~ print raw_i2c_data
		#~ print raw_i2c_data[0]
		#~ print type(bytes(raw_i2c_data[0])+bytes(raw_i2c_data[1]))
		rc_data = []
		for i in range(NUM_CHANNELS):
			rc_data.append(np.uint8(raw_i2c_byte[2*i])*256 + np.uint8(raw_i2c_byte[2*i+1]))
		return rc_data
		


if __name__ == "__main__":
	print "initializing teensy 3.6..."
	teensy = Teensy(address=TEENSY_ADDRESS, busnum=0)
	pub = rospy.Publisher('/rc_inputs', rcData, queue_size=100)
	rospy.init_node('rc_input_pub', anonymous = True)
	r = rospy.Rate(10)
	msg = rcData()
	print "publishing x8r data..."
	while not rospy.is_shutdown():
		msg.header.stamp = rospy.Time.now()
		msg.channels = teensy.read_x8r()
		pub.publish(msg)
		r.sleep()
		#~ time.sleep(0.1)
