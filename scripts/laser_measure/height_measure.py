#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from smbus2 import SMBus
import time
import numpy as np

#RPi i2c channel 1
i2c_channel = 1
#Garmin lidar lite address
address = 0x62
reg_cmd = 0x00
reg_stat = 0x01
reg_meas_low = 0x10
reg_meas_high = 0x11
reg_accuracy = 0xEB

bus = SMBus(i2c_channel)

def get_distance():
	pub = rospy.Publisher('height_measure', PointStamped , queue_size=1)
	rospy.init_node('laser_measure', anonymous=True)
	rate = rospy.Rate(20)  # Publish distance at 20Hz
	# Lidar adjust accuracy & speed
	bus.write_byte_data(address, reg_accuracy, 0x0B)
	# initialize measurement3
	measure = np.float16(0.0)
	height_data = PointStamped()

	while not rospy.is_shutdown():
		# Start Garmin Lidar Lite v4 measure procedure
		# 1. Write 0x04 to reg 0x00
		bus.write_byte_data(address, reg_cmd, 0x04)
		# 2. Read reg 0x01 (Repeat until LSB goes low)
		while (bus.read_byte_data(address, reg_stat, 1) & 0x01):
			pass
		result = bus.read_i2c_block_data(address,reg_meas_low, 2)
		measure = ((result[1] & 0xFF) << 8) | (result[0] & 0xFF)
		#convert measure data from float16 to float32
		measure64 = np.float64(measure)/100  # in meters
		print(measure64)

		height_data.header.stamp = rospy.get_rostime()
		height_data.point.z = measure64
		pub.publish(height_data)  # publish data
		rate.sleep()

if __name__ == '__main__':
	try:
		get_distance()
	except rospy.ROSInterruptException:
		pass

