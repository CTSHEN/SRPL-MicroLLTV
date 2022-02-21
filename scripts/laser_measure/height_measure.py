import rospy
from std_msgs.msg import Float32
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

bus = SMBUS(i2c_channel)

def get_distance():
	pub = rospy.Publisher('height', Float32, queue_size=1)
	rospy.init_node('laser_measure', anonymous=True)
	rate = rospy.Rate(20)  # Publish distance at 20Hz
	# initialize measurement
	measure = np.float16(0.0)

	while not rospy.is_shutdown():
		# Start Garmin Lidar Lite v4 measure procedure
		# 1. Write 0x04 to reg 0x00
		bus.write_i2c_block_data(address, reg_cmd, 0x04)
		# 2. Read reg 0x01 (Repeat until LSB goes low)
		while (bus.read_i2c_block_data(address, reg_stat, 1) & 0x01):
			pass
		result = bus.read_i2c_block_data(address,reg_meas_low, 2)
		measure = ((result[1] & 0xFF) << 8) | (result[0] & 0xFF)
		#convert measure data from float16 to float32
		measure32 = np.float32(measure)
		pub.publish(measure32)  # publish data
		rate.sleep()

if __name__ = '__main__':
	try:
		get_distance()
	except rospy.ROSInterruptException:
		pass

