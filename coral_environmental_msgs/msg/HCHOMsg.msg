# formaldehyde sensor msg for DFRobot_SFA40
#
# Standard timestamp and frame information
#
std_msgs/Header header

float64 concentration	# Formaldehyde gas concentration (typically measured in ppm or ppb)
string units		# "ppm" or "ppb"

float64 variance 	# Measurement variance (0 if unknown)
float64 temperature     # In Degrees Celsius
float64 humidity	# Relative humidity percentage (0.0 to 100.0)

bool sensor_ready	# True if the sensor is ready and providing valid data, False otherwise
