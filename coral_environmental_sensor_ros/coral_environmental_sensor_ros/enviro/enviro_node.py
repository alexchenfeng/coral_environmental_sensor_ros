import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from coral_environmental_msgs.msg import CoralEnviroMsg

from coral_environmental_sensor_ros.enviro.board import EnviroBoard
from coral_environmental_sensor_ros.air_quality.dfrobot_airqualitysensor import DFRobot_AirQualitySensor

from luma.core.render import canvas
import itertools
import time

class EnviroNode(Node):

    def __init__(self):
        super().__init__('enviro_node')
        self._air_quality_sensor_i2c_1 = 0x01
        self._air_quality_sensor_i2c_address = 0x19
        self._sensor_pub = self.create_publisher(CoralEnviroMsg, 'coral_enviro_data', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.enviro = EnviroBoard()
        self.airqualitysensor = DFRobot_AirQualitySensor(self._air_quality_sensor_i2c_1, self._air_quality_sensor_i2c_address)
        self.get_logger().info('EnviroNode initialized')

    def timer_callback(self):
        try:
            time_stamp_msg = self.get_clock().now().to_msg()
            msg = CoralEnviroMsg()
            
            msg.air_quality.header.stamp = time_stamp_msg
            msg.air_quality.pm2_5_standard = self._get_pm2_5_std()
            msg.air_quality.pm2_5_atmosphere = self._get_pm2_5_atmosphere()
            msg.air_quality.pm1_0_standard = self._get_pm1_0_std()
            msg.air_quality.pm1_0_atmosphere = self._get_pm1_0_atmosphere()
            msg.air_quality.pm10_standard = self._get_pm10_0_std()
            msg.air_quality.pm10_atmosphere = self._get_pm10_0_atmosphere()

            msg.ambient_light.header.stamp = time_stamp_msg
            msg.ambient_light.illuminance = float(self.enviro.ambient_light)

            msg.humidity.header.stamp = time_stamp_msg
            msg.humidity.relative_humidity = float(self.enviro.humidity)

            msg.air_pressure.header.stamp = time_stamp_msg
            msg.air_pressure.fluid_pressure = float(self.enviro.pressure)

            msg.temperature.header.stamp = time_stamp_msg
            msg.temperature.temperature = float(self.enviro.temperature)

            self._sensor_pub.publish(msg)
            self.get_logger().info('Publishing PM2.5: %.2f ug/m3' % self._none_to_nan(msg.air_quality.pm2_5_standard))
        except Exception as e:
            self.get_logger().error('Error reading sensors: %s' % str(e))
            time.sleep(5)  # Wait before retrying to avoid spamming logs

    def _get_pm2_5_std(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM2_5_STANDARD)
        return float(concentration)
    
    def _get_pm2_5_atmosphere(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM2_5_ATMOSPHERE)
        return float(concentration)

    def _get_pm1_0_std(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM1_0_STANDARD)
        return float(concentration)
    
    def _get_pm1_0_atmosphere(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM1_0_ATMOSPHERE)
        return float(concentration)

    def _get_pm10_0_std(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM10_STANDARD)
        return float(concentration)

    def _get_pm10_0_atmosphere(self):
        concentration = self.airqualitysensor.gain_particle_concentration_ugm3(self.airqualitysensor.PARTICLE_PM10_ATMOSPHERE)
        return float(concentration)

    def _none_to_nan(self, val):
        return float('nan') if val is None else val


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = EnviroNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()