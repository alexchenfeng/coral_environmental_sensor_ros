import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.parameter_event_handler import ParameterEventHandler
from coral_environmental_msgs.msg import CoralEnviroMsg

from coral_environmental_sensor_ros.enviro.board import EnviroBoard
from coral_environmental_sensor_ros.air_quality.dfrobot_airqualitysensor import DFRobot_AirQualitySensor

from luma.core.render import canvas
import itertools
import time
import threading

class EnviroNode(Node):

    def __init__(self):
        super().__init__('enviro_node')
        self.declare_parameter('enable_oled_display', True)
        self._param_handler = ParameterEventHandler(self)

        self._param_callback_handle = self._param_handler.add_parameter_callback(
            parameter_name='enable_oled_display',
            node_name="enviro_node",
            callback=self._enable_oled_display_callback
        )

        self._data_lock = threading.Lock()
        self._stop_display_event = threading.Event()
        self._oled_display_thread = None
        self._curr_pm2_5_standard = 0.0
        self._curr_pm2_5_atmosphere = 0.0
        self._curr_pm1_0_standard = 0.0
        self._curr_pm1_0_atmosphere = 0.0
        self._curr_pm10_standard = 0.0
        self._curr_pm10_atmosphere = 0.0
        self._curr_ambient_light = 0.0
        self._curr_humidity = 0.0
        self._curr_pressure = 0.0
        self._curr_temperature = 0.0
        
        self._air_quality_sensor_i2c_1 = 0x01
        self._air_quality_sensor_i2c_address = 0x19
        self._sensor_pub = self.create_publisher(CoralEnviroMsg, 'coral_enviro_data', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.enviro = EnviroBoard()
        self.airqualitysensor = DFRobot_AirQualitySensor(self._air_quality_sensor_i2c_1, self._air_quality_sensor_i2c_address)
        self.get_logger().info('EnviroNode initialized')

    def _enable_oled_display_callback(self, param: rclpy.parameter.Parameter):
        self.get_logger().info(f"Received an update to parameter: {param.name}: {rclpy.parameter.parameter_value_to_python(param.value)}")
        p_val = rclpy.parameter.parameter_value_to_python(param.value)

        if p_val:
            if self._oled_display_thread is not None:
                self.get_logger().info("OLED display already enabled")
            else:
                self._oled_display_thread = threading.Thread(target=self._oled_display_loop, daemon=True)
                self._oled_display_thread.start()
                self.get_logger().info("OLED display enabled")
        else:
            if self._oled_display_thread is not None:
                self._stop_display_event.set()
                self._oled_display_thread.join()
                self._oled_display_thread = None
                self._oled_update_display("")
                self.get_logger().info("OLED display disabled")
            else:
                self.get_logger().info("OLED display already disabled")


    def _oled_display_loop(self):
        while not self._stop_display_event.is_set():
            with self._data_lock:
                msg = f"PM2.5 std: {self._curr_pm2_5_standard:.2f} ug/m3\n"
                msg += f"Humidity: {self._curr_humidity:.2f} %\n"
            self._oled_update_display(msg)
            self.get_logger().debug(f"Updated OLED display, msg: {msg}")
            time.sleep(5)

    def _oled_update_display(self, msg: str):
        with canvas(self.enviro.display) as draw:
            draw.text((0, 0), msg, fill='white')


    def timer_callback(self):
        try:
            with self._data_lock:
                time_stamp_msg = self.get_clock().now().to_msg()
                msg = CoralEnviroMsg()

                msg.air_quality.header.stamp = time_stamp_msg

                msg.air_quality.pm2_5_standard = self._get_pm2_5_std()
                self._curr_pm2_5_standard = msg.air_quality.pm2_5_standard

                msg.air_quality.pm2_5_atmosphere = self._get_pm2_5_atmosphere()
                self._curr_pm2_5_atmosphere = msg.air_quality.pm2_5_atmosphere

                msg.air_quality.pm1_0_standard = self._get_pm1_0_std()
                self._curr_pm1_0_standard = msg.air_quality.pm1_0_standard

                msg.air_quality.pm1_0_atmosphere = self._get_pm1_0_atmosphere()
                self._curr_pm1_0_atmosphere = msg.air_quality.pm1_0_atmosphere

                msg.air_quality.pm10_standard = self._get_pm10_0_std()
                self._curr_pm10_standard = msg.air_quality.pm10_standard

                msg.air_quality.pm10_atmosphere = self._get_pm10_0_atmosphere()
                self._curr_pm10_atmosphere = msg.air_quality.pm10_atmosphere

                msg.ambient_light.header.stamp = time_stamp_msg
                msg.ambient_light.illuminance = float(self.enviro.ambient_light)
                self._curr_ambient_light = msg.ambient_light.illuminance

                msg.humidity.header.stamp = time_stamp_msg
                msg.humidity.relative_humidity = float(self.enviro.humidity)
                self._curr_humidity = msg.humidity.relative_humidity

                msg.air_pressure.header.stamp = time_stamp_msg
                msg.air_pressure.fluid_pressure = float(self.enviro.pressure)
                self._curr_pressure = msg.air_pressure.fluid_pressure

                msg.temperature.header.stamp = time_stamp_msg
                msg.temperature.temperature = float(self.enviro.temperature)
                self._curr_temperature = msg.temperature.temperature

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