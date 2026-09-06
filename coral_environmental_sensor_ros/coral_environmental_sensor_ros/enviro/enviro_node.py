import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.parameter_event_handler import ParameterEventHandler
from coral_environmental_msgs.msg import CoralEnviroMsg

from coral_environmental_sensor_ros.enviro.board import EnviroBoard
from coral_environmental_sensor_ros.air_quality.dfrobot_airqualitysensor import DFRobot_AirQualitySensor
from coral_environmental_sensor_ros.hcho_sensor.dfrobot_sfa40 import DFRobot_SFA40

from luma.core.render import canvas
import time
import threading

class EnviroNode(Node):

    def __init__(self):
        super().__init__('enviro_node')
        self.declare_parameter('enable_oled_display', False)
        self._enable_oled_display_param = self.get_parameter('enable_oled_display').get_parameter_value().bool_value
        self._param_handler = ParameterEventHandler(self)
        self._curr_node_name = self.get_name()
        self._curr_namespace = self.get_namespace()

        self._param_callback_handle = self._param_handler.add_parameter_callback(
            parameter_name='enable_oled_display',
            node_name=self._curr_node_name,
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

        self._sfa40_dev = DFRobot_SFA40(bus=1)
        self.current_sfa40_temperature = 0.0
        self.current_sfa40_humidity = 0.0
        self.current_sfa40_hcho = 0.0
        self._sfa40_dev_ready = False

        if self._sfa40_dev.begin() != 0:
            self.get_logger().error("Failed to initialize DFrobot SFA40 sensor")
            self._enabled_sfa40_sensor = False
        else:
            self.get_logger().info("DFrobot SFA40 sensor initialized successfully")
            self._enabled_sfa40_sensor = True

        if self._enabled_sfa40_sensor:
            sfa40_serial = self._sfa40_dev.get_serial_number()
            sfa40_serial_hex = ' '.join('{:02X}'.format(b) for b in sfa40_serial)
            self.get_logger().info(f"DFrobot SFA40 sensor serial number: {sfa40_serial_hex}")
            self._sfa40_dev.start_measurement()
            time.sleep(1)  # Wait for the sensor to stabilize
        
        self._air_quality_sensor_i2c_1 = 0x01
        self._air_quality_sensor_i2c_address = 0x19
        _data_topic_name = f"{self._curr_node_name}/data"
        self._sensor_pub = self.create_publisher(CoralEnviroMsg, _data_topic_name, 10)
        timer_period = 2.0 # 0.5hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.enviro = EnviroBoard()
        self.airqualitysensor = DFRobot_AirQualitySensor(self._air_quality_sensor_i2c_1, self._air_quality_sensor_i2c_address)

        try:
            airquality_sensor_version = self.airqualitysensor.gain_version()
            if airquality_sensor_version:
                self._enabled_air_quality_sensor = True
                self.get_logger().info(f"Air quality sensor version: {airquality_sensor_version}")
            else:
                self._enabled_air_quality_sensor = False
                self.get_logger().error("Failed to fetch air quality sensor version")

        except Exception as e:
            self.get_logger().error(f"Error occurred while fetching air quality sensor version: {e}")
            self._enabled_air_quality_sensor = False

        if self._enable_oled_display_param:
            self._oled_display_thread = threading.Thread(target=self._oled_display_loop, daemon=True)
            self._oled_display_thread.start()
            self.get_logger().info("OLED display enabled on startup")
        else:
            self.get_logger().info("OLED display disabled on startup")
            
        self.get_logger().info(f'EnviroNode initialized in namespace: {self._curr_namespace}, nodename: {self._curr_node_name}, publishing to topic: {_data_topic_name}')

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
                self.enviro.display.clear()
                self._stop_display_event.clear()
                self.get_logger().info("OLED display disabled")
            else:
                self.get_logger().info("OLED display already disabled")


    def _oled_display_loop(self):
        while not self._stop_display_event.is_set():
            # display PM2.5 standard concentration and humidity first
            with self._data_lock:
                msg = f"PM2.5 std: {self._curr_pm2_5_standard:.2f} ug/m3\n"
                msg += f"Humidity: {self._curr_humidity:.2f} %\n"
                aqi = self.calculate_pm2_5_aqi(self._curr_pm2_5_standard)
            self._oled_update_display(msg)
            self.get_logger().debug(f"Updated OLED display, msg: {msg}")
            time.sleep(5)
            ## display AQI and category if PM2.5 standard concentration is available
            if aqi is not None:
                # https://en.wikipedia.org/wiki/Air_quality_index
                if aqi >= 0 and aqi <= 50:
                    aqi_category = "Good"
                elif aqi >= 51 and aqi <= 100:
                    aqi_category = "Moderate"
                elif aqi >= 101 and aqi <= 150:
                    aqi_category = "Unhealthy warning"
                elif aqi >= 151 and aqi <= 200:
                    aqi_category = "Unhealthy"
                elif aqi >= 201 and aqi <= 300:
                    aqi_category = "Very unhealthy"
                elif aqi >= 301 and aqi <= 500:
                    aqi_category = "Hazardous"
                else:
                    aqi_category = "Unknown"
                aqi_msg = f"AQI: {aqi}\n"
                aqi_msg += f"{aqi_category}\n"
                self._oled_update_display(aqi_msg)
                self.get_logger().debug(f"Updated OLED display with AQI, msg: {aqi_msg}")
                time.sleep(5)

            ## display board id
            board_id_msg = f"{self._curr_node_name}\n"
            self._oled_update_display(board_id_msg)
            time.sleep(3)

    def _oled_update_display(self, msg: str):
        with canvas(self.enviro.display) as draw:
            draw.text((0, 0), msg, fill='white')


    def timer_callback(self):
        try:
            with self._data_lock:
                time_stamp_msg = self.get_clock().now().to_msg()
                msg = CoralEnviroMsg()

                # air quality data

                if self._enabled_air_quality_sensor:
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

                # Get HCHO data if the SFA40 sensor is enabled
                if self._enabled_sfa40_sensor:
                    sfa40_temperature, sfa40_humidity, sfa40_hcho, sfa40_ready = self._get_hcho_data()
                    if sfa40_hcho is not None:
                        msg.hcho_data.header.stamp = time_stamp_msg
                        msg.hcho_data.concentration = sfa40_hcho
                        msg.hcho_data.units = "ppb"
                        msg.hcho_data.temperature = sfa40_temperature
                        msg.hcho_data.humidity = sfa40_humidity
                        msg.hcho_data.sensor_ready = sfa40_ready
                        self.current_sfa40_temperature = sfa40_temperature
                        self.current_sfa40_humidity = sfa40_humidity
                        self.current_sfa40_hcho = sfa40_hcho

                self._sensor_pub.publish(msg)

                if self._enabled_air_quality_sensor:
                    self.get_logger().info('Publishing PM2.5: %.2f ug/m3' % self._none_to_nan(msg.air_quality.pm2_5_standard))

                if self._enabled_sfa40_sensor:
                    self.get_logger().info('Publishing HCHO: %.2f ppb' % self._none_to_nan(msg.hcho_data.concentration))

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


    def calculate_pm2_5_aqi(self, concentration):

        # Breakpoints for PM2.5 (US EPA Standard)
        breakpoints = [
            (0.0, 12.0, 0, 50),
            (12.1, 35.4, 51, 100),
            (35.5, 55.4, 101, 150),
            (55.5, 150.4, 151, 200),
            (150.5, 250.4, 201, 300),
            (250.5, 500.4, 301, 500)
        ]
        
        for c_low, c_high, i_low, i_high in breakpoints:
            if c_low <= concentration <= c_high:
                # Linear interpolation formula
                aqi = ((i_high - i_low) / (c_high - c_low)) * (concentration - c_low) + i_low
                return round(aqi)
        return None

    def _get_hcho_data(self):
        if self._enabled_sfa40_sensor:
            sensor_status = self._sfa40_dev.read_measurement_data_raw()
            if sensor_status == 0 and not self._sfa40_dev_ready:
                self.get_logger().info("The DFrobot SFA40 sensor is ready and the data is reliable!")
                self._sfa40_dev_ready = True
            elif sensor_status == 1:
                self.get_logger().warning("The DFrobot SFA40 sensor is not ready (<1 min, HCHO is 0 ppb)!")
            elif sensor_status == 2:
                self.get_logger().warning("The DFrobot SFA40 sensor is not up to specification (<10 min)!")
            elif sensor_status == 3:
                self.get_logger().error("The DFrobot SFA40 sensor read measurement data failed!")

            if sensor_status != 3:
                temperature = float(self._sfa40_dev.temperature_c)
                humidity = float(self._sfa40_dev.humidity)
                hcho = float(self._sfa40_dev.HCHO)
                self.get_logger().info(f"DFrobot SFA40 sensor Temperature: {temperature}°C, Humidity: {humidity}%, HCHO: {hcho} ppb")

                return temperature, humidity, hcho, self._sfa40_dev_ready
            else:
                self.get_logger().error("Failed to read data from DFrobot SFA40 sensor.")
                return None, None, None, False

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = EnviroNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()