# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from coral_environmental_sensor_ros.enviro.board import EnviroBoard
from coral_environmental_sensor_ros.air_quality.dfrobot_airqualitysensor import DFRobot_AirQualitySensor

from luma.core.render import canvas
from PIL import ImageDraw
from time import sleep

import argparse
import itertools

def update_display(display, msg):
    with canvas(display) as draw:
        draw.text((0, 0), msg, fill='white')


def _none_to_nan(val):
    return float('nan') if val is None else val


def air_quality_pm2_5():

    I2C_1       = 0x01             
    I2C_ADDRESS = 0x19
    with DFRobot_AirQualitySensor(I2C_1 ,I2C_ADDRESS) as airqualitysensor:
        concentration = airqualitysensor.gain_particle_concentration_ugm3(airqualitysensor.PARTICLE_PM2_5_STANDARD)
        return concentration




def main():

    # Pull arguments from command line.

    parser = argparse.ArgumentParser(description='Enviro Kit Demo')

    parser.add_argument('--display_duration',
                        help='Measurement display duration (seconds)', type=int,
                        default=5)
    
    args = parser.parse_args()

    # Create instances of EnviroKit and Cloud IoT.
    enviro = EnviroBoard()
    sensors = {}

    for read_count in itertools.count():
        # First display temperature and RH.
        sensors['temperature'] = enviro.temperature
        sensors['humidity'] = enviro.humidity
        msg = 'Temp: %.2f C\n' % _none_to_nan(sensors['temperature'])
        msg += 'RH: %.2f %%' % _none_to_nan(sensors['humidity'])
        update_display(enviro.display, msg)
        sleep(args.display_duration)

        # After 5 seconds, switch to light and pressure.
        sensors['ambient_light'] = enviro.ambient_light
        sensors['pressure'] = enviro.pressure
        msg = 'Light: %.2f lux\n' % _none_to_nan(sensors['ambient_light'])
        msg += 'Pressure: %.2f kPa' % _none_to_nan(sensors['pressure'])
        update_display(enviro.display, msg)
        sleep(args.display_duration)

        # After 10 seconds, switch to PM2.5.
        sensors['pm2_5'] = air_quality_pm2_5()
        msg = 'PM2.5: %.2f ug/m3' % _none_to_nan(sensors['pm2_5'])
        update_display(enviro.display, msg)
        sleep(args.display_duration)


if __name__ == '__main__':
    main()
