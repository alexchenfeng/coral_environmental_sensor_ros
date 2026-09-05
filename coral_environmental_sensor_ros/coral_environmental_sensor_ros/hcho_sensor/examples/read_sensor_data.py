import time
import sys
from DFRobot_SFA40 import DFRobot_SFA40

sfa40 = DFRobot_SFA40(bus=1)

def setup():
    print("SFA40 init...")
    if sfa40.begin() != 0:
        print("failed, Not found SFA40!")
        sys.exit(1)
    print("successed")
    serial = sfa40.get_serial_number()
    if serial:
        serial_hex = ' '.join('{:02X}'.format(b) for b in serial)
        print("SerialNumber: {}.".format(serial_hex))
    else:
        print("Failed to get serial number.")
    sfa40.start_measurement()
    time.sleep(1)

def loop():
    while True:
        status = sfa40.read_measurement_data_raw()
        if status == 0:
            print("The sensor is ready and the data is reliable!")
        elif status == 1:
            print("The sensor is not ready (<1 min, HCHO is 0 ppb)!")
        elif status == 2:
            print("Sensor is not up to specification (<10 min)!")
        elif status == 3:
            print("Read measurement data failed!")

        if status != 3:
            print("Temperature: {:.2f} C, {:.2f} F, humidity: {:.2f} %RH, HCHO: {:.2f} ppb".format(
                sfa40.temperature_c, sfa40.temperature_f, sfa40.humidity, sfa40.HCHO))
        time.sleep(1)

setup()
loop()
