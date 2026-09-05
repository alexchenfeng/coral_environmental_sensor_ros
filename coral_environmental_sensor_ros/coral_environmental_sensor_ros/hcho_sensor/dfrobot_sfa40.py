import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    from smbus import SMBus
    i2c_msg = None


class DFRobot_SFA40:
    SFA40_COMMAND_START = 0x00AC
    SFA40_COMMAND_READ = 0xC0EB
    SFA40_COMMAND_STOP = 0x50D2
    SFA40_COMMAND_ID = 0x02CE
    SFA40_COMMAND_READ_B4 = 0xE06D
    SFA40_COMMAND_ID_B4 = 0x0559
    SFA40_CRC_INITIAL_VALUE = 0xFF
    SFA40_CRC_POLYNOMIAL = 0x31
    SFA40_IIC_ADDRESS = 0x5D

    def __init__(self, bus=1, address=SFA40_IIC_ADDRESS):
        self.bus = SMBus(bus)
        self.address = address
        self._buf = [0] * 15
        self._read_command = self.SFA40_COMMAND_READ
        self._serial_command = self.SFA40_COMMAND_ID
        self._status_index = 9
        self._serial_read_size = 9
        self._serial_data_len = 6
        self.HCHO = float('nan')
        self.humidity = float('nan')
        self.temperature_c = float('nan')
        self.temperature_f = float('nan')

    def _write_command(self, command):
        data = bytes([(command >> 8) & 0xFF, command & 0xFF])
        if i2c_msg is not None:
            msg = i2c_msg.write(self.address, data)
            self.bus.i2c_rdwr(msg)
        else:
            self.bus.write_i2c_block_data(self.address, data[0], [data[1]])

    def _read_reg(self, command, length):
        if i2c_msg is None:
            raise RuntimeError("smbus2 is required for SFA40 I2C read operations")
        write = i2c_msg.write(self.address, bytes([(command >> 8) & 0xFF, command & 0xFF]))
        read = i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)

    def _calc_crc(self, data):
        crc = self.SFA40_CRC_INITIAL_VALUE
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ self.SFA40_CRC_POLYNOMIAL) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def _validate_serial_crc(self, data):
        for i in range(0, len(data), 3):
            if self._calc_crc(data[i:i + 2]) != data[i + 2]:
                return False
        return True

    def _detect_protocol(self):
        self.stop_measurement()
        time.sleep(0.03)
        try:
            data = self._read_reg(self.SFA40_COMMAND_ID, 9)
            if len(data) == 9 and self._validate_serial_crc(data):
                self._read_command = self.SFA40_COMMAND_READ
                self._serial_command = self.SFA40_COMMAND_ID
                self._status_index = 9
                self._serial_read_size = 9
                self._serial_data_len = 6
                return
        except Exception:
            pass
        try:
            data = self._read_reg(self.SFA40_COMMAND_ID_B4, 15)
            if len(data) == 15 and self._validate_serial_crc(data):
                self._read_command = self.SFA40_COMMAND_READ_B4
                self._serial_command = self.SFA40_COMMAND_ID_B4
                self._status_index = 10
                self._serial_read_size = 15
                self._serial_data_len = 10
                return
        except Exception:
            pass
        self._read_command = self.SFA40_COMMAND_READ
        self._serial_command = self.SFA40_COMMAND_ID
        self._status_index = 9
        self._serial_read_size = 9
        self._serial_data_len = 6

    def begin(self):
        '''
        @fn begin
        @brief Check I2C device and auto-detect B4 or production protocol.
        @return Whether the sensor device is found.
        @retval 0: Sensor device exists, -1: Sensor device does not exist.
        '''
        try:
            self._detect_protocol()
            return 0
        except Exception as e:
            print("Init error: {}".format(e))
            return -1

    def start_measurement(self):
        self._write_command(self.SFA40_COMMAND_START)

    def stop_measurement(self):
        self._write_command(self.SFA40_COMMAND_STOP)

    def read_measurement_data_raw(self):
        '''
        @fn read_measurement_data_raw
        @brief Read formaldehyde, humidity, temperature and sensor status.
        @return int(0-3)
        @retval 0: Sensor is ready and data is within specifications.
        @retval 1: Sensor not ready (<1 min after power-up, HCHO is 0 ppb).
        @retval 2: Sensor not yet within specifications (B4: <5 min, production: <10 min).
        @retval 3: I2C communication failed or CRC check failed.
        '''
        try:
            self._buf = self._read_reg(self._read_command, 12)
        except Exception:
            self.HCHO = float('nan')
            self.humidity = float('nan')
            self.temperature_c = float('nan')
            self.temperature_f = float('nan')
            return 3

        if len(self._buf) != 12:
            self.HCHO = float('nan')
            self.humidity = float('nan')
            self.temperature_c = float('nan')
            self.temperature_f = float('nan')
            return 3

        for i in range(0, 9, 3):
            if self._calc_crc(self._buf[i:i + 2]) != self._buf[i + 2]:
                self.HCHO = float('nan')
                self.humidity = float('nan')
                self.temperature_c = float('nan')
                self.temperature_f = float('nan')
                return 3

        if self._calc_crc(self._buf[9:11]) != self._buf[11]:
            self.HCHO = float('nan')
            self.humidity = float('nan')
            self.temperature_c = float('nan')
            self.temperature_f = float('nan')
            return 3

        self.HCHO = (self._buf[0] * 256 + self._buf[1]) / 10.0
        self.humidity = 125.0 * (self._buf[3] * 256 + self._buf[4]) / 65535.0 - 6
        if self.humidity < 0:
            self.humidity = 0
        if self.humidity > 100:
            self.humidity = 100
        self.temperature_c = 175.0 * (self._buf[6] * 256 + self._buf[7]) / 65535.0 - 45
        self.temperature_f = 315.0 * (self._buf[6] * 256 + self._buf[7]) / 65535.0 - 49

        if self._buf[self._status_index] & 0x01:
            return 1
        if self._buf[self._status_index] & 0x02:
            return 2
        return 0

    def get_serial_number(self):
        '''
        @fn get_serial_number
        @brief Read unique serial number (6 bytes for production, 10 bytes for B4).
        @return Serial number bytes, or None on I2C/CRC error.
        '''
        try:
            data = self._read_reg(self._serial_command, self._serial_read_size)
        except Exception:
            return None

        if len(data) != self._serial_read_size or not self._validate_serial_crc(data):
            return None

        serial_number = []
        for i in range(0, self._serial_read_size, 3):
            serial_number.append(data[i])
            serial_number.append(data[i + 1])
        return bytes(serial_number)
