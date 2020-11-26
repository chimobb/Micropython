# Copyright (c) 2020 Joaquin Berenguer, based on Driver by Mika Tuupola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of  this software and associated documentation files (the "Software"), to
# deal in  the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copied of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# https://github.com/tuupola/micropython-mpu9250

"""
MicroPython I2C driver for MPU9250 9-axis motion tracking device
Usage:
import machine
import Mpu9250
i2c1=machine.I2C(1)
mpu=Mpu9250.MPU9250(i2c1)
mpu.temperature
mpu.gyro
mpu.acceleration
mpu.magnetic

To calibrate first time, and change parameter:
mpu.calibrate()
mpu.aka8963.calibrate()
"""

import ustruct
import utime
from machine import I2C, Pin
from micropython import const


__version__ = "0.1.0"

# Used for enabling and disabling the I2C bypass access
_INT_PIN_CFG = const(0x37)
_I2C_BYPASS_MASK = const(0b00000010)
_I2C_BYPASS_EN = const(0b00000010)
_I2C_BYPASS_DIS = const(0b00000000)
_GYRO_CONFIG = const(0x1b)
_ACCEL_CONFIG = const(0x1c)
_ACCEL_CONFIG2 = const(0x1d)
_ACCEL_XOUT_H = const(0x3b)
_ACCEL_XOUT_L = const(0x3c)
_ACCEL_YOUT_H = const(0x3d)
_ACCEL_YOUT_L = const(0x3e)
_ACCEL_ZOUT_H = const(0x3f)
_ACCEL_ZOUT_L = const(0x40)
_TEMP_OUT_H = const(0x41)
_TEMP_OUT_L = const(0x42)
_GYRO_XOUT_H = const(0x43)
_GYRO_XOUT_L = const(0x44)
_GYRO_YOUT_H = const(0x45)
_GYRO_YOUT_L = const(0x46)
_GYRO_ZOUT_H = const(0x47)
_GYRO_ZOUT_L = const(0x48)
_WHO_AM_I = const(0x75)

#_ACCEL_FS_MASK = const(0b00011000)
ACCEL_FS_SEL_2G = const(0b00000000)
ACCEL_FS_SEL_4G = const(0b00001000)
ACCEL_FS_SEL_8G = const(0b00010000)
ACCEL_FS_SEL_16G = const(0b00011000)

_ACCEL_SO_2G = 16384  # 1 / 16384 ie. 0.061 mg / digit
_ACCEL_SO_4G = 8192  # 1 / 8192 ie. 0.122 mg / digit
_ACCEL_SO_8G = 4096  # 1 / 4096 ie. 0.244 mg / digit
_ACCEL_SO_16G = 2048  # 1 / 2048 ie. 0.488 mg / digit

#_GYRO_FS_MASK = const(0b00011000)
GYRO_FS_SEL_250DPS = const(0b00000000)
GYRO_FS_SEL_500DPS = const(0b00001000)
GYRO_FS_SEL_1000DPS = const(0b00010000)
GYRO_FS_SEL_2000DPS = const(0b00011000)

_GYRO_SO_250DPS = 131
_GYRO_SO_500DPS = 62.5
_GYRO_SO_1000DPS = 32.8
_GYRO_SO_2000DPS = 16.4

_TEMP_SO = 333.87
_TEMP_OFFSET = 21

SF_G = 1
SF_M_S2 = 9.80665  # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 0.017453292519943  # 1 deg/s is 0.017453292519943 rad/s

# AKA8963
_WIA = const(0x00)
_HXL = const(0x03)
_HXH = const(0x04)
_HYL = const(0x05)
_HYH = const(0x06)
_HZL = const(0x07)
_HZH = const(0x08)
_ST2 = const(0x09)
_CNTL1 = const(0x0a)
_ASAX = const(0x10)
_ASAY = const(0x11)
_ASAZ = const(0x12)

_MODE_POWER_DOWN = 0b00000000
MODE_SINGLE_MEASURE = 0b00000001
MODE_CONTINOUS_MEASURE_1 = 0b00000010  # 8Hz
MODE_CONTINOUS_MEASURE_2 = 0b00000110  # 100Hz
MODE_EXTERNAL_TRIGGER_MEASURE = 0b00000100
_MODE_SELF_TEST = 0b00001000
_MODE_FUSE_ROM_ACCESS = 0b00001111

OUTPUT_14_BIT = 0b00000000
OUTPUT_16_BIT = 0b00010000

_SO_14BIT = 0.6  # μT per digit when 14bit mode
_SO_16BIT = 0.15  # μT per digit when 16bit mode

#use calibrate for your chip and pass that value as parameter
#use calibrate when change the chip location
class MPU9250:
	"""Class which provides interface to MPU9250 9-axis motion tracking device."""

	def __init__(self, i2c, address=0x68,
              accel_fs=ACCEL_FS_SEL_2G, gyro_fs=GYRO_FS_SEL_250DPS,
              accel_sf=SF_M_S2, gyro_sf=SF_RAD_S, 
              gyro_offset=(0.001747098729408655, -0.02196754166468256, 0.006851520635288933)):
		self.i2c = i2c
		self.address = address
		self.buf1 = bytearray(1)
		self.buf6 = bytearray(6)
		self.buf2 = bytearray(2)
		if self.whoami not in [0x73]:
			raise RuntimeError("MPU9250 not found in I2C bus.")
		self._accel_so = self._accel_fs(accel_fs)
		self._gyro_so = self._gyro_fs(gyro_fs)
		self._accel_sf = accel_sf
		self._gyro_sf = gyro_sf
		self._gyro_offset = gyro_offset
		# Enable I2C bypass to access AK8963 directly.
		char = self._register_char(_INT_PIN_CFG)
		char &= ~_I2C_BYPASS_MASK  # clear I2C bits
		char |= _I2C_BYPASS_EN
		self._register_char(_INT_PIN_CFG, char)
		self.ak8963 = AK8963(i2c)

	@property
	def magnetic(self):
		"""
		X, Y, Z axis micro-Tesla (uT) as floats.
		"""
		return self.ak8963.magnetic

	@property
	def acceleration(self):
		"""
		Acceleration measured by the sensor. By default will return a
		3-tuple of X, Y, Z axis acceleration values in m/s^2 as floats. Will
		return values in g if constructor was provided `accel_sf=SF_M_S2`
		parameter.
		"""
		so = self._accel_so
		sf = self._accel_sf

		xyz = self._register_three_shorts(_ACCEL_XOUT_H)
		return tuple([value / so * sf for value in xyz])

	@property
	def gyro(self):
		"""
		X, Y, Z radians per second as floats.
		"""
		so = self._gyro_so
		sf = self._gyro_sf
		ox, oy, oz = self._gyro_offset

		xyz = self._register_three_shorts(_GYRO_XOUT_H)
		xyz = [value / so * sf for value in xyz]

		xyz[0] -= ox
		xyz[1] -= oy
		xyz[2] -= oz

		return tuple(xyz)

	@property
	def temperature(self):
		"""
		Die temperature in celcius as a float.
		"""
		temp = self._register_short(_TEMP_OUT_H)
		return ((temp - _TEMP_OFFSET) / _TEMP_SO) + _TEMP_OFFSET

	@property
	def whoami(self):
		""" Value of the whoami register. """
		return self._register_char(_WHO_AM_I)

	def calibrate(self, count=256, delay=100):
		ox, oy, oz = (0.0, 0.0, 0.0)
		self._gyro_offset = (0.0, 0.0, 0.0)
		n = float(count)

		while count:
			utime.sleep_ms(delay)
			gx, gy, gz = self.gyro
			ox += gx
			oy += gy
			oz += gz
			count -= 1

		self._gyro_offset = (ox / n, oy / n, oz / n)
		return self._gyro_offset

	def _register_short(self, register, value=None):
		if value is None:
			self.i2c.readfrom_mem_into(self.address, register, self.buf2)
			return ustruct.unpack(">h", self.buf2)[0]

		ustruct.pack_into(">h", self.buf2, 0, value)
		return self.i2c.writeto_mem(self.address, register, self.buf2)

	def _register_three_shorts(self, register):
		self.i2c.readfrom_mem_into(self.address, register, self.buf6)
		return ustruct.unpack(">hhh", self.buf6)

	def _register_char(self, register, value=None):
		if value is None:
			self.i2c.readfrom_mem_into(self.address, register, self.buf1)
			return self.buf1[0]

		ustruct.pack_into("<b", self.buf1, 0, value)
		return self.i2c.writeto_mem(self.address, register, self.buf1)

	def _accel_fs(self, value):
		self._register_char(_ACCEL_CONFIG, value)

		# Return the sensitivity divider
		if ACCEL_FS_SEL_2G == value:
			return _ACCEL_SO_2G
		elif ACCEL_FS_SEL_4G == value:
			return _ACCEL_SO_4G
		elif ACCEL_FS_SEL_8G == value:
			return _ACCEL_SO_8G
		elif ACCEL_FS_SEL_16G == value:
			return _ACCEL_SO_16G

	def _gyro_fs(self, value):
		self._register_char(_GYRO_CONFIG, value)

		# Return the sensitivity divider
		if GYRO_FS_SEL_250DPS == value:
			return _GYRO_SO_250DPS
		elif GYRO_FS_SEL_500DPS == value:
			return _GYRO_SO_500DPS
		elif GYRO_FS_SEL_1000DPS == value:
			return _GYRO_SO_1000DPS
		elif GYRO_FS_SEL_2000DPS == value:
			return _GYRO_SO_2000DPS

	def __enter__(self):
		return self

	def __exit__(self, exception_type, exception_value, traceback):
		pass

#use calibrate for your chip, and pass as constant from that moment.
#(12.6521484375, -23.86875, 12.2765625), (0.9323351569248489, 1.065131578947368, 1.011558887847548)
class AK8963:
	"""Class which provides interface to AK8963 magnetometer."""

	def __init__(
		self, i2c, address=0x0c,
		mode=MODE_CONTINOUS_MEASURE_1, output=OUTPUT_16_BIT,
		offset=(12.6521484375, -23.86875, 12.2765625), 
            scale=(0.9323351569248489, 1.065131578947368, 1.011558887847548)):
		self.i2c = i2c
		self.address = address
		self._offset = offset
		self._scale = scale

		if 0x48 != self.whoami:
			raise RuntimeError("AK8963 not found in I2C bus.")

		# Sensitivity adjustement values
		self._register_char(_CNTL1, _MODE_FUSE_ROM_ACCESS)
		asax = self._register_char(_ASAX)
		asay = self._register_char(_ASAY)
		asaz = self._register_char(_ASAZ)
		self._register_char(_CNTL1, _MODE_POWER_DOWN)

		# Should wait atleast 100us before next mode
		self._adjustement = (
			(0.5 * (asax - 128)) / 128 + 1,
			(0.5 * (asay - 128)) / 128 + 1,
			(0.5 * (asaz - 128)) / 128 + 1
		)

		# Power on
		self._register_char(_CNTL1, (mode | output))

		if output is OUTPUT_16_BIT:
			self._so = _SO_16BIT
		else:
			self._so = _SO_14BIT

	@property
	def magnetic(self):
		"""
		X, Y, Z axis micro-Tesla (uT) as floats.
		"""
		xyz = list(self._register_three_shorts(_HXL))
		self._register_char(_ST2)  # Enable updating readings again

		# Apply factory axial sensitivy adjustements
		xyz[0] *= self._adjustement[0]
		xyz[1] *= self._adjustement[1]
		xyz[2] *= self._adjustement[2]

		# Apply output scale determined in constructor
		so = self._so
		xyz[0] *= so
		xyz[1] *= so
		xyz[2] *= so

		# Apply hard iron ie. offset bias from calibration
		xyz[0] -= self._offset[0]
		xyz[1] -= self._offset[1]
		xyz[2] -= self._offset[2]

		# Apply soft iron ie. scale bias from calibration
		xyz[0] *= self._scale[0]
		xyz[1] *= self._scale[1]
		xyz[2] *= self._scale[2]

		return tuple(xyz)

	@property
	def adjustement(self):
		return self._adjustement

	@property
	def whoami(self):
		""" Value of the whoami register. """
		return self._register_char(_WIA)

	def calibrate(self, count=256, delay=200):
		self._offset = (0, 0, 0)
		self._scale = (1, 1, 1)

		reading = self.magnetic
		minx = maxx = reading[0]
		miny = maxy = reading[1]
		minz = maxz = reading[2]

		while count:
			utime.sleep_ms(delay)
			reading = self.magnetic
			minx = min(minx, reading[0])
			maxx = max(maxx, reading[0])
			miny = min(miny, reading[1])
			maxy = max(maxy, reading[1])
			minz = min(minz, reading[2])
			maxz = max(maxz, reading[2])
			count -= 1

		# Hard iron correction
		offset_x = (maxx + minx) / 2
		offset_y = (maxy + miny) / 2
		offset_z = (maxz + minz) / 2

		self._offset = (offset_x, offset_y, offset_z)

		# Soft iron correction
		avg_delta_x = (maxx - minx) / 2
		avg_delta_y = (maxy - miny) / 2
		avg_delta_z = (maxz - minz) / 2

		avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

		scale_x = avg_delta / avg_delta_x
		scale_y = avg_delta / avg_delta_y
		scale_z = avg_delta / avg_delta_z

		self._scale = (scale_x, scale_y, scale_z)

		return self._offset, self._scale

	def _register_short(self, register, value=None, buf=bytearray(2)):
		if value is None:
			self.i2c.readfrom_mem_into(self.address, register, buf)
			return ustruct.unpack("<h", buf)[0]

		ustruct.pack_into("<h", buf, 0, value)
		return self.i2c.writeto_mem(self.address, register, buf)

	def _register_three_shorts(self, register, buf=bytearray(6)):
		self.i2c.readfrom_mem_into(self.address, register, buf)
		return ustruct.unpack("<hhh", buf)

	def _register_char(self, register, value=None, buf=bytearray(1)):
		if value is None:
			self.i2c.readfrom_mem_into(self.address, register, buf)
			return buf[0]

		ustruct.pack_into("<b", buf, 0, value)
		return self.i2c.writeto_mem(self.address, register, buf)

	def __enter__(self):
		return self

	def __exit__(self, exception_type, exception_value, traceback):
		pass


