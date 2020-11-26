
Introduction
============

.. image:: 
	API from STMicroelectronics, VL53L0X_Spec_Api.pdf 

Micropython driver for the VL53L0X distance sensor and MPU9250  Gyro, Acc, and Mag.

Dependencies
=============
This driver depends on:

* Chimobb <https://github.com/chimobb/Micropython>



Installing
==========

Copy from github to your project, using Micropython >= 1.13

Usage Example
=============

import vl53l0x
import machine
i2c1=machine.I2C(1)
vl1=vl53l0x.VL53L0X(i2c1)
def Midea(cuantos=10):
	n1=0
	for i1 in range(cuantos):
		vl1.StartRange()
		result = vl1.CheckResultRange()
		while result == None:
			time.sleep_ms(1)
			n1+=1
			result = vl1.CheckResultRange()
		print(n1, result) #n1 = 33
		n1=0
		time.sleep(1)

For MPU9250:
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


Contributing
============

Contributions are welcome! 

Documentation
=============

Directly from Micropython Documentation, STMicroelectronics and Invensense
