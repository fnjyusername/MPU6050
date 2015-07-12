###  SET MPU DATA by MPU CLASS REFERENCE


###  void MPU6050::initialize  (  )  


Power on and prepare for general usage. This will activate the device and take it out of sleep mode (which must be done after start-up). This function also sets both the accelerometer and the gyroscope to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets the clock source to use the X Gyro for reference, which is slightly better than the default internal clock source. 
 


###  void MPU6050::reset  (  )  


Trigger a full device reset. A small delay of ~50ms may be desirable after triggering a reset. 
See also:MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_DEVICE_RESET_BIT  


void MPU6050::resetAccelerometerPath  (  )  


Reset accelerometer signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_ACCEL_RESET_BIT  


void MPU6050::resetFIFO  (  )  


Reset the FIFO. This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This bit automatically clears to 0 after the reset has been triggered. 
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_FIFO_RESET_BIT  


void MPU6050::resetGyroscopePath  (  )  


Reset gyroscope signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_GYRO_RESET_BIT  


void MPU6050::resetI2CMaster  (  )  


Reset the I2C Master. This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0. This bit automatically clears to 0 after the reset has been triggered. 
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_I2C_MST_RESET_BIT  


void MPU6050::resetSensors  (  )  


Reset all sensor registers and signal paths. When set to 1, this bit resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers. This bit automatically clears to 0 after the reset has been triggered.

When resetting only the signal path (and not the sensor registers), please use Register 104, SIGNAL_PATH_RESET.
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_SIG_COND_RESET_BIT  


void MPU6050::resetTemperaturePath  (  )  


Reset temperature sensor signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_TEMP_RESET_BIT  


void MPU6050::setAccelerometerPowerOnDelay  ( uint8_t  delay )  


Set accelerometer power-on delay. 
Parameters:
delay New accelerometer power-on delay (0-3)  
See also:getAccelerometerPowerOnDelay() MPU6050_RA_MOT_DETECT_CTRL MPU6050_DETECT_ACCEL_ON_DELAY_BIT  


void MPU6050::setAccelFIFOEnabled  ( bool  enabled )  


Set accelerometer FIFO enabled value. 
Parameters:
enabled New accelerometer FIFO enabled value  
See also:getAccelFIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setAccelXSelfTest  ( bool  enabled )  


Get self-test enabled setting for accelerometer X axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  


void MPU6050::setAccelYSelfTest  ( bool  enabled )  


Get self-test enabled value for accelerometer Y axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  


void MPU6050::setAccelZSelfTest  ( bool  enabled )  


Set self-test enabled value for accelerometer Z axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  


void MPU6050::setAuxVDDIOLevel  ( uint8_t  level )  


Set the auxiliary I2C supply voltage level. When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to the MPU-6000, which does not have a VLOGIC pin. 
Parameters:
level I2C supply voltage level (0=VLOGIC, 1=VDD)  
 


void MPU6050::setClockOutputEnabled  ( bool  enabled )  


Set reference clock output enabled status. When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For further information regarding CLKOUT, please refer to the MPU-60X0 Product Specification document. 
Parameters:
enabled New reference clock output enabled status  
See also:MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_CLKOUT_EN_BIT  


void MPU6050::setClockSource  ( uint8_t  source )  


Set clock source setting. An internal 8MHz oscillator, gyroscope based clock, or external sources can be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator or an external source is chosen as the clock source, the MPU-60X0 can operate in low power modes with the gyroscopes disabled.

Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference for improved stability. The clock source can be selected according to the following table:
 CLK_SEL | Clock Source
 --------+--------------------------------------
 0       | Internal oscillator
 1       | PLL with X Gyro reference
 2       | PLL with Y Gyro reference
 3       | PLL with Z Gyro reference
 4       | PLL with external 32.768kHz reference
 5       | PLL with external 19.2MHz reference
 6       | Reserved
 7       | Stops the clock and keeps the timing generator in reset
 
Parameters:
source New clock source setting  
See also:getClockSource() MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_CLKSEL_BIT MPU6050_PWR1_CLKSEL_LENGTH  


void MPU6050::setDeviceID  ( uint8_t  id )  


Set Device ID. Write a new ID into the WHO_AM_I register (no idea why this should ever be necessary though). 
Parameters:
id New device ID to set.  
See also:getDeviceID() MPU6050_RA_WHO_AM_I MPU6050_WHO_AM_I_BIT MPU6050_WHO_AM_I_LENGTH  


void MPU6050::setDHPFMode  ( uint8_t  bandwidth )  


Set the high-pass filter configuration. 
Parameters:
bandwidth New high-pass filter configuration  
See also:setDHPFMode() MPU6050_DHPF_RESET MPU6050_RA_ACCEL_CONFIG  


void MPU6050::setDLPFMode  ( uint8_t  mode )  


Set digital low-pass filter configuration. 
Parameters:
mode New DLFP configuration setting  
See also:getDLPFBandwidth() MPU6050_DLPF_BW_256 MPU6050_RA_CONFIG MPU6050_CFG_DLPF_CFG_BIT MPU6050_CFG_DLPF_CFG_LENGTH 
