###  SET MPU DATA by MPU CLASS REFERENCE


#####  void MPU6050::initialize  (  )  
`
Power on and prepare for general usage. This will activate the device and take it out of sleep mode (which must be done after start-up). This function also sets both the accelerometer and the gyroscope to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets the clock source to use the X Gyro for reference, which is slightly better than the default internal clock source. 
` 


#####  void MPU6050::reset  (  )  
`
Trigger a full device reset. A small delay of ~50ms may be desirable after triggering a reset. 
See also:MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_DEVICE_RESET_BIT  
`

#####  void MPU6050::resetAccelerometerPath  (  )  

`
Reset accelerometer signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_ACCEL_RESET_BIT  
`

#####  void MPU6050::resetFIFO  (  )  
`
Reset the FIFO. This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This bit automatically clears to 0 after the reset has been triggered. 
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_FIFO_RESET_BIT  
`

#####  void MPU6050::resetGyroscopePath  (  )  

`
Reset gyroscope signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_GYRO_RESET_BIT  
`

#####  void MPU6050::resetI2CMaster  (  )  

`
Reset the I2C Master. This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0. This bit automatically clears to 0 after the reset has been triggered. 
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_I2C_MST_RESET_BIT  
`

#####  void MPU6050::resetSensors  (  )  
`
Reset all sensor registers and signal paths. When set to 1, this bit resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers. This bit automatically clears to 0 after the reset has been triggered.

When resetting only the signal path (and not the sensor registers), please use Register 104, SIGNAL_PATH_RESET.
See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_SIG_COND_RESET_BIT  
`

#####  void MPU6050::resetTemperaturePath  (  )  

`
Reset temperature sensor signal path. The reset will revert the signal path analog to digital converters and filters to their power up configurations. 
See also:MPU6050_RA_SIGNAL_PATH_RESET MPU6050_PATHRESET_TEMP_RESET_BIT  
`

#####  void MPU6050::setAccelerometerPowerOnDelay  ( uint8_t  delay )  

`
Set accelerometer power-on delay. 
Parameters:
delay New accelerometer power-on delay (0-3)  
See also:getAccelerometerPowerOnDelay() MPU6050_RA_MOT_DETECT_CTRL MPU6050_DETECT_ACCEL_ON_DELAY_BIT  
`

#####  void MPU6050::setAccelFIFOEnabled  ( bool  enabled )  

`
Set accelerometer FIFO enabled value. 
Parameters:
enabled New accelerometer FIFO enabled value  
See also:getAccelFIFOEnabled() MPU6050_RA_FIFO_EN  
`

#####  void MPU6050::setAccelXSelfTest  ( bool  enabled )  

`
Get self-test enabled setting for accelerometer X axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  
`

#####  void MPU6050::setAccelYSelfTest  ( bool  enabled )  

`
Get self-test enabled value for accelerometer Y axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  
`

#####  void MPU6050::setAccelZSelfTest  ( bool  enabled )  

`
Set self-test enabled value for accelerometer Z axis. 
Parameters:
enabled Self-test enabled value  
See also:MPU6050_RA_ACCEL_CONFIG  
`

#####  void MPU6050::setAuxVDDIOLevel  ( uint8_t  level )  

`
Set the auxiliary I2C supply voltage level. When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to the MPU-6000, which does not have a VLOGIC pin. 
Parameters:
level I2C supply voltage level (0=VLOGIC, 1=VDD)  
`


#####  void MPU6050::setClockOutputEnabled  ( bool  enabled )  

`
Set reference clock output enabled status. When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For further information regarding CLKOUT, please refer to the MPU-60X0 Product Specification document. 
Parameters:
enabled New reference clock output enabled status  
See also:MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_CLKOUT_EN_BIT  
`

#####  void MPU6050::setClockSource  ( uint8_t  source )  

`
Set clock source setting. An internal 8MHz oscillator, gyroscope based clock, or external sources can be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator or an external source is chosen as the clock source, the MPU-60X0 can operate in low power modes with the gyroscopes disabled.

Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference for improved stability. The clock source can be selected according to the following table:
`

 CLK_SEL | Clock Source
---------|-----------------------
 0       | Internal oscillator
 1       | PLL with X Gyro reference
 2       | PLL with Y Gyro reference
 3       | PLL with Z Gyro reference
 4       | PLL with external 32.768kHz reference
 5       | PLL with external 19.2MHz reference
 6       | Reserved
 7       | Stops the clock and keeps the timing generator in reset

` 
Parameters:
source New clock source setting  
See also:
getClockSource() MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_CLKSEL_BIT MPU6050_PWR1_CLKSEL_LENGTH  
`

#####  void MPU6050::setDeviceID  ( uint8_t  id )  

`
Set Device ID. Write a new ID into the WHO_AM_I register (no idea why this should ever be necessary though). 
Parameters:

id New device ID to set.  
See also:getDeviceID() MPU6050_RA_WHO_AM_I MPU6050_WHO_AM_I_BIT MPU6050_WHO_AM_I_LENGTH  
`

#####  void MPU6050::setDHPFMode  ( uint8_t  bandwidth )  

`
Set the high-pass filter configuration. 
Parameters:

bandwidth New high-pass filter configuration  
See also:setDHPFMode() MPU6050_DHPF_RESET MPU6050_RA_ACCEL_CONFIG  
`

#####  void MPU6050::setDLPFMode  ( uint8_t  mode )  

`
Set digital low-pass filter configuration. 
Parameters:

mode New DLFP configuration setting  
See also:getDLPFBandwidth() MPU6050_DLPF_BW_256 MPU6050_RA_CONFIG MPU6050_CFG_DLPF_CFG_BIT MPU6050_CFG_DLPF_CFG_LENGTH 
`


##### void MPU6050::setFreefallDetectionCounterDecrement  ( uint8_t  decrement )  

`
Set Free Fall detection counter decrement configuration. 
Parameters:
decrement New decrement configuration value  
See also:getFreefallDetectionCounterDecrement() MPU6050_RA_MOT_DETECT_CTRL MPU6050_DETECT_FF_COUNT_BIT  
`

##### void MPU6050::setFreefallDetectionDuration  ( uint8_t  duration )  

`
Get free-fall event duration threshold. 

Parameters:
duration New free-fall duration threshold value (LSB = 1ms)  
See also:getFreefallDetectionDuration() MPU6050_RA_FF_DUR  
`

##### void MPU6050::setFreefallDetectionThreshold  ( uint8_t  threshold )  

`
Get free-fall event acceleration threshold. 

Parameters:
threshold New free-fall acceleration threshold value (LSB = 2mg)  
See also:getFreefallDetectionThreshold() MPU6050_RA_FF_THR  
`

##### void MPU6050::setFSyncInterruptEnabled  ( bool  enabled )  

`
Set FSYNC pin interrupt enabled setting. 
Parameters:
enabled New FSYNC pin interrupt enabled setting  
See also:getFSyncInterruptEnabled() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_FSYNC_INT_EN_BIT  
`

##### void MPU6050::setFSyncInterruptLevel  ( bool  level )  

`
Set FSYNC interrupt logic level mode. 
Parameters:
mode New FSYNC interrupt mode (0=active-high, 1=active-low)  
See also:getFSyncInterruptMode() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  
`

##### void MPU6050::setFullScaleAccelRange  ( uint8_t  range )  

`
Set full-scale accelerometer range. 
Parameters:
range New full-scale accelerometer range setting  
See also:getFullScaleAccelRange()  
`

##### void MPU6050::setFullScaleGyroRange  ( uint8_t  range )  

`
Set full-scale gyroscope range. 
Parameters:

range New full-scale gyroscope range value  
See also:getFullScaleRange() MPU6050_GYRO_FS_250 MPU6050_RA_GYRO_CONFIG MPU6050_GCONFIG_FS_SEL_BIT MPU6050_GCONFIG_FS_SEL_LENGTH  
`

void MPU6050::setI2CBypassEnabled  ( bool  enabled )  


Set I2C bypass enabled status. When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]). 
Parameters:
enabled New I2C bypass enabled status  
See also:MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_I2C_BYPASS_EN_BIT  


void MPU6050::setI2CMasterModeEnabled  ( bool  enabled )  


Set I2C Master Mode enabled status. 
Parameters:
enabled New I2C Master Mode enabled status  
See also:getI2CMasterModeEnabled() MPU6050_RA_USER_CTRL MPU6050_USERCTRL_I2C_MST_EN_BIT  


void MPU6050::setIntDataReadyEnabled  ( bool  enabled )  


Set Data Ready interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntDataReadyEnabled() MPU6050_RA_INT_CFG MPU6050_INTERRUPT_DATA_RDY_BIT  


void MPU6050::setInterruptDrive  ( bool  drive )  


Set interrupt drive mode. 
Parameters:
drive New interrupt drive mode (0=push-pull, 1=open-drain)  
See also:getInterruptDrive() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_INT_OPEN_BIT  


void MPU6050::setInterruptLatch  ( bool  latch )  


Set interrupt latch mode. 
Parameters:
latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)  
See also:getInterruptLatch() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_LATCH_INT_EN_BIT  


void MPU6050::setInterruptLatchClear  ( bool  clear )  


Set interrupt latch clear mode. 
Parameters:
clear New latch clear mode (0=status-read-only, 1=any-register-read)  
See also:getInterruptLatchClear() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_INT_RD_CLEAR_BIT  


void MPU6050::setInterruptMode  ( bool  mode )  


Set interrupt logic level mode. 
Parameters:
mode New interrupt mode (0=active-high, 1=active-low)  
See also:getInterruptMode() MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_INT_LEVEL_BIT  


void MPU6050::setIntFIFOBufferOverflowEnabled  ( bool  enabled )  


Set FIFO Buffer Overflow interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntFIFOBufferOverflowEnabled() MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_FIFO_OFLOW_BIT  


void MPU6050::setIntFreefallEnabled  ( bool  enabled )  


Set Free Fall interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntFreefallEnabled() MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_FF_BIT  


void MPU6050::setIntI2CMasterEnabled  ( bool  enabled )  


Set I2C Master interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntI2CMasterEnabled() MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_I2C_MST_INT_BIT  


void MPU6050::setIntMotionEnabled  ( bool  enabled )  


Set Motion Detection interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntMotionEnabled() MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_MOT_BIT  


void MPU6050::setIntZeroMotionEnabled  ( bool  enabled )  


Set Zero Motion Detection interrupt enabled status. 
Parameters:
enabled New interrupt enabled status  
See also:getIntZeroMotionEnabled() MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_ZMOT_BIT  


void MPU6050::setMasterClockSpeed  ( uint8_t  speed )  


Set I2C master clock speed. speed Current I2C master clock speed 
See also:MPU6050_RA_I2C_MST_CTRL  


void MPU6050::setMotionDetectionCounterDecrement  ( uint8_t  decrement )  


Set Motion detection counter decrement configuration. 
Parameters:
decrement New decrement configuration value  
See also:getMotionDetectionCounterDecrement() MPU6050_RA_MOT_DETECT_CTRL MPU6050_DETECT_MOT_COUNT_BIT  


void MPU6050::setMotionDetectionDuration  ( uint8_t  duration )  


Set motion detection event duration threshold. 
Parameters:
duration New motion detection duration threshold value (LSB = 1ms)  
See also:getMotionDetectionDuration() MPU6050_RA_MOT_DUR  


void MPU6050::setMotionDetectionThreshold  ( uint8_t  threshold )  


Set free-fall event acceleration threshold. 
Parameters:
threshold New motion detection acceleration threshold value (LSB = 2mg)  
See also:getMotionDetectionThreshold() MPU6050_RA_MOT_THR  


void MPU6050::setMultiMasterEnabled  ( bool  enabled )  


Set multi-master enabled value. 
Parameters:
enabled New multi-master enabled value  
See also:getMultiMasterEnabled() MPU6050_RA_I2C_MST_CTRL  


void MPU6050::setRate  ( uint8_t  rate )  


Set gyroscope sample rate divider. 
Parameters:
rate New sample rate divider  
See also:getRate() MPU6050_RA_SMPLRT_DIV  


void MPU6050::setSlave0FIFOEnabled  ( bool  enabled )  


Set Slave 0 FIFO enabled value. 
Parameters:
enabled New Slave 0 FIFO enabled value  
See also:getSlave0FIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setSlave1FIFOEnabled  ( bool  enabled )  


Set Slave 1 FIFO enabled value. 
Parameters:
enabled New Slave 1 FIFO enabled value  
See also:getSlave1FIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setSlave2FIFOEnabled  ( bool  enabled )  


Set Slave 2 FIFO enabled value. 
Parameters:
enabled New Slave 2 FIFO enabled value  
See also:getSlave2FIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setSlave3FIFOEnabled  ( bool  enabled )  


Set Slave 3 FIFO enabled value. 
Parameters:
enabled New Slave 3 FIFO enabled value  
See also:getSlave3FIFOEnabled() MPU6050_RA_MST_CTRL  


void MPU6050::setSlave4Address  ( uint8_t  address )  


Set the I2C address of Slave 4. 
Parameters:
address New address for Slave 4  
See also:getSlave4Address() MPU6050_RA_I2C_SLV4_ADDR  


void MPU6050::setSlave4Enabled  ( bool  enabled )  


Set the enabled value for Slave 4. 
Parameters:
enabled New enabled value for Slave 4  
See also:getSlave4Enabled() MPU6050_RA_I2C_SLV4_CTRL  


void MPU6050::setSlave4InterruptEnabled  ( bool  enabled )  


Set the enabled value for Slave 4 transaction interrupts. 
Parameters:
enabled New enabled value for Slave 4 transaction interrupts.  
See also:getSlave4InterruptEnabled() MPU6050_RA_I2C_SLV4_CTRL  


void MPU6050::setSlave4MasterDelay  ( uint8_t  delay )  


Set Slave 4 master delay value. 
Parameters:
delay New Slave 4 master delay value  
See also:getSlave4MasterDelay() MPU6050_RA_I2C_SLV4_CTRL  


void MPU6050::setSlave4OutputByte  ( uint8_t  data )  


Set new byte to write to Slave 4. This register stores the data to be written into the Slave 4. If I2C_SLV4_RW is set 1 (set to read), this register has no effect. 
Parameters:
data New byte to write to Slave 4  
See also:MPU6050_RA_I2C_SLV4_DO  


void MPU6050::setSlave4Register  ( uint8_t  reg )  


Set the active internal register for Slave 4. 
Parameters:
reg New active register for Slave 4  
See also:getSlave4Register() MPU6050_RA_I2C_SLV4_REG  


void MPU6050::setSlave4WriteMode  ( bool  mode )  


Set write mode for the Slave 4. 
Parameters:
mode New write mode for Slave 4 (0 = register address + data, 1 = data only)  
See also:getSlave4WriteMode() MPU6050_RA_I2C_SLV4_CTRL  


void MPU6050::setSlaveAddress  ( uint8_t  num,  
  uint8_t  address  
 )   


Set the I2C address of the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
address New address for specified slave  
See also:getSlaveAddress() MPU6050_RA_I2C_SLV0_ADDR  


void MPU6050::setSlaveDataLength  ( uint8_t  num,  
  uint8_t  length  
 )   


Set number of bytes to read for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
length Number of bytes to read for specified slave  
See also:getSlaveDataLength() MPU6050_RA_I2C_SLV0_CTRL  


void MPU6050::setSlaveDelayEnabled  ( uint8_t  num,  
  bool  enabled  
 )   


Set slave delay enabled status. 
Parameters:
num Slave number (0-4)  
enabled New slave delay enabled status.  
See also:MPU6050_RA_I2C_MST_DELAY_CTRL MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT  


void MPU6050::setSlaveEnabled  ( uint8_t  num,  
  bool  enabled  
 )   


Set the enabled value for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
enabled New enabled value for specified slave  
See also:getSlaveEnabled() MPU6050_RA_I2C_SLV0_CTRL  


void MPU6050::setSlaveOutputByte  ( uint8_t  num,  
  uint8_t  data  
 )   


Write byte to Data Output container for specified slave. This register holds the output data written into Slave when Slave is set to write mode. For further information regarding Slave control, please refer to Registers 37 to 39 and immediately following. 
Parameters:
num Slave number (0-3)  
data Byte to write  
See also:MPU6050_RA_I2C_SLV0_DO  


void MPU6050::setSlaveReadWriteTransitionEnabled  ( bool  enabled )  


Set slave read/write transition enabled value. 
Parameters:
enabled New slave read/write transition enabled value  
See also:getSlaveReadWriteTransitionEnabled() MPU6050_RA_I2C_MST_CTRL  


void MPU6050::setSlaveRegister  ( uint8_t  num,  
  uint8_t  reg  
 )   


Set the active internal register for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
reg New active register for specified slave  
See also:getSlaveRegister() MPU6050_RA_I2C_SLV0_REG  


void MPU6050::setSlaveWordByteSwap  ( uint8_t  num,  
  bool  enabled  
 )   


Set word pair byte-swapping enabled for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
enabled New word pair byte-swapping enabled value for specified slave  
See also:getSlaveWordByteSwap() MPU6050_RA_I2C_SLV0_CTRL  


void MPU6050::setSlaveWordGroupOffset  ( uint8_t  num,  
  bool  enabled  
 )   


Set word pair grouping order offset for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
enabled New word pair grouping order offset for specified slave  
See also:getSlaveWordGroupOffset() MPU6050_RA_I2C_SLV0_CTRL  


void MPU6050::setSlaveWriteMode  ( uint8_t  num,  
  bool  mode  
 )   


Set write mode for the specified slave (0-3). 
Parameters:
num Slave number (0-3)  
mode New write mode for specified slave (0 = register address + data, 1 = data only)  
See also:getSlaveWriteMode() MPU6050_RA_I2C_SLV0_CTRL  


void MPU6050::setSleepEnabled  ( bool  enabled )  


Set sleep mode status. 
Parameters:
enabled New sleep mode enabled status  
See also:getSleepEnabled() MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_SLEEP_BIT  


void MPU6050::setStandbyXAccelEnabled  ( bool  enabled )  


Set X-axis accelerometer standby enabled status. 
Parameters:
New X-axis standby enabled status  
See also:getStandbyXAccelEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_XA_BIT  


void MPU6050::setStandbyXGyroEnabled  ( bool  enabled )  


Set X-axis gyroscope standby enabled status. 
Parameters:
New X-axis standby enabled status  
See also:getStandbyXGyroEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_XG_BIT  


void MPU6050::setStandbyYAccelEnabled  ( bool  enabled )  


Set Y-axis accelerometer standby enabled status. 
Parameters:
New Y-axis standby enabled status  
See also:getStandbyYAccelEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_YA_BIT  


void MPU6050::setStandbyYGyroEnabled  ( bool  enabled )  


Set Y-axis gyroscope standby enabled status. 
Parameters:
New Y-axis standby enabled status  
See also:getStandbyYGyroEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_YG_BIT  


void MPU6050::setStandbyZAccelEnabled  ( bool  enabled )  


Set Z-axis accelerometer standby enabled status. 
Parameters:
New Z-axis standby enabled status  
See also:getStandbyZAccelEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_ZA_BIT  


void MPU6050::setStandbyZGyroEnabled  ( bool  enabled )  


Set Z-axis gyroscope standby enabled status. 
Parameters:
New Z-axis standby enabled status  
See also:getStandbyZGyroEnabled() MPU6050_RA_PWR_MGMT_2 MPU6050_PWR2_STBY_ZG_BIT  


void MPU6050::setTempFIFOEnabled  ( bool  enabled )  


Set temperature FIFO enabled value. 
Parameters:
enabled New temperature FIFO enabled value  
See also:getTempFIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setTempSensorEnabled  ( bool  enabled )  


Set temperature sensor enabled status. Note: this register stores the *disabled* value, but for consistency with the rest of the code, the function is named and used with standard true/false values to indicate whether the sensor is enabled or disabled, respectively.
Parameters:
enabled New temperature sensor enabled status  
See also:getTempSensorEnabled() MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_TEMP_DIS_BIT  


void MPU6050::setWaitForExternalSensorEnabled  ( bool  enabled )  


Set wait-for-external-sensor-data enabled value. 
Parameters:
enabled New wait-for-external-sensor-data enabled value  
See also:getWaitForExternalSensorEnabled() MPU6050_RA_I2C_MST_CTRL  


void MPU6050::setWakeCycleEnabled  ( bool  enabled )  


Set wake cycle enabled status. 
Parameters:
enabled New sleep mode enabled status  
See also:getWakeCycleEnabled() MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_CYCLE_BIT  


void MPU6050::setWakeFrequency  ( uint8_t  frequency )  


Set wake frequency in Accel-Only Low Power Mode. 
Parameters:
frequency New wake frequency  
See also:MPU6050_RA_PWR_MGMT_2  


void MPU6050::setXGyroFIFOEnabled  ( bool  enabled )  


Set gyroscope X-axis FIFO enabled value. 
Parameters:
enabled New gyroscope X-axis FIFO enabled value  
See also:getXGyroFIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setYGyroFIFOEnabled  ( bool  enabled )  


Set gyroscope Y-axis FIFO enabled value. 
Parameters:
enabled New gyroscope Y-axis FIFO enabled value  
See also:getYGyroFIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::setZeroMotionDetectionDuration  ( uint8_t  duration )  


Set zero motion detection event duration threshold. 
Parameters:
duration New zero motion detection duration threshold value (LSB = 1ms)  
See also:getZeroMotionDetectionDuration() MPU6050_RA_ZRMOT_DUR  


void MPU6050::setZeroMotionDetectionThreshold  ( uint8_t  threshold )  


Set zero motion detection event acceleration threshold. 
Parameters:
threshold New zero motion detection acceleration threshold value (LSB = 2mg)  
See also:getZeroMotionDetectionThreshold() MPU6050_RA_ZRMOT_THR  


void MPU6050::setZGyroFIFOEnabled  ( bool  enabled )  


Set gyroscope Z-axis FIFO enabled value. 
Parameters:
enabled New gyroscope Z-axis FIFO enabled value  
See also:getZGyroFIFOEnabled() MPU6050_RA_FIFO_EN  


void MPU6050::switchSPIEnabled  ( bool  enabled )  


Switch from I2C to SPI mode (MPU-6000 only) If this is set, the primary SPI interface will be enabled in place of the disabled primary I2C interface. 
 


bool MPU6050::testConnection  (  )  


Verify the I2C connection. Make sure the device is connected and responds as expected. 
Returns:True if connection is valid, false otherwise 
The documentation for this class was generated from the following files:•MPU6050/MPU6050.h
•MPU6050/MPU6050.cpp
