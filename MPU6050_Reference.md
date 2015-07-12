### GET DATA by MPU CLASS REFERENCE
http://www.i2cdevlib.com/docs/html/class_m_p_u6050.html#a7c0146d45537e4bd7a0d4c1c476fdab7


###### Member Function Documentation

```
void MPU6050::getAcceleration ( int16_t *  x,    int16_t *  y,    int16_t *  z   ) 
```
Get 3-axis accelerometer readings. These registers store the most recent accelerometer measurements. Accelerometer measurements are written to these registers at the Sample Rate as defined in Register 25.

The accelerometer measurement registers, along with the temperature measurement registers, gyroscope measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set.

The data within the accelerometer sensors' internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set's data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.

Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS (Register 28). For each full scale setting, the accelerometers' sensitivity per LSB in ACCEL_xOUT is shown in the table below:

 AFS_SEL | Full Scale Range | LSB Sensitivity
---------|------------------|----------------
 0       |   +/- 2g         | 8192 LSB/mg
 1       |   +/- 4g         | 4096 LSB/mg
 2       |   +/- 8g         | 2048 LSB/mg
 3       |   +/- 16g        | 1024 LSB/mg
 
 
Parameters:

x 16-bit signed integer container for X-axis acceleration  
y 16-bit signed integer container for Y-axis acceleration  
z 16-bit signed integer container for Z-axis acceleration  
See also:MPU6050_RA_GYRO_XOUT_H 


```
int16_t MPU6050::getAccelerationX ()
```
Get X-axis accelerometer reading. 
Returns:X-axis acceleration measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_ACCEL_XOUT_H 


```
int16_t MPU6050::getAccelerationY  (  )  
```
Get Y-axis accelerometer reading. 
Returns:Y-axis acceleration measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_ACCEL_YOUT_H 

```
int16_t MPU6050::getAccelerationZ  (  )  
```
Get Z-axis accelerometer reading. 
Returns:Z-axis acceleration measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_ACCEL_ZOUT_H

```
bool MPU6050::getAccelXSelfTest  (  )  
```
Get self-test enabled setting for accelerometer X axis. 
Returns:Self-test enabled value See also:MPU6050_RA_ACCEL_CONFIG  

```
bool MPU6050::getAccelYSelfTest  (  )  
```
Get self-test enabled value for accelerometer Y axis. 
Returns:Self-test enabled value See also:MPU6050_RA_ACCEL_CONFIG  

```
bool MPU6050::getAccelZSelfTest  (  )  
```
Get self-test enabled value for accelerometer Z axis. 
Returns:Self-test enabled value See also:MPU6050_RA_ACCEL_CONFIG 

```
bool MPU6050::getClockOutputEnabled  (  )  
```
Get reference clock output enabled status. When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For further information regarding CLKOUT, please refer to the MPU-60X0 Product Specification document. 
Returns:Current reference clock output enabled status See also:MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_CLKOUT_EN_BIT  

```
uint8_t MPU6050::getClockSource  (  )  
```
Get clock source setting. 
Returns:Current clock source setting See also:MPU6050_RA_PWR_MGMT_1 MPU6050_PWR1_CLKSEL_BIT MPU6050_PWR1_CLKSEL_LENGTH  

```
uint8_t MPU6050::getDeviceID  (  )  
```
Get Device ID. This register is used to verify the identity of the device (0b110100). 
Returns:Device ID (should be 0x68, 104 dec, 150 oct) See also:MPU6050_RA_WHO_AM_I MPU6050_WHO_AM_I_BIT MPU6050_WHO_AM_I_LENGTH 

```
uint8_t MPU6050::getDHPFMode  (  )  
```
Get the high-pass filter configuration. The DHPF is a filter module in the path leading to motion detectors (Free Fall, Motion threshold, and Zero Motion). The high pass filter output is not available to the data registers (see Figure in Section 8 of the MPU-6000/ MPU-6050 Product Specification document).

The high pass filter has three modes:
    Reset: The filter output settles to zero within one sample. This
           effectively disables the high pass filter. This mode may be toggled
           to quickly settle the filter.
    On:    The high pass filter will pass signals above the cut off frequency.
    Hold:  When triggered, the filter holds the present sample. The filter
           output will be the difference between the input sample and the held
           sample.
 
 ACCEL_HPF | Filter Mode | Cut-off Frequency
-----------|-------------|------------------
 0         | Reset       |    None
 1         | On          |    5Hz
 2         | On          |   2.5Hz
 3         | On          |   1.25Hz
 4         | On          |   0.63Hz
 7         | Hold        |    None
 
Returns:Current high-pass filter configuration See also:MPU6050_DHPF_RESET MPU6050_RA_ACCEL_CONFIG  

```
uint8_t MPU6050::getDLPFMode  (  )  
```

Get digital low-pass filter configuration. The DLPF_CFG parameter sets the digital low pass filter configuration. It also determines the internal sampling rate used by the device as shown in the table below.

Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz, the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than once.


| FILTER   |  ACCEL    | ACCEL  | GYRO      | GYRO   | Sample Rate
|----------|-----------|--------|-----------|--------|-------------
|DLPF_CFG  | Bandwidth |  Delay | Bandwidth |  Delay | 
| 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
| 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
| 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
| 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
| 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
| 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
| 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
| 7        |   -- Reserved --   |   -- Reserved --   | Reserved

Returns:DLFP configuration See also:MPU6050_RA_CONFIG MPU6050_CFG_DLPF_CFG_BIT MPU6050_CFG_DLPF_CFG_LENGTH 


```
uint8_t MPU6050::getExternalFrameSync  (  )  
```

Get external FSYNC configuration. Configures the external Frame Synchronization (FSYNC) pin sampling. An external signal connected to the FSYNC pin can be sampled by configuring EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short strobes may be captured. The latched FSYNC signal will be sampled at the Sampling Rate, as defined in register 25. After sampling, the latch will reset to the current FSYNC signal state.

The sampled value will be reported in place of the least significant bit in a sensor data register determined by the value of EXT_SYNC_SET according to the following table.

 EXT_SYNC_SET | FSYNC Bit Location
--------------|----------------------
 0            | Input disabled
 1            | TEMP_OUT_L[0]
 2            | GYRO_XOUT_L[0]
 3            | GYRO_YOUT_L[0]
 4            | GYRO_ZOUT_L[0]
 5            | ACCEL_XOUT_L[0]
 6            | ACCEL_YOUT_L[0]
 7            | ACCEL_ZOUT_L[0]
 
Returns:FSYNC configuration value 


```
uint8_t MPU6050::getExternalSensorByte  ( int  position )  
```

Read single byte from external sensor data register. These registers store data read from external sensors by the Slave 0, 1, 2, and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in I2C_SLV4_DI (Register 53).

External sensor data is written to these registers at the Sample Rate as defined in Register 25. This access rate can be reduced by using the Slave Delay Enable registers (Register 103).

External sensor data registers, along with the gyroscope measurement registers, accelerometer measurement registers, and temperature measurement registers, are composed of two sets of registers: an internal register set and a user-facing read register set.

The data within the external sensors' internal register set is always updated at the Sample Rate (or the reduced access rate) whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.

Data is placed in these external sensor data registers according to I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in Register 52 and 103). During each Sample cycle, slave reads are performed in order of Slave number. If all slaves are enabled with more than zero bytes to be read, the order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.

Each enabled slave will have EXT_SENS_DATA registers associated with it by number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may change the higher numbered slaves' associated registers. Furthermore, if fewer total bytes are being read from the external sensors as a result of such a change, then the data remaining in the registers which no longer have an associated slave device (i.e. high numbered registers) will remain in these previously allocated registers unless reset.

If the sum of the read lengths of all SLVx transactions exceed the number of available EXT_SENS_DATA registers, the excess bytes will be dropped. There are 24 EXT_SENS_DATA registers and hence the total read lengths between all the slaves cannot be greater than 24 or some bytes will be lost.

Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further information regarding the characteristics of Slave 4, please refer to Registers 49 to 53.

EXAMPLE: Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00 through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05 will be associated with Slave 1. If Slave 2 is enabled as well, registers starting from EXT_SENS_DATA_06 will be allocated to Slave 2.

If Slave 2 is disabled while Slave 3 is enabled in this same situation, then registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3 instead.

REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE: If a slave is disabled at any time, the space initially allocated to the slave in the EXT_SENS_DATA register, will remain associated with that slave. This is to avoid dynamic adjustment of the register allocation.

The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).

This above is also true if one of the slaves gets NACKed and stops functioning.
Parameters:

position Starting position (0-23)  
Returns:Byte read from register 

```
uint8_t MPU6050::getFullScaleAccelRange  (  )  
```

Get full-scale accelerometer range. The FS_SEL parameter allows setting the full-scale range of the accelerometer sensors, as described in the table below.
 0 = +/- 2g
 1 = +/- 4g
 2 = +/- 8g
 3 = +/- 16g
 
Returns:Current full-scale accelerometer range setting See also:MPU6050_ACCEL_FS_2 MPU6050_RA_ACCEL_CONFIG MPU6050_ACONFIG_AFS_SEL_BIT MPU6050_ACONFIG_AFS_SEL_LENGTH  

```
uint8_t MPU6050::getFullScaleGyroRange  (  )  
```

Get full-scale gyroscope range. The FS_SEL parameter allows setting the full-scale range of the gyro sensors, as described in the table below.
 0 = +/- 250 degrees/sec
 1 = +/- 500 degrees/sec
 2 = +/- 1000 degrees/sec
 3 = +/- 2000 degrees/sec
 
Returns:Current full-scale gyroscope range setting See also:MPU6050_GYRO_FS_250 MPU6050_RA_GYRO_CONFIG MPU6050_GCONFIG_FS_SEL_BIT MPU6050_GCONFIG_FS_SEL_LENGTH 


```
uint8_t MPU6050::getFreefallDetectionCounterDecrement  (  )  
```

Get Free Fall detection counter decrement configuration. Detection is registered by the Free Fall detection module after accelerometer measurements meet their respective threshold conditions over a specified number of samples. When the threshold conditions are met, the corresponding detection counter increments by 1. The user may control the rate at which the detection counter decrements when the threshold condition is not met by configuring FF_COUNT. The decrement rate can be set according to the following table:

 FF_COUNT | Counter Decrement
----------|------------------
 0        | Reset
 1        | 1
 2        | 2
 3        | 4
 

When FF_COUNT is configured to 0 (reset), any non-qualifying sample will reset the counter to 0. For further information on Free Fall detection, please refer to Registers 29 to 32.
Returns:Current decrement configuration See also:MPU6050_RA_MOT_DETECT_CTRL MPU6050_DETECT_FF_COUNT_BIT  

```
uint8_t MPU6050::getFreefallDetectionDuration  (  )  
```

Get free-fall event duration threshold. This register configures the duration counter threshold for Free Fall event detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit of 1 LSB = 1 ms.

The Free Fall duration counter increments while the absolute value of the accelerometer measurements are each less than the detection threshold (Register 29). The Free Fall interrupt is triggered when the Free Fall duration counter reaches the time specified in this register.

For more details on the Free Fall detection interrupt, see Section 8.2 of the MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and 58 of this document.
Returns:Current free-fall duration threshold value (LSB = 1ms) See also:MPU6050_RA_FF_DUR  

```
uint8_t MPU6050::getFreefallDetectionThreshold  (  )  
```

Get free-fall event acceleration threshold. This register configures the detection threshold for Free Fall event detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the absolute value of the accelerometer measurements for the three axes are each less than the detection threshold. This condition increments the Free Fall duration counter (Register 30). The Free Fall interrupt is triggered when the Free Fall duration counter reaches the time specified in FF_DUR.

For more details on the Free Fall detection interrupt, see Section 8.2 of the MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and 58 of this document.
Returns:Current free-fall acceleration threshold value (LSB = 2mg) See also:MPU6050_RA_FF_THR 


```
bool MPU6050::getI2CBypassEnabled  (  )  
```
Get I2C bypass enabled status. When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]). 
Returns:Current I2C bypass enabled status See also:MPU6050_RA_INT_PIN_CFG MPU6050_INTCFG_I2C_BYPASS_EN_BIT 

```
bool MPU6050::getI2CMasterModeEnabled  (  )  
```
Get I2C Master Mode enabled status. When this mode is enabled, the MPU-60X0 acts as the I2C Master to the external sensor slave devices on the auxiliary I2C bus. When this bit is cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically driven by the primary I2C bus (SDA and SCL). This is a precondition to enabling Bypass Mode. For further information regarding Bypass Mode, please refer to Register 55. 
Returns:Current I2C Master Mode enabled status See also:MPU6050_RA_USER_CTRL MPU6050_USERCTRL_I2C_MST_EN_BIT 

 

```
bool MPU6050::getIntFreefallEnabled  (  )  
```
Get Free Fall interrupt enabled status. Will be set 0 for disabled, 1 for enabled. 
Returns:Current interrupt enabled status See also:MPU6050_RA_INT_ENABLE MPU6050_INTERRUPT_FF_BIT  

```
bool MPU6050::getIntFreefallStatus  (  )  
```
Get Free Fall interrupt status. This bit automatically sets to 1 when a Free Fall interrupt has been generated. The bit clears to 0 after the register has been read. 
Returns:Current interrupt status See also:MPU6050_RA_INT_STATUS MPU6050_INTERRUPT_FF_BIT

```
uint8_t MPU6050::getMasterClockSpeed  (  )  
```
Get I2C master clock speed. I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to the following table:
 I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 ------------+------------------------+-------------------
 0           | 348kHz                 | 23
 1           | 333kHz                 | 24
 2           | 320kHz                 | 25
 3           | 308kHz                 | 26
 4           | 296kHz                 | 27
 5           | 286kHz                 | 28
 6           | 276kHz                 | 29
 7           | 267kHz                 | 30
 8           | 258kHz                 | 31
 9           | 500kHz                 | 16
 10          | 471kHz                 | 17
 11          | 444kHz                 | 18
 12          | 421kHz                 | 19
 13          | 400kHz                 | 20
 14          | 381kHz                 | 21
 15          | 364kHz                 | 22
 
Returns:Current I2C master clock speed See also:MPU6050_RA_I2C_MST_CTRL  



```
void MPU6050::getMotion6  ( int16_t *  ax,  
  int16_t *  ay,  
  int16_t *  az,  
  int16_t *  gx,  
  int16_t *  gy,  
  int16_t *  gz  
 )   
```
Get raw 6-axis motion sensor readings (accel/gyro). Retrieves all currently available motion sensor values. 
Parameters:
ax 16-bit signed integer container for accelerometer X-axis value  
ay 16-bit signed integer container for accelerometer Y-axis value  
az 16-bit signed integer container for accelerometer Z-axis value  
gx 16-bit signed integer container for gyroscope X-axis value  
gy 16-bit signed integer container for gyroscope Y-axis value  
gz 16-bit signed integer container for gyroscope Z-axis value  
See also:getAcceleration() getRotation() MPU6050_RA_ACCEL_XOUT_H  



```
void MPU6050::getMotion9  ( int16_t *  ax,  
  int16_t *  ay,  
  int16_t *  az,  
  int16_t *  gx,  
  int16_t *  gy,  
  int16_t *  gz,  
  int16_t *  mx,  
  int16_t *  my,  
  int16_t *  mz  
 )   
```
Get raw 9-axis motion sensor readings (accel/gyro/compass). FUNCTION NOT FULLY IMPLEMENTED YET. 
Parameters:
ax 16-bit signed integer container for accelerometer X-axis value  
ay 16-bit signed integer container for accelerometer Y-axis value  
az 16-bit signed integer container for accelerometer Z-axis value  
gx 16-bit signed integer container for gyroscope X-axis value  
gy 16-bit signed integer container for gyroscope Y-axis value  
gz 16-bit signed integer container for gyroscope Z-axis value  
mx 16-bit signed integer container for magnetometer X-axis value  
my 16-bit signed integer container for magnetometer Y-axis value  
mz 16-bit signed integer container for magnetometer Z-axis value  
See also:getMotion6() getAcceleration() getRotation() MPU6050_RA_ACCEL_XOUT_H  

```
uint8_t MPU6050::getMotionDetectionCounterDecrement  (  )  
```
Get Motion detection counter decrement configuration. Detection is registered by the Motion detection module after accelerometer measurements meet their respective threshold conditions over a specified number of samples. When the threshold conditions are met, the corresponding detection counter increments by 1. The user may control the rate at which the detection counter decrements when the threshold condition is not met by configuring MOT_COUNT. The decrement rate can be set according to the following table:

 MOT_COUNT | Counter Decrement
 ----------+------------------
 0         | Reset
 1         | 1
 2         | 2
 3         | 4
 

When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will reset the counter to 0. For further information on Motion detection, please refer to Registers 29 to 32. 
 

```
uint8_t MPU6050::getMotionDetectionDuration  (  )  
```
Get motion detection event duration threshold. This register configures the duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1LSB = 1ms. The Motion detection duration counter increments when the absolute value of any of the accelerometer measurements exceeds the Motion detection threshold (Register 31). The Motion detection interrupt is triggered when the Motion detection counter reaches the time count specified in this register.

For more details on the Motion detection interrupt, see Section 8.3 of the MPU-6000/MPU-6050 Product Specification document.
Returns:Current motion detection duration threshold value (LSB = 1ms) See also:MPU6050_RA_MOT_DUR  

```
uint8_t MPU6050::getMotionDetectionThreshold  (  )  
```
Get motion detection event acceleration threshold. This register configures the detection threshold for Motion interrupt generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the absolute value of any of the accelerometer measurements exceeds this Motion detection threshold. This condition increments the Motion detection duration counter (Register 32). The Motion detection interrupt is triggered when the Motion Detection counter reaches the time count specified in MOT_DUR (Register 32).

The Motion interrupt will indicate the axis and polarity of detected motion in MOT_DETECT_STATUS (Register 97).

For more details on the Motion detection interrupt, see Section 8.3 of the MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and 58 of this document.
Returns:Current motion detection acceleration threshold value (LSB = 2mg) See also:MPU6050_RA_MOT_THR 


```
uint8_t MPU6050::getRate  (  )  
```

Get gyroscope output rate divider. The sensor register output, FIFO output, DMP sampling, Motion detection, Zero Motion detection, and Free Fall detection are all based on the Sample Rate. The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:

Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)

where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled (see Register 26).

Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz, the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than once.

For a diagram of the gyroscope and accelerometer signal paths, see Section 8 of the MPU-6000/MPU-6050 Product Specification document.
Returns:Current sample rate See also:MPU6050_RA_SMPLRT_DIV  

```
void MPU6050::getRotation  ( int16_t *  x,   int16_t *  y,    int16_t *  z  )   
```

Get 3-axis gyroscope readings. These gyroscope measurement registers, along with the accelerometer measurement registers, temperature measurement registers, and external sensor data registers, are composed of two sets of registers: an internal register set and a user-facing read register set. The data within the gyroscope sensors' internal register set is always updated at the Sample Rate. Meanwhile, the user-facing read register set duplicates the internal register set's data values whenever the serial interface is idle. This guarantees that a burst read of sensor registers will read measurements from the same sampling instant. Note that if burst reads are not used, the user is responsible for ensuring a set of single byte reads correspond to a single sampling instant by checking the Data Ready interrupt.

Each 16-bit gyroscope measurement has a full scale defined in FS_SEL (Register 27). For each full scale setting, the gyroscopes' sensitivity per LSB in GYRO_xOUT is shown in the table below:
 
 FS_SEL | Full Scale Range   | LSB Sensitivity
--------|--------------------|------------------
 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
Parameters:

x 16-bit signed integer container for X-axis rotation  
y 16-bit signed integer container for Y-axis rotation  
z 16-bit signed integer container for Z-axis rotation  
See also:getMotion6() MPU6050_RA_GYRO_XOUT_H  

```
int16_t MPU6050::getRotationX  (  )  
```
Get X-axis gyroscope reading. 
Returns:X-axis rotation measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_GYRO_XOUT_H  

```
int16_t MPU6050::getRotationY  (  )  
```
Get Y-axis gyroscope reading. 
Returns:Y-axis rotation measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_GYRO_YOUT_H  

```
int16_t MPU6050::getRotationZ  (  )  
```
Get Z-axis gyroscope reading. 
Returns:Z-axis rotation measurement in 16-bit 2's complement format See also:getMotion6() MPU6050_RA_GYRO_ZOUT_H  

```
uint8_t MPU6050::getSlate4InputByte  (  )  
```
Get last available byte read from Slave 4. This register stores the data read from Slave 4. This field is populated after a read transaction. 
Returns:Last available byte read from to Slave 4 See also:MPU6050_RA_I2C_SLV4_DI 


```
bool MPU6050::getXNegMotionDetected  (  )  
```
Get X-axis negative motion detection interrupt status. 
Returns:Motion detection status See also:MPU6050_RA_MOT_DETECT_STATUS MPU6050_MOTION_MOT_XNEG_BIT  

```
bool MPU6050::getXPosMotionDetected  (  )  
```
Get X-axis positive motion detection interrupt status. 
Returns:Motion detection status See also:MPU6050_RA_MOT_DETECT_STATUS MPU6050_MOTION_MOT_XPOS_BIT  

```
bool MPU6050::getYGyroFIFOEnabled  (  )  
```
Get gyroscope Y-axis FIFO enabled value. When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and 70) to be written into the FIFO buffer. 
Returns:Current gyroscope Y-axis FIFO enabled value See also:MPU6050_RA_FIFO_EN  

```
bool MPU6050::getYNegMotionDetected  (  )  
```
Get Y-axis negative motion detection interrupt status. 
Returns:Motion detection status See also:MPU6050_RA_MOT_DETECT_STATUS MPU6050_MOTION_MOT_YNEG_BIT  

```
bool MPU6050::getYPosMotionDetected  (  )  
```
Get Y-axis positive motion detection interrupt status. 
Returns:Motion detection status See also:MPU6050_RA_MOT_DETECT_STATUS MPU6050_MOTION_MOT_YPOS_BIT  

```
bool MPU6050::getZeroMotionDetected  (  )  
```
Get zero motion detection interrupt status. 
Returns:Motion detection status See also:MPU6050_RA_MOT_DETECT_STATUS MPU6050_MOTION_MOT_ZRMOT_BIT  

```
uint8_t MPU6050::getZeroMotionDetectionDuration  (  )  
```
Get zero motion detection event duration threshold. This register configures the duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter increments while the absolute value of the accelerometer measurements are each less than the detection threshold (Register 33). The Zero Motion interrupt is triggered when the Zero Motion duration counter reaches the time count specified in this register.

For more details on the Zero Motion detection interrupt, see Section 8.4 of the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56 and 58 of this document.
Returns:Current zero motion detection duration threshold value (LSB = 64ms) See also:MPU6050_RA_ZRMOT_DUR  

```
uint8_t MPU6050::getZeroMotionDetectionThreshold  (  )  
```
Get zero motion detection event acceleration threshold. This register configures the detection threshold for Zero Motion interrupt generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when the absolute value of the accelerometer measurements for the 3 axes are each less than the detection threshold. This condition increments the Zero Motion duration counter (Register 34). The Zero Motion interrupt is triggered when the Zero Motion duration counter reaches the time count specified in ZRMOT_DUR (Register 34).

Unlike Free Fall or Motion detection, Zero Motion detection triggers an interrupt both when Zero Motion is first detected and when Zero Motion is no longer detected.

When a zero motion event is detected, a Zero Motion Status will be indicated in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion condition is detected, the status bit is set to 1. When a zero-motion-to- motion condition is detected, the status bit is set to 0.

For more details on the Zero Motion detection interrupt, see Section 8.4 of the MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and 58 of this document.
Returns:Current zero motion detection acceleration threshold value (LSB = 2mg) See also:MPU6050_RA_ZRMOT_THR 
