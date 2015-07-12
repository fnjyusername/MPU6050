### MPU CLASS REFERENCE - 
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

