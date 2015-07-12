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
