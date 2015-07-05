# Step by Step MPU6050 (Work in progress)

Identify and define I2C Address declaring as gloval variable with  #define
There are teo possible address depending on how the wires are lay-out on the PCB,  
```
#define mpu6050Address  0x68 // AD0 is logic low on the PCB
#define mpu6050Address  0x68 // AD0 is logic low on the PCB
```
We can define Timeout, time to check connection and error in I2C communication
```
#define mpu6050Timeout  1000
```

Identify and define Registry address See MPU6050 datasheet
```
#define registerAddress  0x19
```

I2C functions for  communication, to call this function we need three variable, registerAddress, data, stop bit 
```
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) 
{
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

```

Begin Transmission
```
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
```
