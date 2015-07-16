#include "Arduino.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
float last_x_angle=0;  // These are the filtered angles
float last_y_angle=0;
float last_z_angle=0;  
float last_gyro_x_angle=0;  // Store the gyro angles to compare drift
float last_gyro_y_angle=0;
float last_gyro_z_angle=0;


//  Use the following global variables 
//  to calibrate the Gyroscope sensor, Accelerometer, and Magnetometer readings
float    offset_x_gyro = 0;
float    offset_y_gyro = 0;
float    offset_z_gyro = 0;
float    offset_x_accel = 0;
float    offset_y_accel = 0;
float    offset_z_accel = 0;

float    base_x_mag = 0;
float    base_y_mag = 0;
float    base_z_mag = 0;

float    GYRO_FACTOR;// This global variable tells us how to scale gyroscope data
float    ACCEL_FACTOR;// This global varible tells how to scale acclerometer data

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Time Variable
volatile unsigned long  lastTime=0, now=0; 
volatile unsigned long  dt_loop=0;

/*IMU STABILIZATION VARIABLES */
float Acc_x, Acc_y, Acc_z;
float Gyr_x, Gyr_y, Gyr_z;

//Declared by Exceptions
float gyro_angle_z=0;

MPU6050 mpu;
void setup() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif  
 

//Serial Initialization
Serial.begin(115200); Serial1.begin(115200); delay(500);//give time for serial to initialized 
while (Serial.available() && Serial.read()); delay(500);// empty buffer 

/*********************START OF MPU 6050 CONFIG********************************/ 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
 MPU6050_GYRO_FS_250         0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_GYRO_FS_500         0x01
 MPU6050_GYRO_FS_1000        0x02
 MPU6050_GYRO_FS_2000        0x03
 
 MPU6050_ACCEL_FS_2          0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_ACCEL_FS_4          0x01
 MPU6050_ACCEL_FS_8          0x02
 MPU6050_ACCEL_FS_16         0x03
*/
    //A. GYRO SETTINGS
        // Set the full scale range of the GYROSCOPE (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above
        uint8_t FS_SEL = 0, LSB = 131.0;      
        mpu.setFullScaleGyroRange(FS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange(); delay(50); 
        Serial.print("GyroRange = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = LSB/(FS_SEL + 1);
               
     //B. ACCEL SETTINGS    
        // Set the full scale range of the ACCELEROMETER (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above     
        uint8_t AFS_SEL = 2;
        mpu.setFullScaleAccelRange(AFS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange(); delay(50); 
        Serial.print("AccelRange = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
      //C. Read High pass filter
         READ_AFS_SEL = mpu.getDHPFMode(); delay(50);     
        Serial.print("DHPFMode = ");
        Serial.println(READ_AFS_SEL);     

      //D. Read Low pass filter     
         mpu.setDLPFMode(2);
         READ_AFS_SEL = mpu.getDLPFMode(); delay(50);     
        Serial.print("DLPFMode = ");
        Serial.println(READ_AFS_SEL);

     //E. CALIBRATION
      calibrate_sensors();
}



void loop() 
{dt_loop = (micros()- lastTime);

        if (dt_loop >= 2500)
           {//Start 400 Hz
            lastTime = micros(); 
              loopIMU(dt_loop); //Extract IMU 
         }delayMicroseconds(2500);//End 400 Hz loop   
}//End Main Loop


void loopIMU(unsigned long  dt_imu)
{ float dt=dt_imu*0.000001f; //micro to seconds
  
////////////////////////////////////START IMU DATA WXTRACTION////////////////////////////////////////
const float RADIANS_TO_DEGREES = 57.295779579; //180/3.14159
const float DEGREES_TO_RADIANS = 0.0174532925; //180/3.14159
unsigned long t_now = millis();

        //Read magnetometer measures
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
        //STORE IMU DATA TO FLOAT VARIABLE
        double axx = ax ;//Serial.print(ax); Serial.print("\t");
        double ayy = ay ;//Serial.print(ay); Serial.print("\t");
        double azz = az ;//Serial.println(az);              
        //Normalise the measurements
        double R = sqrt(axx*axx + ayy*ayy + azz*azz); //Serial.println(R/16384);// R^2 = Rx^2 + Ry^2 + Rz^2  Pythegorian  

        float  Ax = axx/R ;//Serial.print(axx); Serial.print("\t");
        float  Ay = ayy/R ;//Serial.print(ayy); Serial.print("\t");
        float  Az = azz/R ;//Serial.println(azz);
                       
       
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - offset_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - offset_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - offset_z_gyro)/GYRO_FACTOR;     
        
        float accel_x = Ax; // - offset_x_accel;
        float accel_y = Ay; // - offset_y_accel;
        float accel_z = Az; // - offset_z_accel;
        

              
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        
        float gyro_angle_x = gyro_x*dt + last_x_angle;
        float gyro_angle_y = gyro_y*dt + last_y_angle;
              gyro_angle_z = gyro_z*dt + gyro_angle_z + 0.00000002;
        
        
                           
        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.95;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x; last_x_angle = angle_x; 
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y; last_y_angle = angle_y;
        float angle_z = gyro_angle_z;   //Accelerometer doesn't give z-angle
        angle_z = alpha*(angle_z + gyro_angle_z)+(1.0 - alpha)*gyro_angle_z; last_z_angle = angle_z;


      //Adjustment Here
       Acc_x= angle_x+0.00; 
       Acc_y= angle_y-0.50;                   
       Acc_z=-angle_z;      


////////////////////////////////////END IMU DATA USAGE AND MANUPULATION////////////////////////////////////////       
}



void calibrate_sensors() {
  int       readCounts = 2000; //Default 10

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < readCounts; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offset_x_gyro += gx;
    offset_y_gyro += gy;
    offset_z_gyro += gz;
    offset_x_accel += ax;
    offset_y_accel += ay;
    offset_z_accel += az;     
  }
  
  offset_x_gyro /= readCounts;
  offset_y_gyro /= readCounts;
  offset_z_gyro /= readCounts;
  offset_x_accel /= readCounts;
  offset_y_accel /= readCounts;
  offset_z_accel /= readCounts;

  
 /* 
  Serial.print("xg ");Serial.println(offset_x_gyro);
  Serial.print("yg ");Serial.println(offset_y_gyro);
  Serial.print("zg ");Serial.println(offset_z_gyro);
  Serial.print("xa ");Serial.println(offset_x_accel);
  Serial.print("ya ");Serial.println(offset_y_accel);
  Serial.print("za ");Serial.println(offset_z_accel);
*/
  delay(100);
}
