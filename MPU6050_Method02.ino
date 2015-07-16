
//////////////////////////////
//  IMU Bank Orientation    //
//        -X  +X            //
//           |              //
//    -Y     |    -Y        //
//       ---------          //
//    +Y     |    +Y        //
//           |              //  
//        -X  +X            //
//       Pitch about Y      //
//       Rolls about X      //
//      Both Gyro & Acc     //
//////////////////////////////

#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define LED_PIN 13 


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[256]; // FIFO storage buffer, default value is 64
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
int16_t gyro[3];        // Inserted,not originally inclusive
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro0, gyro1, gyro2;
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//MPU Interrupt detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {mpuInterrupt = true;}


/*IMU STABILIZATION VARIABLES */
float Acc_x, Acc_y, Acc_z;
float Gyr_x, Gyr_y, Gyr_z, gyroX, gyroY, gyroZ;
float Rol_x, Pit_y, Yaw_z;
float  Mag_z; float Z; 



//Time Variable
volatile float lastTime=0, now=0;
volatile float timeChangeSec=0; 

void setup() 
{
//Serial Initialization
Serial.begin(115200); Serial2.begin(115200); delay(500);//give time for serial to initialized 
while (Serial.available() && Serial.read()); delay(500);// empty buffer 

/*********************START OF MPU 6050 CONFIG********************************/ 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        // If you are using an Arduino DUE, modify the variable TWI_CLOCK to 400000, defined in the file:      
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) - For Due, remove or comment the //TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize MPU device
    Serial.println(F("Initializing I2C devices..."));    
    mpu.initialize(); delay(500);//give time for serial to check 
    // verify device connection
    Serial.println(F("Testing device connections..."));    
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize(); delay(500);//give time for serial to check


    mpu.setXAccelOffset(-3507);
    mpu.setYAccelOffset(2604);
    mpu.setZAccelOffset(1010); 
   
    mpu.setXGyroOffset(23);
    mpu.setYGyroOffset(-50);
    mpu.setZGyroOffset(-5);


// Digital low pass filter configuration. 
// It also determines the internal sampling rate used by the device as shown in the table below.
// The accelerometer output rate is fixed at 1kHz. This means that for a Sample
// Rate greater than 1kHz, the same accelerometer sample may be output to the
// FIFO, DMP, and sensor registers more than once.
/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 
  mpu.setDLPFMode(3);
// Full-scale range of the gyro sensors:
// 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

// Full-scale accelerometer range.
// The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range

//mpu.setIntDataReadyEnabled(true); // enable data ready interrupt
*/

// make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
          }
/*******END OF MPU 6050 CONFIG********************************/  
}


void loop() 
{
  while(!mpuInterrupt && fifoCount < packetSize)
  {/*Do nothing while MPU is not working,This should be a VERY short period*/} 
  
timeChangeSec = (millis()- lastTime)/1000; now=millis();
    loopIMU();
    lastTime=now; 
}





/****************************************************************************/
/****************************MPU FUNCTIONS***********************************/
/****************************************************************************/
float emaX=0, emaY=0, emaZ=0, emaXg=0, emaYg=0, emaZg=0;
float gyroXp=0, gyroYp=0, gyroZp=0;
const float alpha = 0.40; //0 to 1
void loopIMU()
{ 

   if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

//START IMU DATA
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
        {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        }
        
     else if (mpuIntStatus & 0x02) 
        {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
   
            /******IMU READY TO EXTRACT DATA***********/
            //GET DATA functions

                    mpu.dmpGetQuaternion(&q, fifoBuffer);
                    mpu.dmpGetGravity(&gravity, &q);  
                 
                    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //(ypr[0] * 180/M_PI), (ypr[1] * 180/M_PI), (ypr[2] * 180/M_PI)
                    mpu.dmpGetGyro(gyro, fifoBuffer); //(gyro[0]), (gyro[1]), (gyro[2])  
                    
                    Acc_z=(ypr[0] * 180/M_PI); //Acc_z=zEMA(Acc_z);//Yaw
                    Acc_y=(ypr[1] * 180/M_PI); //Acc_y=yEMA(Acc_y);//Pitch  (-)FORWARD   (+)BACKWARD
                    Acc_x=(ypr[2] * 180/M_PI); //Acc_x=xEMA(Acc_x);//RollS  (+)ROLLRIRGT (-)ROLLLEFT
                                       
                    if       (Acc_x>30)  {Acc_x=30;}
                    else if  (Acc_x<-30)  {Acc_x=-30;}
                    else     {Acc_x=Acc_x;}
                    
                    if       (Acc_y>30)  {Acc_y=30;}
                    else if  (Acc_y<-30) {Acc_y=-30;}
                    else     {Acc_y=Acc_y;}  
                   
                   
                    if (abs(gyro[0])==1) gyro[0]=0;   
                    if (abs(gyro[1])==1) gyro[1]=0; 
                    if (abs(gyro[2])==1) gyro[2]=0;                   
                 
                    gyroX=gyro[0]; //Rolls Gyro + Rigt    -Left about X (See Axis top page)
                    gyroX=gxEMA(gyroX);
                    gyroY=-gyro[1]; //Pitch Gyro + Forward -Backward about Y (See Axis top page)                          
                    gyroY=gyEMA(gyroY);
                    gyroZ=gyro[2]; //Pitch Gyro -CW +CCW about Z (See Axis top page)                     
                    gyroZ=gzEMA(gyroZ);
                    
                    if       (gyroX>75.0)  {gyroX=gyroXp*1.50; gyroX=min(gyroX,75.00);}
                    else if  (gyroX<-75.0) {gyroX=gyroXp*1.50; gyroX=max(gyroX,-75.00);}
                    else     {gyroX=gyroX;} 
                    
                    if       (gyroY>75.00)  {gyroY=gyroYp*1.50; gyroY=min(gyroY,75.00);}
                    else if  (gyroY<-75.0)  {gyroY=gyroYp*1.50; gyroY=max(gyroY,-75.00);}
                    else     {gyroY=gyroY;}  
                    
                    if       (gyroZ>50)   {gyroZ=gyroZp*1.50; gyroZ=min(gyroZ,75.00);}
                    else if  (gyroZ<-50)  {gyroZ=gyroZp*1.50; gyroZ=max(gyroZ,-75.00);}
                    else     {gyroZ=gyroZ;} 
                    //Serial.print(gyro[0]);Serial.print("\t");Serial.print(gyroX);Serial.print("\t");Serial.print(gyro[1]);Serial.print("\t"); Serial.print(gyroY);Serial.print("\t");Serial.print(gyroZ);Serial.print("\t");
                    gyroXp=gyroX; gyroYp=gyroY; gyroZp=gyroZ;
                    //mpu.resetFIFO();
             }        
}






float xEMA(float new_value) {
  emaX += alpha*(new_value - emaX);
  return(emaX);
}

float yEMA(float new_value) {
  emaY += alpha*(new_value - emaY);
  return(emaY);
}

float zEMA(float new_value) {
  emaZ += alpha*(new_value - emaZ);
  return(emaZ);
}

float gxEMA(float new_value) {
  emaXg += alpha*(new_value - emaXg);
  return(emaXg);
}

float gyEMA(float new_value) {
  emaYg += alpha*(new_value - emaYg);
  return(emaYg);
}

float gzEMA(float new_value) {
  emaZg += alpha*(new_value - emaZg);
  return(emaZg);
}

