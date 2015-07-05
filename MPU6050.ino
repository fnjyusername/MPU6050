Enter file contents here
#STEP on MPU6050 6DOF

Identify and define the I2C Address in the #define, there are two possible address depending on the board's pcb
wiring circuit.
```
#define 0x68  //When AD0 is logic LOW on the PCB
#define 0x68  //When AD0 is logic HIGH on the PCB
```
