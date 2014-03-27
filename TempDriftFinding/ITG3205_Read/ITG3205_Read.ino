#include <Wire.h> // I2C library, gyroscope
#include <MatrixMath.h>
// Gyroscope ITG3200 
#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
//#define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69  
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z
// offsets are chip specific. 
//initializes the gyroscope

 //
void setup()
{
 Serial.begin(115200);
 Wire.begin();
 initGyro();
}
//
void loop()
{
 int gyro[4];
 
 getGyroscopeData(gyro);
 
 printData(gyro);
 
 delay(5); 
}

void initGyro()
{
 /*****************************************
 * ITG 3200
 * power management set to:
 * clock select = internal oscillator
 * no reset, no sleep mode
 * no standby mode
 * sample rate to = 125Hz
 * parameter to +/- 2000 degrees/sec
 * low pass filter = 5Hz
 * no interrupt
 ******************************************/
 writeTo(GYRO, G_PWR_MGM, 0x00);
 writeTo(GYRO, G_SMPLRT_DIV, 0x04); // Internal Sample Rate/(04h +1) = 1kHz/5 = 5mS
 writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz Internal Sample,  5Hz LPF ,  0001 1101
 writeTo(GYRO, G_INT_CFG, 0x00);
}
void getGyroscopeData(int * result)
{
 /**************************************
 Gyro ITG-3200 I2C
 registers:
 temp MSB = 1B, temp LSB = 1C
 x axis MSB = 1D, x axis LSB = 1E
 y axis MSB = 1F, y axis LSB = 20
 z axis MSB = 21, z axis LSB = 22
 *************************************/
 int regAddress = 0x1B;
 int temp, x, y, z;
 byte buff[G_TO_READ];
 readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
 result[3] = ((buff[0] << 8) | buff[1]); // temperature
 result[0] = ((buff[2] << 8) | buff[3]) - result[3]/500 + 22;
 result[1] = ((buff[4] << 8) | buff[5]) - result[3]/90 - 145;
 result[2] = ((buff[6] << 8) | buff[7]) - 6;
 }

void printData(int * data)
{
 //Serial.print(" X=");
 Serial.print(data[0]);
 Serial.print(" ");
 //Serial.print(" Y=");
 Serial.print(data[1]);
 Serial.print(" ");
//Serial.print(" Z=");
 Serial.print(data[2]);
 Serial.print(" ");
 //Serial.print(" F=");
 Serial.print(data[3]);
 //Serial.println("C");
 Serial.println(" "); 
}

//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on ACC in to buff array
 void readFrom(int DEVICE, byte address, int num, byte buff[]) {
 Wire.beginTransmission(DEVICE); //start transmission to ACC 
 Wire.write(address);        //sends address to read from
 Wire.endTransmission(); //end transmission
 
 Wire.beginTransmission(DEVICE); //start transmission to ACC
 Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
 
 int i = 0;
 while(Wire.available())    //ACC may send less than requested (abnormal)
 { 
   buff[i] = Wire.read(); // receive a byte
   i++;
 }
 Wire.endTransmission(); //end transmission
}
