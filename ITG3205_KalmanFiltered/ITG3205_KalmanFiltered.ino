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

#define N (3)
float I[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float QVal=0.1;
float RVal=1;
float X[N][1]={ {1},{1},{1} };
float Y[N][1]={ {0},{0},{0} };
float Q[N][N]={ {QVal,0,0},{0,QVal,0},{0,0,QVal}};
float R[N][N]={ {RVal,0,0},{0,RVal,0},{0,0,RVal} };
float A[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float H[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float P[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float S[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float K[N][N]={ {1,0,0},{0,1,0},{0,0,1} };
float Z[N][1];
float YNew[N][1];
float KAngle[N][1]={ {0},{0},{0} };

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
 Z[0][0]=gyro[0];
 Z[1][0]=gyro[1];
 Z[2][0]=gyro[2];
 Z[3][0]=gyro[3];
 
 Matrix.Multiply((float*)A,(float*)X,N,N,1,(float*)X);      //X=AX
 //Matrix.Print((float*)X,N,1,"X");
 Matrix.Add((float*) P, (float*) Q, N, N, (float*) P);      //P=APA'+Q  Skip APA' since A is identity and APA'=1*P*1
 //Matrix.Print((float*)P,N,N,"P");
 Matrix.Subtract((float*) Z, (float*) X, N, 1, (float*) Y); //Y=Z-HX   Skip HX since H is identity
 //Matrix.Print((float*)Y,N,1,"Y");
 Matrix.Add((float*) P, (float*) R, N, N, (float*) S);      //S=HPH'+R Skip HPH' since H is identity
 //Matrix.Print((float*)S,N,N,"S");
 Matrix.Invert((float*)S,N);
 //Matrix.Print((float*)S,N,N,"S");
 Matrix.Multiply((float*)P,(float*)S,N,N,N,(float*)K);      //K=PH'S^-1
 //Matrix.Print((float*)K,N,N,"K");
 Matrix.Multiply((float*)K,(float*)Y,N,N,1,(float*)YNew);      //Save KY in Y for last step        
 //Matrix.Print((float*)YNew,N,1,"Y");
 Matrix.Subtract((float*) I, (float*) K, N, N, (float*) K); //Convert K to (I-K) for next step
 
 Matrix.Multiply((float*)K,(float*)P,N,N,N,(float*)P);      //P=(I-KH)P   
 
 Matrix.Add((float*) X, (float*) YNew, N, 1, (float*) X);      //X=X+KY
 //Matrix.Print((float*)X,N,1,"X");
 KAngle[0][0]+=X[0][0]/270;
 KAngle[1][0]+=X[1][0]/270;
 KAngle[2][0]+=X[2][0]/270;
 Serial.print(millis());
 Serial.print(" ");
 Serial.print(KAngle[0][0]);
 Serial.print(" ");
 Serial.print(KAngle[1][0]);
 Serial.print(" ");
 Serial.print(KAngle[2][0]);
 Serial.println(" "); 
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
