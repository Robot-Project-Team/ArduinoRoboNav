/* Wiring instruction of ICM 20948:
 * Vin -- 5V power supply 
 * GND -- GND
 * SDA -- A4
 * SCL -- A5
 */

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

#define USE_SPI       // Uncomment this to use SPI
//Analog input
#define   AnalogInput1 A0
#define   AnalogInput2 A1
#define   AnalogInput3 A2
#define   AnalogInput4 A3
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif


double Vout1 = 0;
double Vout2 = 0;
double Vout3 = 0;
double Vout4 = 0;
bool switch_power = false;
String Cpp_input;
bool toggle = false;
bool right_toggle = false;

long int direct;
double currentAngleY = 0;

long int printVal;
long int printVal2;

bool bidire_check = false;
bool available_check = false;
long int Buffer;

bool check_start = true;
double offset_val = 0;
double accelX, accelY, accelZ;
double x,y,z;

int led = 13;
bool move_forward = true;

void setup()
{
  pinMode(AnalogInput3, INPUT);
  pinMode(AnalogInput4, INPUT);
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  Serial.setTimeout(500);
  unsigned long currentTime = 0;
  
  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif
  
    //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  
    bool initialized = false;
    while (!initialized)
    {
  
  #ifdef USE_SPI
      myICM.begin(CS_PIN, SPI_PORT);
  #else
      myICM.begin(WIRE_PORT, AD0_VAL);
  #endif
  
      Serial.print(F("Initialization of the sensor returned: "));
      Serial.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        Serial.println("Trying again...");
        delay(500);
      }
      else
      {
        initialized = true;
      }
    }
  
    // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  Serial.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false); // Huaxing set this to true
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(myICM.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }

  Serial.println();
  Serial.println(F("Configuration complete!"));

    while (!Serial)
    {
    };
}


void loop()
{ 
    if(Serial.available()>0){
      Cpp_input = Serial.readString();
      if(Cpp_input.equals("T")){
        move_forward = !move_forward;
      
      }
    }  
  
  if (myICM.dataReady())
  {

    
      Vout3 = analogRead(AnalogInput3);
      Vout4 = analogRead(AnalogInput4);
      long int f3=0;
      long int f4=0;
      Vout3 = PrintVoltage_mV(Vout3);
      Vout4 = PrintVoltage_mV(Vout4);

      if(Vout3 > 0){ 
          f3 = AproxForce(Vout3);
      }
      if(Vout4 > 0){ 
          f4 = AproxForce(Vout4);
      }
      if(f3 > 20000){
          f3 = 20000;
      }
      if(f4 > 20000){
          f4 = 20000;
      }
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    
    accelX = myICM.accX();
    accelY = myICM.accY();
    accelZ = myICM.accZ();
   
    y = 180-(RAD_TO_DEG * (atan2(-accelX, -accelZ)+PI));  
    if(check_start){
      offset_val = y;
      check_start = false;
    }  
    y -= offset_val; // subtract the offset (the original value)
  if(y >1.5  && (f3 <= 2 && f4 <= 2)){ // forward

    if(move_forward){
      direct = 200001;
    }else if(!move_forward){
      direct = 200002;
    }
    
    available_check = true;
    if((y) > 7){
      y = 7;
    }
    y *= 100; // multiplied by 100 since the map() function uses integer math.
    printVal = map(y, 0, 700, 0, 100);
  }
//  else if(y < -1.5 && (f3 <= 2 && f4 <= 2)) { // backward
//    direct = 200002; 
//    available_check = true;
//    if((y) < -7){
//      y = -7;
//    }
//    y *= 100; // multiplied by 100 since the map() function uses integer math.
//    printVal = map((-1.0*y), 0, 700, 0, 100);
//  }
  else if(y > 1.5 && f3 > 2){ // F&L

    if(move_forward){
      direct = 200005;
    }else if(!move_forward){
      direct = 200006;
    }
    bidire_check = true;
    available_check = true;
    if((y) > 7){
      y = 7;
    }
    y *= 100; // multiplied by 100 since the map() function uses integer math.
    printVal = map(y, 0, 700, 0, 100);
    printVal2 = map(f3, 0, 20000, 0, 100);
  }
  else if(y > 1.5 && f4 > 2){ // F&R
    if(move_forward){
      direct = 200007;
    }else if(!move_forward){
      direct = 200008;
    }
    bidire_check = true;
    available_check = true;
    if((y) > 7){
      y = 7;
    }
    y *= 100; // multiplied by 100 since the map() function uses integer math.
    printVal = map(y, 0, 700, 0, 100);
    printVal2 = map(f4, 0, 20000, 0, 100);
  }
  else if(y < -1.5 && f3 > 2){ // B&L
    direct = 200006;
    bidire_check = true;
    available_check = true;
    if((y) < -7){
      y = 7;
    }
    y *= 100; // multiplied by 100 since the map() function uses integer math.
    printVal = map(y, 0, 700, 0, 100);
    printVal2 = map(f3, 0, 20000, 0, 100);
  }
  else if(y < -1.5 && f4 > 2){ // B&R
    direct = 200008;
    bidire_check = true;
    available_check = true;
    if((y) <-7){
      y = 7;
    }
    y *= 100; // multiplied by 100 since the map() function uses integer math.
    printVal = map(y, 0, 700, 0, 100);
    printVal2 = map(f4, 0, 20000, 0, 100);
  }
  else if(f3 > 2){ // left
    direct = 200003;//1952867692;
    available_check = true;
    printVal = map(f3, 0, 20000, 0, 100);
  }
  else if(f4 > 2){ //right
    direct = 200004;
    available_check = true;
    printVal = map(f4, 0, 20000, 0, 100);
  }
  if(available_check == true){
    byte Buffer2[] = {
      byte(direct & 0xff),
      byte(direct >> 8 & 0xff),
      byte(direct >> 16 & 0xff),
      byte(direct >> 24 & 0xff)
    };
    byte Buffer1[] = {
      byte(printVal & 0xff),
      byte(printVal >> 8 & 0xff),
      byte(printVal >> 16 & 0xff),
      byte(printVal >> 24 & 0xff)
    };
    Serial.write(Buffer2, 4);
    Serial.write(Buffer1,4);
  }

  if(bidire_check == true){
    byte Buffer[] = {
      byte(printVal2 & 0xff),
      byte(printVal2 >> 8 & 0xff),
      byte(printVal2 >> 16 & 0xff),
      byte(printVal2 >> 24 & 0xff)
    };
    Serial.write(Buffer,4);
  }
  if(bidire_check){
    delay(80);
  }else{
    delay(60);
    }
    bidire_check = false;
    available_check = false;
  }

  
  else{
    Serial.println("Waiting for data");
  }
}


double PrintVoltage_mV(double vout){
  double Vout = 0;
  // map the reading of the sensors from 0 to 5V
  Vout = map(vout, 0, 1023, 0, 5000);
  //Serial.print("Voltage = ");
  //Serial.println(Vout/1000);
  return Vout;
}

long int AproxForce(double fsrVoltage){
  unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
  unsigned long fsrConductance; 
  long fsrForce;       // Finally, the resistance converted to force
  double x, y; // x is fsr resistance, and y is force in gram
  // so FSR = ((Vcc - V) * R) / V        yay math!
  fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
  fsrResistance *= 10000;                // 10K resistor
  fsrResistance /= fsrVoltage;

  x = log10((fsrResistance*10)/1000); // divide fsr by 1k because
  y = (-1.4182 * x) +3.5644;
  y = pow(10,y)*10;
  y *= 100;
  long int output = y;
  return output;
    
}

#ifdef USE_SPI
double YprintScaledAGMT(ICM_20948_SPI *sensor, double currentAngle, unsigned long deltaTime)
{
#else
double YprintScaledAGMT(ICM_20948_I2C *sensor, double currentAngle, unsigned long deltaTime)
{
#endif
  double deltaAngle = (sensor->gyrY()) * deltaTime *0.001;
  currentAngle = currentAngle + deltaAngle;
  return currentAngle;
}
