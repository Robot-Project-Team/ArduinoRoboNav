/*ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data*/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define USE_SPI       // Uncomment this to use SPI


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

double currentAngleX = 0;
double currentAngleY = 0;
int count_duplicate = 0;
double check_duplicate = 0;
bool check_start = true;
double offset_valX = 0;
double offset_valY = 0;
double offset_valZ = 0;
double offset_mag_x, offset_mag_y, offset_mag_z;
double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;
double magnXread, magnYread, magnZread;
double x,y,z,yaw;
double magnX, magnY, magnZ, magNorm;
double a, b; //formula for yaw

unsigned long currentTime = millis();
unsigned long deltaTime = 0;

void setup()
{
  unsigned long currentTime = 0;
  Serial.begin(115200);
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
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true); // Huaxing set this to true
  ICM_20948_Status_e magDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Mag, true);
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(myICM.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status == ICM_20948_Stat_Ok)
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
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    
    accelX = myICM.accX();
    accelY = myICM.accY();
    accelZ = myICM.accZ();
    gyroX = myICM.gyrX();
    gyroY = myICM.gyrY();
    gyroZ = myICM.gyrZ();
    magnXread = myICM.magX();
    magnYread = myICM.magY();
    magnZread = myICM.magZ();

   
    x = 180-(RAD_TO_DEG * (atan2(-accelY, -accelZ)+PI)); 
    y = 180-(RAD_TO_DEG * (atan2(-accelX, -accelZ)+PI)); 
    z = 180-(RAD_TO_DEG * (atan2(-accelX, -accelY)+PI));
    magNorm = sqrt((magnXread*magnXread)+(magnYread*magnYread)+(magnZread*magnZread));
    magnXread /= magNorm;
    magnYread /= magNorm;
    magnZread /= magNorm;
    magnX = magnXread * cos(y-offset_valY) + magnYread * sin(x-offset_valX)*sin(y-offset_valY) + magnZread * cos(x-offset_valX) * sin(y-offset_valY);
    magnY = -magnYread * cos(x-offset_valX) + magnZread * sin(x-offset_valX);
    yaw = 180 * atan2(-magnY, magnX)/PI;
    
    if(check_start){
      offset_valX = x;
      offset_valY = y;
      offset_valZ = z;
//      offset_mag_x = magnXread;
//      offset_mag_y = magnYread;
//      offset_mag_z = magnZread;
//      a = atan2(offset_mag_z, offset_mag_x);
      check_start = false;
    }  
//    b = atan2(magnZread, magnXread);
//    yaw = (a - b) * 180 /PI;
    //magnX = atan2(magnZ, magnX);
    Serial.print("roll: ");
    Serial.print(x-offset_valX);
    Serial.print(" ");
    Serial.print("pitch: ");
    Serial.print(y-offset_valY);
    Serial.print(" ");
    Serial.print("yaw: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.println();
//    Serial.print("X: ");
//    Serial.print(gyroX);
//    Serial.print(" ");
//    Serial.print("y: ");
//    Serial.print(gyroY);
//    Serial.print(" ");
//    Serial.print("z: ");
//    Serial.print(gyroZ);
//    Serial.print(" ");
//    Serial.println();
    delay(100);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}

void printFormattedFloat(double val, uint8_t leading, uint8_t decimals)
{
  double aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

#ifdef USE_SPI
double XprintScaledAGMT(ICM_20948_SPI *sensor, double currentAngle, unsigned long deltaTime)
{
#else
double XprintScaledAGMT(ICM_20948_I2C *sensor, double currentAngle, unsigned long deltaTime)
{
#endif
  Serial.print("Gyr (DPS) [ ");
  
  double deltaAngle = (sensor->gyrX()) * deltaTime*0.001;
  currentAngle = currentAngle + deltaAngle;
  printFormattedFloat(currentAngle, 5, 2);
  Serial.print(", ");

  return currentAngle;
}

#ifdef USE_SPI
double YprintScaledAGMT(ICM_20948_SPI *sensor, double currentAngle, unsigned long deltaTime)
{
#else
double YprintScaledAGMT(ICM_20948_I2C *sensor, double currentAngle, unsigned long deltaTime)
{
#endif
  double gyro_rate = sensor->gyrY();
  if((gyro_rate >= -5) && (gyro_rate <= 5)){
    gyro_rate = 0;
  }
  double deltaAngle = gyro_rate * deltaTime *0.001;
  currentAngle = currentAngle + deltaAngle;
  printFormattedFloat(currentAngle, 5, 2);
  //Serial.print(" ]");
  Serial.println();
  return currentAngle;
}


unsigned long CalculateDeltaTime(unsigned long oldTime){
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - oldTime;
  return deltaTime;
}
