int led = 13;

#include <math.h>
//Analog input
#define   AnalogInput1 A0
#define   AnalogInput2 A1
#define   AnalogInput3 A2
#define   AnalogInput4 A3
double Vout1 = 0;
double Vout2 = 0;
double Vout3 = 0;
double Vout4 = 0;
bool switch_power = false;
String Cpp_input;
bool toggle = false;
bool right_toggle = false;

long int direct;

void setup(){
  pinMode(AnalogInput1, INPUT);
  pinMode(AnalogInput2, INPUT);
  pinMode(AnalogInput3, INPUT);
  pinMode(AnalogInput4, INPUT);
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  Serial.setTimeout(500);
  
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

void loop()
{
//  if(Serial.available()>0){
//    Cpp_input = Serial.readString();
//    
//  }
//  else{
//    return;
//    }
//  if(Cpp_input.equals("ON")){
        //get analog readings from four sensors
        Vout1 = analogRead(AnalogInput1);
        Vout2 = analogRead(AnalogInput2);
        Vout3 = analogRead(AnalogInput3);
        Vout4 = analogRead(AnalogInput4);
        long int f1=0;
        long int f2=0;
        long int f3=0;
        long int f4=0;
        Vout1 = PrintVoltage_mV(Vout1);
        Vout2 = PrintVoltage_mV(Vout2);
        Vout3 = PrintVoltage_mV(Vout3);
        Vout4 = PrintVoltage_mV(Vout4);
      if(Vout1 > 0){ 
          f1 = AproxForce(Vout1);
      }
      if(Vout2 > 0){ 
          f2 = AproxForce(Vout2);
      }
      if(Vout3 > 0){ 
          f3 = AproxForce(Vout3);
      }
      if(Vout4 > 0){ 
          f4 = AproxForce(Vout4);
      }
      if(f1 > 200000){
          f1 = 200000;
      }
      if(f2 > 200000){
          f2 = 200000;
      }
      if(f3 > 200000){
          f3 = 200000;
      }
      if(f4 > 200000){
          f4 = 200000;
      }

      if(f1 > 2 && (f2 <= 2 && f3 <= 2 && f4 <= 2)){ // forward
          f1 = map(f1, 0, 200000, 0, 600);
          byte Buffer[] = {
          byte(f1 & 0xff),
          byte(f1 >> 8 & 0xff),
          byte(f1 >> 16 & 0xff),
          byte(f1 >> 24 & 0xff)
          };
          long int direct = 200001;//1801675106;
          byte Buffer2[] = {
          byte(direct & 0xff),
          byte(direct >> 8 & 0xff),
          byte(direct >> 16 & 0xff),
          byte(direct >> 24 & 0xff)
          };
          Serial.write(Buffer2, 4);
          Serial.write(Buffer,4);
       }

       if(f2 > 2 && (f1 <= 2 && f3 <= 2 && f4 <= 2)){ //backward only
          f2 = map(f2, 0, 200000, 0, 600);
          byte Buffer[] = {
          byte(f2 & 0xff),
          byte(f2 >> 8 & 0xff),
          byte(f2 >> 16 & 0xff),
          byte(f2 >> 24 & 0xff)
          };
          long int direct = 200002;//1801675106;
          byte Buffer2[] = {
          byte(direct & 0xff),
          byte(direct >> 8 & 0xff),
          byte(direct >> 16 & 0xff),
          byte(direct >> 24 & 0xff)
          };
          Serial.write(Buffer2, 4);
          Serial.write(Buffer,4);
       }

       if(f3 > 2){ // left
          if((f1>2)){//F & L
            direct = 200005;
            f1 = map(f1, 0, 200000, 0, 600);
            f3 = map(f3, 0, 200000, 0, 600);
            byte Buffer2[] = {
            byte(direct & 0xff),
            byte(direct >> 8 & 0xff),
            byte(direct >> 16 & 0xff),
            byte(direct >> 24 & 0xff)
            };
            byte Buffer1[] = {
            byte(f1 & 0xff),
            byte(f1 >> 8 & 0xff),
            byte(f1 >> 16 & 0xff),
            byte(f1 >> 24 & 0xff)
            };
            byte Buffer[] = {
            byte(f3 & 0xff),
            byte(f3 >> 8 & 0xff),
            byte(f3 >> 16 & 0xff),
            byte(f3 >> 24 & 0xff)
            };
            Serial.write(Buffer2, 4);
            Serial.write(Buffer1, 4);
            Serial.write(Buffer,4);
        }
        else if(f2>2){ //B & L
            direct = 200006;
            f2 = map(f2, 0, 200000, 0, 600);
            f3 = map(f3, 0, 200000, 0, 600);
            byte Buffer2[] = {
            byte(direct & 0xff),
            byte(direct >> 8 & 0xff),
            byte(direct >> 16 & 0xff),
            byte(direct >> 24 & 0xff)
            };
            byte Buffer1[] = {
            byte(f2 & 0xff),
            byte(f2 >> 8 & 0xff),
            byte(f2 >> 16 & 0xff),
            byte(f2 >> 24 & 0xff)
            };
            byte Buffer[] = {
            byte(f3 & 0xff),
            byte(f3 >> 8 & 0xff),
            byte(f3 >> 16 & 0xff),
            byte(f3 >> 24 & 0xff)
            };
            Serial.write(Buffer2, 4);
            Serial.write(Buffer1, 4);
            Serial.write(Buffer,4);
          }
        else{ // left only
            direct = 200003;//1952867692;
            f3 = map(f3, 0, 200000, 0, 600);
            byte Buffer2[] = {
            byte(direct & 0xff),
            byte(direct >> 8 & 0xff),
            byte(direct >> 16 & 0xff),
            byte(direct >> 24 & 0xff)
            };
            byte Buffer[] = {
            byte(f3 & 0xff),
            byte(f3 >> 8 & 0xff),
            byte(f3 >> 16 & 0xff),
            byte(f3 >> 24 & 0xff)
            };
            Serial.write(Buffer2, 4);
            Serial.write(Buffer,4);
        }
      } 

      if(f4 > 2){  //right
        if((f1>2)){//F & R
        direct = 200007;
        f1 = map(f1, 0, 200000, 0, 600);
        f4 = map(f4, 0, 200000, 0, 600);
        byte Buffer2[] = {
        byte(direct & 0xff),
        byte(direct >> 8 & 0xff),
        byte(direct >> 16 & 0xff),
        byte(direct >> 24 & 0xff)
        };
        byte Buffer1[] = {
        byte(f1 & 0xff),
        byte(f1 >> 8 & 0xff),
        byte(f1 >> 16 & 0xff),
        byte(f1 >> 24 & 0xff)
        };
        byte Buffer[] = {
        byte(f4 & 0xff),
        byte(f4 >> 8 & 0xff),
        byte(f4 >> 16 & 0xff),
        byte(f4 >> 24 & 0xff)
        };
        Serial.write(Buffer2, 4);
        Serial.write(Buffer1, 4);
        Serial.write(Buffer,4);
        }

        else if(f2>2){ //B & R
          direct = 200008;
          f2 = map(f2, 0, 200000, 0, 600);
          f4 = map(f4, 0, 200000, 0, 600);
          byte Buffer2[] = {
          byte(direct & 0xff),
          byte(direct >> 8 & 0xff),
          byte(direct >> 16 & 0xff),
          byte(direct >> 24 & 0xff)
          };
          byte Buffer1[] = {
          byte(f2 & 0xff),
          byte(f2 >> 8 & 0xff),
          byte(f2 >> 16 & 0xff),
          byte(f2 >> 24 & 0xff)
          };
          byte Buffer[] = {
          byte(f4 & 0xff),
          byte(f4 >> 8 & 0xff),
          byte(f4 >> 16 & 0xff),
          byte(f4 >> 24 & 0xff)
          };
          Serial.write(Buffer2, 4);
          Serial.write(Buffer1, 4);
          Serial.write(Buffer,4);

        }
        else{ //right only
           direct = 200004;
          f4 = map(f4, 0, 200000, 0, 600);
          byte Buffer2[] = {
          byte(direct & 0xff),
          byte(direct >> 8 & 0xff),
          byte(direct >> 16 & 0xff),
          byte(direct >> 24 & 0xff)
          };
          byte Buffer[] = {
          byte(f4 & 0xff),
          byte(f4 >> 8 & 0xff),
          byte(f4 >> 16 & 0xff),
          byte(f4 >> 24 & 0xff)
          };
          Serial.write(Buffer2, 4);
          Serial.write(Buffer,4);
       }
     }
        delay(60);

  //}

}
