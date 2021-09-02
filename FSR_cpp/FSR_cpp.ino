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

void setup(){
  pinMode(AnalogInput1, INPUT);
  pinMode(AnalogInput2, INPUT);
  pinMode(AnalogInput3, INPUT);
  pinMode(AnalogInput4, INPUT);
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  
  }

double PrintVoltage_mV(double vout){
  double Vout = 0;
  // map the reading of the sensors from 0 to 5V
    Vout = map(vout, 0, 1023, 0, 5000);
    //Serial.print("Voltage = ");
    //Serial.println(Vout/1000);
    return Vout;
}

double AproxForce(double fsrVoltage){
  unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
  unsigned long fsrConductance; 
  long fsrForce;       // Finally, the resistance converted to force
  double x, y; // x is fsr resistance, and y is force in gram
  // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    //Serial.print("FSR resistance in ohms = ");
    //Serial.println(fsrResistance); // x axis

  // calculate the force
  // convert the equation of fsr and force to logrithmic scale
  // according to the force curve in datasheet
  x = log10((fsrResistance*10)/1000); // divide fsr by 1k because
  y = (-1.4182 * x) +3.5644;
  y = pow(10,y)*10;
  return y;
    
}

void loop()
{
  if(Serial.available()>0){
    Cpp_input = Serial.readStringUntil('\n');
    if(Cpp_input.equals("ON")){
      //if((switch_power == false)){
      digitalWrite(led, HIGH);  
      Serial.println("Loop is ON!");
      switch_power = true;
      //}
    }  
    else if(Cpp_input.equals("OFF")){
      digitalWrite(led, LOW);
      Serial.println("Loop is OFF!");
      switch_power = false;
      }
  }
  if(switch_power == true){
        //get analog readings from four sensors
        Vout1 = analogRead(AnalogInput1);
        Vout2 = analogRead(AnalogInput2);
        Vout3 = analogRead(AnalogInput3);
        Vout4 = analogRead(AnalogInput4);
        double f1, f2, f3, f4;
      Vout1 = PrintVoltage_mV(Vout1);
      Vout2 = PrintVoltage_mV(Vout2);
      Vout3 = PrintVoltage_mV(Vout3);
      Vout4 = PrintVoltage_mV(Vout4);
//        if(Vout1 > 0){
//          
//          f1 = AproxForce(Vout1);
//          if(f1 > 0.02){
//              Serial.println("-----------------------");
//              Serial.print("Front sensor is activated!\n"); 
//              Serial.print("Force (g) = ");  
//              Serial.println(f1);
//            }
//
//        }
//      
//        if(Vout2 > 0){ 
//          
//          f2 = AproxForce(Vout2);
//          if(f2 > 0.02){
//              Serial.println("-----------------------");
//              Serial.print("Back sensor is activated!\n"); 
//              Serial.print("Force (g) = ");  
//              Serial.println(f2);
//            }
//        }
      
        if(Vout3 > 0){
          
          f3 = AproxForce(Vout3);
          if(f3 > 0.02){
              Serial.println("-----------------------");
              Serial.print("Left sensor is activated!\n"); 
              Serial.print("Force (g) = ");  
              Serial.println(f3);
            }
        }
      
        if(Vout4 > 0){
          
          f4 = AproxForce(Vout4);
          if(f4 > 0.02){
              Serial.println("-----------------------");
              Serial.print("Right sensor is activated!\n"); 
              Serial.print("Force (g) = ");  
              Serial.println(f4);
            }
        }
  delay(500);
  }

}
