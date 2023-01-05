
// Arduino Code for the Up-Down Arm Lab

// Likely User Modified Variables ******************************
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3;


// End Likely User Modified Variables***************************

// For arduino and host interface, NO NEED TO MODIFY!
unsigned long transferDt = 20000; // usecs between host updates
unsigned long transferDt1 = 30000;
int angleSensorPin = A0;
int pwmVoltagePin = A1;
int motorVoltagePin = A2;
int motorOutputPin = 9;  // Do not change this!!

// Arduino-specific DAC values.
int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024;  //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 485; // 1024/2 = 512


// Control variables (set by host).
float direct = 80; // Direct motor command from host.
float desired;  // Desired value (speed or angle) from host.
float Kp = 0.7; // Feedback gains (proportional)
float ki = 7;
float kd = 6.5;
float proportional = 0;
float integral = 0;
float derivative = 0;
float oldError = 0;
int counter = 0;
float r = 0.999;
float pastError[pastSize];

float filtered_de = 0;
float prevFiltered_de = 0;

void setup() {
  // Set up inputs
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);
  
  // Set up PWM motor output pin, affects delay() function
  // Delay argument will be in 1/64 milliseconds.
  pinMode(motorOutputPin,OUTPUT);
  digitalWrite(motorOutputPin,LOW);
  //setPwmFrequency(motorOutputPin, 1); // Set pwm to approx 30khz.

  // Set for fastest serial transfer.
  Serial.begin(115200);
}


// Main code, runs repeatedly
void loop() {
  // Measure angle three times and sum, then by twelve. Averaging reduces noise.
  // Factor of four is ratio of adc and dac.
  float Vangle = float (analogRead(angleSensorPin) + analogRead(angleSensorPin) + analogRead(angleSensorPin) - 3*adcCenter)*2;
                        
  Vangle = map(Vangle, -150, 150, -90, 90);

  for(int i =pastSize-1; i>0; i--){
    pastError[i] = pastError[i-1];
  }
  
  // Compute the error, which is the difference between desired and measured angle. 
  float error = desired - Vangle;
  pastError[0] = error;
  proportional = Kp * error;

  if(integral < 200 && integral >= 0) integral += ki*error*0.001;
  if(integral > 200) integral = 180;
  if(integral < 0) integral = 0;

  filtered_de =(1-r)* prevFiltered_de + r*(error-pastError[pastSize-1]);
  filtered_de /= pastSize;
  derivative = kd * (filtered_de);
  prevFiltered_de = filtered_de;
  //oldError = error;

  // Compute and limit the motor output command.
  int motorCmd = int(proportional + integral + derivative + direct);
  motorCmd = min(max(motorCmd, 0), dacMax);
  analogWrite(motorOutputPin,motorCmd);


  Serial.print(error);
  Serial.print(",      ");
  Serial.print(filtered_de);
  Serial.print(",      ");
  Serial.print(proportional);
  Serial.print(",   ");
  Serial.print(integral);
  Serial.print(",   ");
  Serial.println(derivative);

  counter++;

}


//void setPwmFrequency(int pin, int divisor) {
//  byte mode;
//  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
//    switch(divisor) {
//      case 1: mode = 0x01; break;
//      case 8: mode = 0x02; break;
//      case 64: mode = 0x03; break;
//      case 256: mode = 0x04; break;
//      case 1024: mode = 0x05; break;
//      default: return;
//    }
//    if(pin == 5 || pin == 6) {
//      TCCR0B = TCCR0B & 0b11111000 | mode;
//    } else {
//        TCCR1B = TCCR1B & 0b11111000 | mode;
//    }
//  } else if(pin == 3 || pin == 11) {
//    switch(divisor) {
//      case 1: mode = 0x01; break;
//      case 8: mode = 0x02; break;
//      case 32: mode = 0x03; break;
//      case 64: mode = 0x04; break;
//      case 128: mode = 0x05; break;
//      case 256: mode = 0x06; break;
//      case 1024: mode = 0x7; break;
//      default: return;
//    }
//    TCCR2B = TCCR2B & 0b11111000 | mode;
//  }
//}
