
// Arduino Code for the Up-Down Arm Lab

// Likely User Modified Variables ******************************
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3;

// End Likely User Modified Variables***************************

// For arduino and host interface, NO NEED TO MODIFY!
unsigned long transferDt = 20000; // usecs between host updates
int angleSensorPin = A0;
int pwmVoltagePin = A1;
int motorVoltagePin = A2;
int motorOutputPin = 9;  // Do not change this!!

float filtered_de = 0;
float prevFiltered_de = 0;
float derivativeError = 0;

// Arduino-specific DAC values.
int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024;  //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 485; // 1024/2 = 512

// Control variables (set by host).
float direct = 0; // Direct motor command from host.
float desired = 0;  // Desired value (speed or angle) from host.
float integral = 0;
float Kp = 0.8; // Feedback gains (proportional)
float Kd = 5; // Feedback gains (delta)
float ki = 0;
int counter = 0;

// Variables to reset every time loop restarts.
int loop_counter;
int numSkip;  // Number of loops to skip between host updates.
String inputString = ""; //holds received serial data.
boolean stringComplete; // String received flag.

//data
float pastError[pastSize];


// Setup sets up pwm frequency (30khz), initializes past values.
int loopPin = 13;
boolean loopFlag;

void setup() {
  // Set up inputs
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);
  
  // Set up PWM motor output pin, affects delay() function
  // Delay argument will be in 1/64 milliseconds.
  pinMode(motorOutputPin,OUTPUT);
  digitalWrite(motorOutputPin,LOW);
  setPwmFrequency(motorOutputPin, 1); // Set pwm to approx 30khz.

  // Set for fastest serial transfer.
  Serial.begin(115200);
}


// Main code, runs repeatedly
void loop() {
  float Vangle = float (analogRead(angleSensorPin)
                        + analogRead(angleSensorPin) + analogRead(angleSensorPin) - 3*adcCenter)/3.0;
                        
  Vangle = map(Vangle, -25, 25, -90, 90);
  
  // Compute the error, which is the difference between desired and measured angle. 
  for(int i =pastSize-1; i>0; i--){
    pastError[i] = pastError[i-1];
  }
  float error = desired - Vangle;
  pastError[0] = error;
  derivativeError = pastError[0] - pastError[pastSize-1];
  filtered_de = 0.7*(prevFiltered_de) + 0.3*derivativeError;
  prevFiltered_de = filtered_de;
  float delta = (filtered_de)/(pastSize*(float(deltaT)*1e-6));

  integral += ki*error*0.001;

  if(counter % 100 == 0){
    Serial.print(error);
    Serial.print(",     ");
    Serial.print(Kd * delta);
    Serial.print(",     ");
    Serial.println(integral);
  }
  counter++;

  // Compute and limit the motor output command.
  int motorCmd = int(Kp *error + Kd*delta + integral + direct);
  motorCmd = min(max(motorCmd, 0), dacMax);
  analogWrite(motorOutputPin,motorCmd);
}


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
        TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
