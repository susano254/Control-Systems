
// Teensy Code for the Motor Speed Control Lab

// Likely User Modified Variables ******************************
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3;

// End Likely User Modified Variables***************************

// For teensy and host interface, NO NEED TO MODIFY!
unsigned long transferDt = 20000; // usecs between host updates
int angleSensorPin = A9;
int pwmVoltagePin = A0;
int motorVoltagePin = A1;
int motorOutputPin = 4;  // Do not change this!!
int monitorPin=2;

// Arduino-specific DAC values.
int dacRes = 12;
int dacMax = 4095; // Teensy dac is 12 bits.
int adcRes=14;
int adcMax = 16383;  //Teensy ADC is 14 bits.
int adcCenter = 8192; // adcMax/2.


bool first_time = false;
String config_message  = "&A~Desired~5&C&S~K_P~P~0~20~0.05&S~K_D~D~0~10~0.05&S~K_I~I~0~20~0.05&S~SumMax~S~0~5000~1&S~Direct~O~0~260~0.01&S~Desired~A~-130~130~1&T~ArmAngle~F4~-130~130&T~Error~F4~-150~150&T~Delta~F4~-1000~1000&T~Sum~F4~-1000~1000&T~MotorCmd~F4~0~260&H~4&";

// Control variables (set by host).
float direct; // Direct motor command from host.
float desired;  // Desired value (speed or angle) from host.
float Kp; // Feedback gains (proportional)
float Kd; // Feedback gains (delta)
float Ki; // Feedback gains (sum)
float maxSum;

// Variables to reset every time loop restarts.
int loopCounter;  // Number of passes through the loop.
int numSkip;  // Number of loops to skip between host updates.

unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;
char buf[60]; 

//data
float pastError[pastSize];
float sum;


elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.

void setup() {
  Serial.begin(115200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);

    // Set up output
  analogWriteResolution(dacRes);
  pinMode(motorOutputPin, OUTPUT);
  analogWriteFrequency(motorOutputPin, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutputPin, LOW);

  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);

  // Number of loops between transfers to host.
  numSkip = max(int((transferDt+100)/deltaT-1),0); 
  sum = 0;
}


// Main code, runs repeatedly
void loop() {
   // Reinitializes or updates from sliders on GUI.
  startup();
  
   // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag);
  
  // User modifiable code between stars!!
  /***********************************************/
  // Measure angle three times and sum, then by twelve. Averaging reduces noise.
  // Factor of four is ratio of adc and dac.
  float Vangle = float (analogRead(angleSensorPin)
                        + analogRead(angleSensorPin) + analogRead(angleSensorPin) - 3*adcCenter)/12.0;
  Vangle = Vangle/16.0;  // To be compatble with Arduino.
  
  for(int i =pastSize-1; i>0; i--){
    pastError[i] = pastError[i-1];
  }
   // Compute the error, which is the difference between desired and measured angle. 
  float error = desired - Vangle;
  pastError[0] = error;
  float delta = (pastError[0] - pastError[pastSize-1])/(pastSize*(float(deltaT)*1e-6));
  sum = min(max(sum + error*float(deltaT)*1e-6,-1*maxSum),maxSum);
  // Compute and limit the motor output command.
  // Factor of 16 so be compatible with Arduino.
  int motorCmd = int(Kp *error + Kd*delta + Ki*sum + direct);
  motorCmd = min(max(motorCmd, 0), dacMax/16);
  analogWrite(motorOutputPin,16*motorCmd);
  
  /***********************************************/

  if (loopCounter == numSkip) {  // Lines below are for debugging.
    packStatus(buf, Vangle, error, delta, sum, float(motorCmd), float(headroom));
    Serial.write(buf,26);
    loopCounter = 0;
  } else loopCounter += 1;

}

void init_loop() {
  // Initialize loop variables
  loopCounter = 0; 
  headroom = deltaT;
}

void processString(String inputString) {
char St = inputString.charAt(0);
  inputString.remove(0,1);
  float val = inputString.toFloat();
  switch (St) {
    case 'P':
      Kp = val;
      break;  
    case 'D':
      Kd = val;
      break;  
    case 'I'
      Ki = val;
      break;
    case 'S'
      maxSum = val;
      break;
    case 'O':  
      direct = val;
      break;
    case 'A':
      desired = val;
      break;
    case '~':
      first_time = true;
      break;
    default:
    break;  
  }
}

// Load the serial output buffer.
void packStatus(char *buf, float a, float b, float c, float d,float e, float f) {
  
  // Start Byte.
  buf[0] = byte(0);
  int n = 1; 
  
  memcpy(&buf[n],&a,sizeof(a));
  n+=sizeof(a);
  memcpy(&buf[n],&b,sizeof(b));
  n+=sizeof(b);
  memcpy(&buf[n],&c,sizeof(c));
  n+=sizeof(c);
  memcpy(&buf[n],&d,sizeof(d));
  n+=sizeof(d);
  memcpy(&buf[n],&d,sizeof(e));
  n+=sizeof(e);
  memcpy(&buf[n],&d,sizeof(f));
  n+=sizeof(f);

  // Stop byte (255 otherwise unused).
  buf[n] = byte(255); 
}

// Initializes the loop if this is the first time, or if reconnect sent
// from GUI.  Otherwise, just look for serial event.
void startup(){
  if (first_time) {
    while(Serial.available() > 0) Serial.read(); // Clear out rcvr.
    Serial.println(config_message);  // Send the configuration files.
    while (! Serial.available()) {}  // Wait for serial return.
    while(Serial.available() > 0) {   // Clear out rcvr.
        Serial.read();
        //char inChar = (char) Serial.read(); 
        //if (inChar == '\n') break;
    }
    init_loop();
    first_time = false;
  } else {
    serialEvent();
  }
}


// Simple serial event, only looks for disconnect character, resets loop if found.
void serialEvent() {
  String inputString = ""; 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // if the incoming character is a newline, ready to process.
    if (inChar == '\n') {
      processString(inputString);
      inputString = "";
      break;
    }
  }
}

