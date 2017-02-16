// Working 4X Encoding. Testing out Non-official PID library to constrain PWM values.

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed.

// Quadrature encoders
// Encoder reading is actually from the cover/label side of encoder so we 
// reverse it in the physical system (instead of pin 2 for Channel A, Channel A connects to Arduino pin 3)
// Channel A is Green Wire | Channel B is Yellow Wire
// Green to pin 3 | Yellow to pin 2

#define c_EncoderInterruptA 0
#define c_EncoderInterruptB 1
#define c_EncoderPinA 2 
#define c_EncoderPinB 3
#define EncoderIsReversed

#define PWM1 6                         // PWM generator
#define CLOCKWISE            0       // direction constant
#define COUNTER_CLOCKWISE    1       // direction constant
#define LOOPTIME        100          // PID loop time

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long _EncoderTicks = 0;

int Direc = 0;  // Flag for CW/CCW direction

// Angle Control
double desiredAngle = 90.0; // the angle that is desired
int desiredCount = 0; // the counts required to move the desired angle
double encoderCount = 400.0; //number of counts per rotation
double cycle = 360.0;

// PID Control
//Define Variables we'll be connecting to
int Setpoint, Input, Output; // Setpoint = desiredCount; Input = _Encoderticks; Output = Output; 
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int PWM_val = 0;
float Kd = 1;                                // PID Derivitave control gain
float Kp = 0;                                // PID proportional control Gain

void setup()
{
  Serial.begin(9600);

  // Quadrature encoders
  //  encoder
  pinMode(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_EncoderInterruptA, HandleMotorInterruptA, CHANGE);
  attachInterrupt(c_EncoderInterruptB, HandleMotorInterruptB, CHANGE);

  pinMode(PWM1, OUTPUT);
  
  desiredCount = int(desiredAngle * (encoderCount/cycle));
  Setpoint = int(desiredCount);
}

void loop() {  
    Input = int(_EncoderTicks);  

    if((millis()-lastMilli) >= LOOPTIME)   {                                     // enter tmed loop
       lastMilli = millis();
       PWM_val= updatePid(PWM_val, Setpoint, Input);                     // compute PWM value
       analogWrite(PWM1, PWM_val);                                               // send PWM to motor
    }
    
      Serial.print("Encoder Ticks: ");
      Serial.print(_EncoderTicks);
      Serial.print("  Revolutions: ");
      Serial.print(_EncoderTicks/400);//400 Counts Per Revolution
      Serial.println (Direc == CLOCKWISE ? " clockwise " : " counter-clockwise ");
      Serial.print("\n");
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  int error=0;                                  
  static int last_error=0;                             
   error = abs(targetValue) - abs(currentValue); 
   pidTerm = (Kp * error) + (Kd * (error - last_error));                            
   last_error = error;
   return constrain(command + int(pidTerm), 0, 255);
}

// Interrupt service routines for Channel A Rising/Falling edge
void HandleMotorInterruptA(){
  _EncoderBSet = digitalReadFast(c_EncoderPinB);
  _EncoderASet = digitalReadFast(c_EncoderPinA);
  
  _EncoderTicks+=ParseEncoder();
  
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;
}

// Interrupt service routines for Channel B Rising/Falling edge
void HandleMotorInterruptB(){
  // Test transition;
  _EncoderBSet = digitalReadFast(c_EncoderPinB);
  _EncoderASet = digitalReadFast(c_EncoderPinA);
  
  _EncoderTicks+=ParseEncoder();
  
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;
}

int ParseEncoder(){
  if(_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && _EncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(_EncoderASet && !_EncoderBSet){
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(!_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && !_EncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(_EncoderASet && _EncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(!_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && !_EncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(!_EncoderASet && _EncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && _EncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(!_EncoderASet && !_EncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }
}

// Angle movement testing

//      desiredCount = int (desiredAngle * (encoderCount/cycle));
//      
//      if (desiredCount <= -1) // If it is moving in a CCW direction
//      { 
//        while(_EncoderTicks >= desiredCount) { // Run motor until desiredCount is reached
//          analogWrite(E1, 120); // Turn on motor in CCW direction
//        }
//      }
//      else if (desiredCount >= 1) // If it is moving in a CW direction
//      {
//        while (_EncoderTicks <= desiredCount) { // Run motor until desiredCount is reached
//          analogWrite(E1, 140); // Turn on motor in CW direction
//        }
//      }
//      else if (desiredCount == 0){
//        analogWrite (E1, 127); // Motor Stop
//      }
//      else {
//        analogWrite(E1, 127);
//      }
//      
//      _EncoderTicks = 0;
//      desiredCount = 0;
//      
