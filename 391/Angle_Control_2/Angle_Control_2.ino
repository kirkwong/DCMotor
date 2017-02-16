// 2nd interation of Angle Control with 4X Encoding. Added Dir to ParseEncoder fn. May need to change if it doesn't work

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed.

// Quadrature encoders
// Left encoder
#define c_LeftEncoderInterruptA 0
#define c_LeftEncoderInterruptB 1
#define c_LeftEncoderPinA 2
#define c_LeftEncoderPinB 3
#define LeftEncoderIsReversed

#define E1 6                         // PWM generator
#define CLOCKWISE            0       // direction constant
#define COUNTER_CLOCKWISE    1       // direction constant

volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile long _LeftEncoderTicks = 0;

int Direc = 0;  // Flag for CW/CCW direction

// Angle Control
double desiredAngle = 0.0; // the angle that is desired
int desiredCount = 0; // the counts required to move the desired angle
double encoderCount = 400.0; //number of counts per rotation
double cycle = 360.0;

void setup()
{
  Serial.begin(9600);

  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(c_LeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_LeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_LeftEncoderInterruptA, HandleLeftMotorInterruptA, CHANGE);
  attachInterrupt(c_LeftEncoderInterruptB, HandleLeftMotorInterruptB, CHANGE);

  pinMode(E1, OUTPUT);
}

void loop() {  
      desiredCount = int (desiredAngle * (encoderCount/cycle));
      
      if (desiredCount <= -1) // If it is moving in a CCW direction
      { 
        while(_LeftEncoderTicks >= desiredCount) { // Run motor until desiredCount is reached
          analogWrite(E1, 120); // Turn on motor in CCW direction
        }
      }
      else if (desiredCount >= 1) // If it is moving in a CW direction
      {
        while (_LeftEncoderTicks <= desiredCount) { // Run motor until desiredCount is reached
          analogWrite(E1, 140); // Turn on motor in CW direction
        }
      }
      else if (desiredCount == 0){
        analogWrite (E1, 127); // Motor Stop
      }
      else {
        analogWrite(E1, 127);
      }
      
      _LeftEncoderTicks = 0;
      desiredCount = 0;
      
      Serial.print("Encoder Ticks: ");
      Serial.print(_LeftEncoderTicks);
      Serial.print("  Revolutions: ");
      Serial.print(_LeftEncoderTicks/400.0);//400 Counts Per Revolution
      Serial.println (Direc == CLOCKWISE ? " clockwise " : " counter-clockwise ");
      Serial.print("\n");
}

// Interrupt service routines for Channel A Rising/Falling edge
void HandleLeftMotorInterruptA(){
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoder();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

// Interrupt service routines for Channel B Rising/Falling edge
void HandleLeftMotorInterruptB(){
  // Test transition;
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoder();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

int ParseEncoder(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(_LeftEncoderASet && !_LeftEncoderBSet){
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(_LeftEncoderASet && _LeftEncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(!_LeftEncoderASet && _LeftEncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) {
      Direc = CLOCKWISE;  
      return 1;
    }
    if(!_LeftEncoderASet && !_LeftEncoderBSet) {
      Direc = COUNTER_CLOCKWISE;  
      return -1;
    }
  }
}
