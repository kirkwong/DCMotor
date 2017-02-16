// Working 4X Encoding. Adding PID control to it.

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed.

#include <PID_v1.h>            // Arduino PID library

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

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long _EncoderTicks = 0;

int Direc = 0;  // Flag for CW/CCW direction

// Angle Control
double desiredAngle = 360.0; // the angle that is desired
int desiredCount = 0; // the counts required to move the desired angle
double encoderCount = 400.0; //number of counts per rotation
double cycle = 360.0;

// PID Control
//Define Variables we'll be connecting to
double Setpoint, Input, Output; // Setpoint = desiredCount; Input = _Encoderticks; Output = Output; 
double lowGap = 50; // within 45 degrees error

//Define the aggressive and conservative Tuning Parameters
double aggKp=1, aggKi=0, aggKd=0;
double consKp=1, consKi=0, consKd=0;
int PIDout = 0;
int PWM_val = 0;
int negOutput = 0;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT); 

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
  myPID.SetMode(AUTOMATIC);

  desiredCount = int(desiredAngle * (encoderCount/cycle));
  Setpoint = double(desiredCount);
}

void loop() {  
    Input = double(_EncoderTicks);  //casting as doubles as done in examples
    
//    // Adaptive Tuning
//    double gap = abs(Setpoint-Input); //distance away from setpoint
//    if(gap < lowGap) // if within 45 degrees (50 encoder ticks) away
//    {  //we're close to setpoint, use conservative tuning parameters
//      myPID.SetTunings(consKp, consKi, consKd);
//    }
//    else
//    {
//       //we're far from setpoint, use aggressive tuning parameters
//       myPID.SetTunings(aggKp, aggKi, aggKd);
//    }
  
    myPID.Compute();

    // + error for CW || - error for CCW
    negOutput = -1 * Output;  // CW (0 - 126) || CCW (128-255) -- a negative error means to move CCW but the following would move it CW if the output isn't multiplied by -1
    negOutput = negOutput / 2; // scaling the PID value down
    PIDout = constrain(negOutput, -126, 127);
    PWM_val = PIDout + 127;
    
    if (PWM_val >= 120 && PWM_val <= 126) // when the pwm value gets too close to 127, it doesn't move
      PWM_val = 120;
    if (PWM_val >= 128 && PWM_val <= 135)
      PWM_val = 135;

    analogWrite(PWM1, PWM_val);
    
      Serial.print("Encoder Ticks: ");
      Serial.print(_EncoderTicks);
      Serial.print("  Revolutions: ");
      Serial.print(_EncoderTicks/400);//400 Counts Per Revolution
      Serial.println (Direc == CLOCKWISE ? " clockwise " : " counter-clockwise ");
      Serial.print("\n");
}



// ------------------- INTERRUPT FUNCTIONS ------------------------ // 

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

