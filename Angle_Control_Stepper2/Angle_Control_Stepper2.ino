// Working 4X Encoding. Adding PID control to it.

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

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long _EncoderTicks = 0;

int Direc = 0;  // Flag for CW/CCW direction

// Angle Control
double desiredAngle1 = 360.0; // the angle that is desired
double desiredAngle2 = 180.0; // the second angle desired
int cnt1 = 0; // the desired counts associated to the angles
int cnt2 = 0; // the desired counts associated to the angles
int desiredCount; 
double encoderCount = 400.0; //number of counts per rotation
double cycle = 360.0;

int countInit = 0;
boolean run = false;  

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

  cnt1 = int(desiredAngle1 * (encoderCount/cycle));
  cnt2 = int(desiredAngle2 * (encoderCount/cycle));
}

void loop() {  
      
      moveMotor(120, cnt1); // CW    // direction, PWM, ticks number
      delay(3000);
      moveMotor(135, cnt2); // CCW             
      delay(3000);
    
      analogWrite(PWM1, PWM_val);
    
      Serial.print("Encoder Ticks: ");
      Serial.print(_EncoderTicks);
      Serial.print("  Revolutions: ");
      Serial.print(_EncoderTicks/400);//400 Counts Per Revolution
      Serial.println (Direc == CLOCKWISE ? " clockwise " : " counter-clockwise ");
      Serial.print("\n");
}

void moveMotor(int PWM_val, int ticks)  {
 analogWrite(PWM1, PWM_val);
 desiredCount = ticks;
 countInit = int(_EncoderTicks); 
 run = true;
}

void checkBrake(){
  if(run){
      if (abs(abs(_EncoderTicks)-abs(countInit)) >= desiredCount){
        motorBrake();
      }
   }
}

void motorBrake()  {
 analogWrite(PWM1, 127);
 run = false;
}

// ------------------- INTERRUPT FUNCTIONS ------------------------ // 

// Interrupt service routines for Channel A Rising/Falling edge
void HandleMotorInterruptA(){
  _EncoderBSet = digitalReadFast(c_EncoderPinB);
  _EncoderASet = digitalReadFast(c_EncoderPinA);
  
  _EncoderTicks+=ParseEncoder();

  checkBrake();
      
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;
}

// Interrupt service routines for Channel B Rising/Falling edge
void HandleMotorInterruptB(){
  // Test transition;
  _EncoderBSet = digitalReadFast(c_EncoderPinB);
  _EncoderASet = digitalReadFast(c_EncoderPinA);
  
  _EncoderTicks+=ParseEncoder();

  checkBrake();
    
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

