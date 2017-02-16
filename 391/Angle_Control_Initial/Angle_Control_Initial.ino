#define E1 3                         // PWM generator
#define CLOCKWISE            0       // direction constant
#define COUNTER_CLOCKWISE    1       // direction constant

int InputA = 2;
int InputB = 5; 

int Direc = 0;  
volatile int counter = 0; 
volatile long rotaryposition = 0;

short currentDirection = CLOCKWISE;
long initialposition;

// Angle Control
double desiredAngle = 0; // the angle that is desired
int desiredCount = 50; // the counts required to move the desired angle
int i = 0;
int j = 0;
double encoderCount = 100; //number of counts per rotation

// channel A interrupts 
void emitter (){

  if (digitalRead(InputA) == digitalRead(InputB)) 
    {
      Direc = CLOCKWISE;  
      counter++;
      rotaryposition++;
     }
   else
   { 
       Direc = COUNTER_CLOCKWISE;  
       counter--; 
       rotaryposition--;
   }
}

void setup() {

  pinMode( InputA, INPUT);
  pinMode( InputB, INPUT);
  pinMode(E1, OUTPUT);

  attachInterrupt( digitalPinToInterrupt(InputA), emitter, RISING);

  Serial.begin (250000);
}

void loop() {
//      desiredAngle = 270; //enter angle here
//      
//      desiredCount = 0;
//      desiredCount = desiredAngle * (encoderCount/360)
//
      if (desiredCount <= -1) //CCW
      { 
        while(counter >= desiredCount) {
          analogWrite(E1, 120);
        }
      }
      else if (desiredCount >= 1) //CW
      {
        while (counter <= desiredCount) {
          analogWrite(E1, 140);
        }
      }
      else if (desiredCount == 0){
        analogWrite (E1, 127); 
      }
      else {
        analogWrite(E1, 127);
      }

      counter = 0;
      desiredCount = 0;

      if ( rotaryposition != initialposition)
      {  
        Serial.print ("Begin");
        Serial.print ("\n");
        Serial.print (rotaryposition, DEC);
        Serial.print ("\t");
        Serial.print ("\t");
        Serial.println (Direc == CLOCKWISE ? "clockwise" : "counter-clockwise");
        
        initialposition= rotaryposition;
      }
}
