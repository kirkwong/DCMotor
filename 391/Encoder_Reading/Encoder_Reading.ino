#define E1 3                        // PWM generator
#define CLOCKWISE            0       // direction constant
#define COUNTER_CLOCKWISE    1       // direction constant

int InputA = 2; //Green
int InputB = 5; //Yellow

int Direc = 0;  
volatile int counter = 0; 
volatile long rotaryposition = 0;

short currentDirection = CLOCKWISE;
long initialposition;

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

      analogWrite(E1, 135);

     

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

//       delay ( 5000);
//
     // analogWrite( E1, 130);
//
//      delay (5000);
}
