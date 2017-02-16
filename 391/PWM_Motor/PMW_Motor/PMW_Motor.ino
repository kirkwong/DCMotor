#define E1 6 // Enable Pin for motor 1
 
void setup() {
 
    pinMode(E1, OUTPUT);
}
 
void loop() {

 // M1A goes to the red Dot. 
 // 127 is mid point, motor doesnt move in any direction. 

    analogWrite(E1, 135);
        
}

 
//analogWrite(E1, 255); // Run in full speed
// 
//    delay(5000); // 1 sec 
// 
//    //change speed 
//
//    analogWrite(E1, 200); // Run in half speed
// 
//    delay(5000); // 1 sec
//
//    //change speed 
//
//    analogWrite(E1, 127); // off
// 
//    delay(5000); // 1 sec
////
////    // change direction and speed
////
//    analogWrite(E1, 60); // Run in half speed
//// 
//    delay(5000); // 10 sec
//
//    // change direction and speed 
//
//    analogWrite(E1, 0); // Run in full speed
//// 
//    delay(5000); // 10 sec   
//
//    analogWrite(E1, 127); // off
// 
//    delay(5000); // 10 sec
