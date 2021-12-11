#include "math.h"

//double* inverseKinematics(double x, double y, double L1, double L2);
//int angleToJoint1(const int angle);

const int stepPinX = 2; //X.STEP
const int dirPinX = 5; // X.DIR
const int stepPinY = 3; //Y.STEP
const int dirPinY = 6; // Y.DIR
const int stepPinZ = 4; //Z.STEP
const int dirPinZ = 7; // Z.DIR
const int motorSpeed = 1700;
const int motorSpeed2 = 1000;
float x = 0;
float y = 282.84;
double theta1;
double theta2;

double numerator;
double denominator;

float L1 = 200;
float L2 = 200;
//float n1, n2;
float n1 = 300;
float n2 = 332.5;
float n3 = 5000;


 
 void setup() {
 // Sets the two pins as Outputs
 pinMode(stepPinX,OUTPUT); 
 pinMode(dirPinX,OUTPUT);
 pinMode(stepPinY,OUTPUT); 
 pinMode(dirPinY,OUTPUT);
  pinMode(stepPinZ,OUTPUT); 
 pinMode(dirPinZ,OUTPUT);
 Serial.begin(9600); // open the serial port at 9600 bps:
 }
 void loop() {

//  double *angles;
//  angles = inverseKinematics(x, y, L1, L2);

  numerator = ((x*x) + (y*y) - (L1*L1) - (L2*L2));
  denominator = (2 * L1 * L2);
//  Serial.println(numerator);
//  Serial.println(denominator);
  theta2 = acos(numerator / denominator);
  theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));

  
  n1 = theta1 * 180 * 200 * 6.65 / (360 * PI);
  n2 = theta2 * 180 * 200 * 6.65 / (360 * PI);

  
//  if (x >= 0 && y >= 0) {       // 1st quadrant
//    theta1 = 90.0 - theta1;
//  }
//  if (x < 0 && y > 0) {       // 2nd quadrant
//    theta1 = 90.0 - theta1;
//  }
//  if (x < 0 && y < 0) {       // 3d quadrant
//    theta1 = 270 - theta1;
////    phi = 270 - angles[0] - theta2;
////    phi = (-1) * phi;
//  }
//  if (x > 0 && y < 0) {       // 4th quadrant
//    theta1 = -90 - theta1;
//  }
//  if (x < 0 && y == 0) {
//    theta1 = 270 + theta1;
//  }
  Serial.println(n1);
  Serial.println(n2);
  Serial.println(theta1 * 180 / PI);
  Serial.println(theta2 * 180 / PI);
//  n1 = angleToJoint1(theta1);
//  n2 = angleToJoint1(theta2);
// digitalWrite(dirPinX,LOW); // Enables the motor to move in a particular direction
// Serial.print("Motor X\n");
// // Makes 200 pulses for making one full cycle rotation
// for(int x = 0; x < n1; x++) {
// digitalWrite(stepPinX,HIGH); 
// delayMicroseconds(motorSpeed); 
// digitalWrite(stepPinX,LOW); 
// delayMicroseconds(motorSpeed);
// }

//TEST CASE #1
//  digitalWrite(dirPinY,LOW); // Enables the motor to move in a particular direction
// Serial.print("Motor Y\n");
// // Makes 200 pulses for making one full cycle rotation
// for(int x = 0; x < n1; x++) {
// digitalWrite(stepPinY,HIGH); 
// delayMicroseconds(motorSpeed); 
// digitalWrite(stepPinY,LOW); 
// delayMicroseconds(motorSpeed);
// }
//   digitalWrite(dirPinX,LOW); // Enables the motor to move in a particular direction
// Serial.print("Motor X\n");
// // Makes 200 pulses for making one full cycle rotation
// for(int x = 0; x < n2; x++) {
// digitalWrite(stepPinX,HIGH); 
// delayMicroseconds(motorSpeed); 
// digitalWrite(stepPinX,LOW); 
// delayMicroseconds(motorSpeed);
// }
// delay(2000);
//   digitalWrite(dirPinY,HIGH); // Enables the motor to move in a particular direction
// Serial.print("Motor Y\n");
// // Makes 200 pulses for making one full cycle rotation
// for(int x = 0; x < n1; x++) {
// digitalWrite(stepPinY,HIGH); 
// delayMicroseconds(motorSpeed); 
// digitalWrite(stepPinY,LOW); 
// delayMicroseconds(motorSpeed);
// }
//    digitalWrite(dirPinX,HIGH); // Enables the motor to move in a particular direction
// Serial.print("Motor X\n");
// // Makes 200 pulses for making one full cycle rotation
// for(int x = 0; x < n2; x++) {
// digitalWrite(stepPinX,HIGH); 
// delayMicroseconds(motorSpeed); 
// digitalWrite(stepPinX,LOW); 
// delayMicroseconds(motorSpeed);
// }
//  delay(5000);
  
      digitalWrite(dirPinZ,LOW); // Enables the motor to move in a particular direction
 Serial.print("Motor Z\n");
 // Makes 200 pulses for making one full cycle rotation
 for(int x = 0; x < n3; x++) {
 digitalWrite(stepPinZ,HIGH); 
 delayMicroseconds(motorSpeed2); 
 digitalWrite(stepPinZ,LOW); 
 delayMicroseconds(motorSpeed2);
 }
delay(1000);

digitalWrite(dirPinZ,HIGH); // Enables the motor to move in a particular direction
 Serial.print("Motor Z\n");
 // Makes 200 pulses for making one full cycle rotation
 for(int x = 0; x < n3; x++) {
 digitalWrite(stepPinZ,HIGH); 
 delayMicroseconds(motorSpeed2); 
 digitalWrite(stepPinZ,LOW); 
 delayMicroseconds(motorSpeed2);
 }
delay(1000);


//END TEST CASE #1

//  while(true);
// delay(1000); // One second delay
// 
// digitalWrite(dirPin,LOW); //Changes the rotations direction
// Serial.print("Dos vueltas\n");
//
// // Makes 400 pulses for making two full cycle rotation
// for(int x = 0; x < n2; x++) {
// digitalWrite(stepPin,HIGH);
// delayMicroseconds(motorSpeed);
// digitalWrite(stepPin,LOW);
// delayMicroseconds(motorSpeed);
// }
// delay(1000);
}

int angleToJoint1(int angle) {
  return angle * 200 * 7 / 360;
}

//double* inverseKinematics(double x, double y, double L1, double L2) {
//  double angles[2] = {0, 0};
//  angles[1] = acos((sqrt(x) + sqrt(y) - sqrt(L1) - sqrt(L2)) / (2 * L1 * L2));
//  if (x < 0 && y < 0) {
//    angles[1] = (-1.0) * angles[1];
//  }
//  
//  angles[0] = atan(x / y) - atan((L2 * sin(angles[1])) / (L1 + L2 * cos(angles[1])));
//  
//  angles[1] = (-1.0) * (angles[1]) * 180.0 / PI;
//  angles[0] = angles[0] * 180.0 / PI;
//
// // Angles adjustment depending in which quadrant the final tool coordinate x,y is
//  if (x >= 0 && y >= 0) {       // 1st quadrant
//    angles[0] = 90.0 - angles[0];
//  }
//  if (x < 0 && y > 0) {       // 2nd quadrant
//    angles[0] = 90.0 - angles[0];
//  }
//  if (x < 0 && y < 0) {       // 3d quadrant
//    angles[0] = 270 - angles[0];
////    phi = 270 - angles[0] - theta2;
////    phi = (-1) * phi;
//  }
//  if (x > 0 && y < 0) {       // 4th quadrant
//    angles[0] = -90 - angles[0];
//  }
//  if (x < 0 && y == 0) {
//    angles[0] = 270 + angles[0];
//  }
//
//  return angles;
//}
