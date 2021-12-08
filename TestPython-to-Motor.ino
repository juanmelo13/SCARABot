const int stepPinX = 2; //X.STEP
const int dirPinX = 5; // X.DIR

const int motorSpeed = 500;




void setup() {
  // put your setup code here, to run once:
 pinMode(stepPinX,OUTPUT); 
 pinMode(dirPinX,OUTPUT);
 Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
 digitalWrite(dirPinX,LOW); // Enables the motor to move in a particular direction
 while (Serial.available()==0){
 }
   int angleRotation=Serial.parseInt();//*(200/360)
//   String angleRotation=Serial.readString();
//   int b=angleRotation.toInt();
//   angleRotation=angleRotation+1;
   Serial.println("Received");
   Serial.println(angleRotation);
   Serial.println("Received");
   // Makes 200 pulses for making one full cycle rotation
   for(int x = 0; x < angleRotation; x++) {
   digitalWrite(stepPinX,HIGH); 
   delayMicroseconds(motorSpeed); 
   digitalWrite(stepPinX,LOW); 
   delayMicroseconds(motorSpeed);
 }
}
