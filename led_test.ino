const int stepPin = 3; //X.STEP
String InBytes;

void setup() {
  // put your setup code here, to run once:
pinMode(stepPin,OUTPUT);
Serial.begin(115200); // open the serial port
}

void loop() {
  // put your main code here, to run repeatedly:
if (Serial.available()>0){
  InBytes=Serial.readStringUntil('\n');
  if (InBytes=="on"){
    digitalWrite(stepPin,HIGH);
    Serial.write("Led ON");
  }
  if (InBytes=="off"){
    digitalWrite(stepPin,LOW);
    Serial.write("Led OFF");
  }
    
  }
}
