/*
 keyestudio 4wd BT Car V2
 lesson 2.2
 pwm
 http://www.keyestudio.com
*/
int ledPin = 3; // Define the LED pin at D3
void setup(){
  pinMode (ledPin, OUTPUT); // initialize ledpin as an output.
}
void loop(){
  for (int value = 0; value <255; value = value + 1){
    analogWrite (ledPin, value); // LED lights gradually light up
    delay (30); // delay 30MS
  }
  for(int value=255; value>0;value=value-1){
    analogWrite (ledPin, value); // LED gradually goes out
    delay (30); // delay 30MS
  }
}//**********************************************************
