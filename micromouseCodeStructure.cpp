#include <SoftwareSerial.h>

#include <Wire.h>
#include <MPU6050.h>
 

#define LEFT_A1 4
#define RIGHT_A2 6 
 

#define IR_TRIG 9 
#define IR_ECHO 8 

MPU6050 mpu;

bool mode = true;
 
void setup() {
  Serial.begin(9600);  
 

  pinMode(LEFT_A1, OUTPUT);
  pinMode(RIGHT_A2, OUTPUT);
 

  pinMode(IR_TRIG, OUTPUT);
  pinMode(IR_ECHO, INPUT);
}
 
void loop() {
  
  float duration, distance;
  digitalWrite(IR_TRIG, HIGH);
  delay(10);
  digitalWrite(IR_TRIG, LOW);
 

  duration = pulseIn(IR_ECHO, HIGH);

  distance = ((float)(340 * duration) / 10000) / 2;
  Serial.print("\nDistance : ");
  Serial.println(distance);
  int sum = 0;

  //if we are going to hit a wall, stop and calibrate directions
  if(distance < 20) {      
     Serial.println("stop");
     stop(); 
    
    //calibration algorithm

   }
   //if we are chilling, then decide if we are sensing (mapping) or moving (with flood fill)
   else {        
      if (mode == true) {

        //run moving algorithm
      }
      else {
        //run sensing algorithm
  }
   }
  
 


}
 
void forward(){
  digitalWrite(LEFT_A1, HIGH);
  digitalWrite(RIGHT_A2, HIGH);
}
void forwardi (){
  digitalWrite(LEFT_A1, HIGH);
  digitalWrite(RIGHT_A2, HIGH);
  delay (4000);
}
void backward(){
  digitalWrite(LEFT_A1, LOW);
  digitalWrite(RIGHT_A2, LOW);
  delay(1000);
}
void left(){
  digitalWrite(LEFT_A1, LOW);
  digitalWrite(RIGHT_A2, HIGH);
  delay(1000);
}
void right(){
  digitalWrite(LEFT_A1, HIGH);
  digitalWrite(RIGHT_A2, LOW);
  delay(1000);
}
void stop(){
  digitalWrite(LEFT_A1, LOW);
  digitalWrite(RIGHT_A2, LOW);
  delay(3000);
}