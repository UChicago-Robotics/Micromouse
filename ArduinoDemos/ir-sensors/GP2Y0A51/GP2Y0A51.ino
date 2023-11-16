// https://robojax.com/learn/arduino/?vid=robojax_SHARP_0A51SK_IR_LCD
const int sensorPin[] = {A0};
float distance[1];//watch video for details https://youtu.be/rdNCIyL6OA8
const int AVERAGE_OF =50;
const float MCU_VOLTAGE = 5.0;
void setup() {
  Serial.begin(9600); 
}

void loop(){
  readDistance(0);
  Serial.println(distance[0]);
  delay(30);
}

void readDistance(int sensor)
{
      float voltage_temp_average=0;
      for(int i=0; i < AVERAGE_OF; i++)
      {
        int sensorValue = analogRead(sensorPin[sensor] );
        delay(1);      
        voltage_temp_average +=sensorValue * (MCU_VOLTAGE / 1023.0);
      }
     voltage_temp_average /= AVERAGE_OF;
  // equation of the fitting curve
  ////33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
  distance[sensor] = 33.9 + -69.5*(voltage_temp_average) + 62.3*pow(voltage_temp_average,2) + -25.4*pow(voltage_temp_average,3) + 3.83*pow(voltage_temp_average,4);
 }
