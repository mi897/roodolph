/*
  - Two cores
  - One to handle communication
  - One to get data from sensors
  - Use FreeRTOS tasks to manage each core's workload
  - Ensure thread safety when sharing data between cores
  - Optimize for low latency and high throughput
  - Consider power management for battery-operated devices
  - Implement error handling and recovery mechanisms
  - Use appropriate libraries for sensor communication and data processing
 */
#include <Arduino.h>

#define LED_PIN 5

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("Roodolph Hub is running...");
}

void loop()
{
  // put your main code here, to run repeatedly:

  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED is ON");
  Serial.println(millis());
  delay(500);
  digitalWrite(LED_PIN, LOW);
  Serial.println("LED is OFF");
  Serial.println(millis());
  delay(500);

}
