#include <Arduino.h>
#include "sbus.h"
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial2);
/* SBUS data */
bfs::SbusData data;

#define MAX_MESSAGE_LENGTH 100

#define HC_12_SETPIN 22

#define SERVO_PWM_PIN 15
// #DEFINE PWM_8_BIT
#define PWM_16_BIT
// 8 int values for Zoom possitions
#ifdef PWM_8_BIT
int16_t Zoom12Val = 17;
int16_t Zoom14Val = 19;
int16_t Zoom15Val = 20;
int16_t Zoom18Val = 22;
int16_t Zoom25Val = 25;
int16_t Zoom30Val = 28;
int16_t Zoom35Val = 31;
int16_t Zoom40Val = 34;
#endif
#ifdef PWM_16_BIT
int16_t Zoom12Val = 2220;
int16_t Zoom14Val = 2472;
int16_t Zoom15Val = 2630;
int16_t Zoom18Val = 2855;
int16_t Zoom25Val = 3253;
int16_t Zoom30Val = 3624;
int16_t Zoom35Val = 3975;
int16_t Zoom40Val = 4400;
#endif

int ZoomPWMValDesired = 2200;

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  sbus_tx.Begin();
  Serial.println("SBUS RX/TX started");
  pinMode(SERVO_PWM_PIN, OUTPUT);
  pinMode(HC_12_SETPIN, OUTPUT);
  digitalWrite(HC_12_SETPIN, HIGH);

  // Set the PWM frequency to 50Hz
  analogWriteFrequency(SERVO_PWM_PIN, 66);
#ifdef PWM_16_BIT
  // set PWM resolution to 15 bits
  analogWriteResolution(15);
#endif

  Serial1.begin(9600);

  // Setup Complete
  Serial.println("Setup Complete");
}

void loop()
{
  // if Serial1 Available
  if (Serial1.available() >= 1)
  {
    // temp array for 8 bytes
    byte temp[8];
    // read 8 bytes from Serial1
    Serial.println("Receiving Packet");
    // delay(1000);
    // Serial1.readBytes(temp, sizeof(temp));
    for (auto i = 0; i < 8; i++)
    {
      /* code */
      temp[i] = Serial1.read();
      if ((char)temp[0] != 'S')
      {
        break;
      }
    }
    if ((char)temp[0] == 'S')
    {
      /* code */
      // reconstrct the first 3 channels in data from temp array
      data.ch[0] = temp[1] + (temp[2] << 8);
      ZoomPWMValDesired = data.ch[0];
      data.ch[1] = temp[3] + (temp[4] << 8);
      data.ch[2] = temp[5] + (temp[6] << 8);
      // print the data

      Serial.println("Packet Received");
      Serial.println(data.ch[0]);
      Serial.println(data.ch[1]);
      Serial.println(data.ch[2]);
    }
    // else
      // purge the buffer
      // Serial1.flush();
  }
  static unsigned long previousMillis = 0;
  // save the current time
  unsigned long currentMillis = millis();
  // if 15ms have passed since the last time the loop ran
  if (currentMillis - previousMillis >= 15)
  {
    // save the last time the loop ran
    previousMillis = currentMillis;
    // write the SBUS data
    sbus_tx.data(data);
    // Transmit SBUS data
    sbus_tx.Write();
  }

  // Zoom Control

  static int ZoomPWMVal = 2200;
  // If the desired zoom value is differnt from the current zoom value slowly change the zoom value until they match
  // Only Check the zoom value every 500ms
  static unsigned long previousMillisZoom = 0;
  // save the current time
  unsigned long currentMillisZoom = millis();
  // if 500ms have passed since the last time the loop ran
  if (currentMillisZoom - previousMillisZoom >= 1)
  {
    // save the last time the loop ran
    previousMillisZoom = currentMillisZoom;
    // Check the zoom value
    if (ZoomPWMValDesired != ZoomPWMVal)
    {
      if (ZoomPWMValDesired > ZoomPWMVal)
      {
        ZoomPWMVal++;
      }
      else if (ZoomPWMValDesired < ZoomPWMVal)
      {
        ZoomPWMVal--;
      }
      // Update the servo PWM value
      analogWrite(SERVO_PWM_PIN, ZoomPWMVal);
    }
  }
}