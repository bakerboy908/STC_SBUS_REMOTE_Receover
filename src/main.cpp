#include <Arduino.h>
#include "sbus.h"
#include <EEPROM.h>
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
#define ZoomEEPROMAddress 0

int16_t ZoomPWMValDesired = 4400;
int16_t ZoomPWMValDesiredOLD;
long messageCount = 0;
int ZoomPWMVal = 2200;
void setup()
{
  

  Serial.begin(115200);
  Serial.println("SYSTEM STARTING");
  delay(1000);
  ZoomPWMValDesired = EEPROM.read(ZoomEEPROMAddress) | (EEPROM.read(ZoomEEPROMAddress + 1) << 8);

  ZoomPWMValDesiredOLD = ZoomPWMValDesired;
  ZoomPWMVal = ZoomPWMValDesired;
  Serial.println("EEPROM Read");
  Serial.println(ZoomPWMValDesired);
  // put your setup code here, to run once:
  sbus_tx.Begin();
  Serial.println("SBUS RX/TX started");
  pinMode(SERVO_PWM_PIN, OUTPUT);
  analogWrite(SERVO_PWM_PIN, ZoomPWMValDesired);
  delay(1000);
  pinMode(HC_12_SETPIN, OUTPUT);
  digitalWrite(HC_12_SETPIN, HIGH);

  // Set the PWM frequency to 50Hz
  analogWriteFrequency(SERVO_PWM_PIN, 50);
#ifdef PWM_16_BIT
  // set PWM resolution to 15 bits
  analogWriteResolution(15);
#endif

  Serial1.begin(9600);

  // Setup Complete
  Serial.println("Setup Complete");
}
void ChangeChannel(int Channel)
{
  // Put HC-12 in to command mode
  digitalWrite(HC_12_SETPIN, LOW);
  delay(100);
  // Send command to change channel with extra leading zeros before channel number
  Serial1.print("AT+C");
  Serial1.print("0");
  Serial1.print("0");
  Serial1.println(Channel);

  // Wait for response
  delay(100);
  // Put HC-12 back in to normal mode
  digitalWrite(HC_12_SETPIN, HIGH);
  delay(100);
}

void loop()
{
  // if Serial1 Available
  if (Serial1.available() >= 1)
  {
    // temp array for 8 bytes
    byte temp[16];
    // read 8 bytes from Serial1
    // Serial.println("Receiving Packet");
    // delay(1000);
    // Serial1.readBytes(temp, sizeof(temp));
    // Create a sliding window reading serial bits in, until we find the start code
    bool searching = true;
    int startcount = 0;
    while (searching)
    {
      // delay(10);
      // for loop to move data down one slot in temp array
      for (auto i = 0; i < 5; i++)
      {
        temp[i] = temp[i + 1];
      }
      // read one byte from serial into back of temp array
      temp[4] = Serial1.read();
      // print out the temp array
      // ++;

      if (temp[0] == 'S' && temp[1] == 'T' && temp[2] == 'A' && temp[3] == 'R' && temp[4] == 'T')
      {
        // if start code found, break out of while loop
        searching = false;
        messageCount++;
        // Serial.print("Start Condition Found: ");
        // Serial.print(messageCount);
      }
      // check for chang channel code
      if (temp[0] == 'C' &&
          temp[1] == 'H' &&
          temp[2] == 'A' &&
          temp[3] == 'N' &&
          temp[4] == 'G')
      {
        // if start code found, break out of while loop
        searching = false;
        Serial.println("Change Channel Code Found");
      }
      if (Serial1.available() <= 10)
      {
        searching = false;
      }
    }

    if (temp[0] == 'S' && temp[1] == 'T' && temp[2] == 'A' && temp[3] == 'R' && temp[4] == 'T')
    {
      // Read the rest of the packet
      Serial1.readBytes(temp + 5, 8);
      // Serial.println("Start Condition Found");
    }
    else if (temp[0] == 'C' &&
             temp[1] == 'H' &&
             temp[2] == 'A' &&
             temp[3] == 'N' &&
             temp[4] == 'G' &&
             temp[5] == 'E')
    {
      while (Serial1.available() == 0)
      {
        /* code */
      }

      // Read the rest of the packet
      byte NewChannel_Char = Serial1.read();
      Serial.print("Changing Channel to: ");
      Serial.println((char)NewChannel_Char);
      // respond with the neq channel number
      Serial1.print((char)NewChannel_Char);
      // Change the channel
      ChangeChannel(NewChannel_Char);
    }
    else
    {
      // Purge the buffer
      Serial1.flush();
    }

    if (temp[0] == 'S' && temp[1] == 'T' && temp[2] == 'A' && temp[3] == 'R' && temp[4] == 'T')
    {

      int oldZoomPWMValDesired = ZoomPWMValDesired;
      auto oldDat = data;
      // reconstrct the first 3 channels in data from temp array
      data.ch[0] = temp[5] + (temp[6] << 8);
      data.ch[1] = temp[7] + (temp[8] << 8);
      data.ch[2] = temp[9] + (temp[10] << 8);

      // Reconstruct the check sum
      uint16_t checksum = temp[11] + (temp[12] << 8) + (temp[13] << 16) + (temp[14] << 24);

      // Check the checksum
      if (checksum == data.ch[0] + data.ch[1] + data.ch[2])
      {
        // Checksum is good
        Serial.println("Checksum Good");
      }
      else
      {
        // Checksum is bad
        Serial.println("Checksum Bad");
        data = oldDat;
      }


      // print the data
      ZoomPWMValDesired = data.ch[0];

      // Serial.println("Packet Received");
      // Serial.println(data.ch[0]);
      // Serial.println(data.ch[1]);
      // Serial.println(data.ch[2]);
      if (ZoomPWMValDesired != oldZoomPWMValDesired)
      {
        Serial.print("ZoomPWMValDesired: ");
        Serial.println(ZoomPWMValDesired);
      

      
      Serial.print(" ZoomPWMValDesired: ");
      Serial.println(ZoomPWMValDesired);
      if (ZoomPWMValDesired == Zoom12Val ||
          ZoomPWMValDesired == Zoom14Val ||
          ZoomPWMValDesired == Zoom15Val ||
          ZoomPWMValDesired == Zoom18Val ||
          ZoomPWMValDesired == Zoom25Val ||
          ZoomPWMValDesired == Zoom30Val ||
          ZoomPWMValDesired == Zoom35Val ||
          ZoomPWMValDesired == Zoom40Val)
      {

          EEPROM.write(ZoomEEPROMAddress, ZoomPWMValDesired & 0xFF);
          EEPROM.write(ZoomEEPROMAddress + 1, ZoomPWMValDesired >> 8);
          ZoomPWMValDesiredOLD = ZoomPWMValDesired;
          Serial.println("Zoom Value Saved");

        

      }
      else
      {
        ZoomPWMValDesired = oldZoomPWMValDesired;
        Serial.println("Invalid Zoom Value");
      }
      }

      Serial1.print("START");
      Serial1.write(data.ch[0] & 0xFF);
      Serial1.write(data.ch[0] >> 8);

      Serial1.write(data.ch[1] & 0xFF);
      Serial1.write(data.ch[1] >> 8);

      Serial1.write(data.ch[2] & 0xFF);
      Serial1.write(data.ch[2] >> 8);
      Serial1.print("END");
    }
    // else
    // purge the buffer
    // Serial1.flush();
  }
  static unsigned long previousMillis = 0;
  // save the current time
  unsigned long currentMillis = millis();

  // if 15ms have passed since the last time the loop ran
  if (currentMillis - previousMillis >= 20)
  {
    // save the last time the loop ran
    previousMillis = currentMillis;
    // write the SBUS data
    sbus_tx.data(data);
    // Transmit SBUS data
    sbus_tx.Write();
  }

  // Zoom Control

  // If the desired zoom value is differnt from the current zoom value slowly change the zoom value until they match
  // Only Check the zoom value every 500ms
  static unsigned long previousMillisZoom = 0;
  // save the current time
  unsigned long currentMillisZoom = millis();
  // if 500ms have passed since the last time the loop ran
  if (currentMillisZoom - previousMillisZoom > 2)
  {
    // Serial.println("Milis 1");
    // save the last time the loop ran
    previousMillisZoom = currentMillisZoom;
    // Check the zoom value
    if (ZoomPWMValDesired != ZoomPWMVal)
    {
      // Serial.println("Zoom desiered != Zoom");
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