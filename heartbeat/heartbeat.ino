/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

#define serial Serial1

MAX30105 particleSensor;

const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
int lastVal;
long lastSent = 0;

void setup()
{

  
  serial.begin(115200);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    while (1);
  }

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  for (byte x = 0 ; x < RATE_SIZE ; x++)
    rates[x] = 80;
  beatAvg = 80;
}

void loop()
{
  if ((millis() - lastBeat) > 250) {
    TXLED0;
  }
  long irValue = particleSensor.getIR();
  float bpm2, bpm3, d1, d2, d3 = 0;

  if (irValue > 50000) {
    if (checkForBeat(irValue) == true) {
      TXLED1;
      //We sensed a beat!
      long delta = (millis() - lastBeat);
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
      bpm2 = beatsPerMinute * 2;
      bpm3 = beatsPerMinute * 3;
  
      d1 = abs(beatsPerMinute - beatAvg);
      d2 = abs(bpm2 - beatAvg);
      d3 = abs(bpm3 - beatAvg);
  
      if (d2 <= d1 && d2 <= d3) {
        beatsPerMinute = bpm2;
      } else if (d3 <= d1 && d3 <= d2) {
        beatsPerMinute = bpm3;
      }
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    if (beatAvg != lastVal) {
      serial.print("b");
      serial.println(beatAvg);
      lastVal = beatAvg;
      lastSent = millis();
    }

  } else if (0 != lastVal) {
    serial.println("b0");
    lastVal = 0;
    lastSent = millis();
  }

  if (millis() - lastSent > 500) {
    serial.write(0); // keepalive
    lastSent = millis();
  }
}
