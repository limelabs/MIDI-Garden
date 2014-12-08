// This software is placed under GNU GPL 3.0 license. 
// Copyright (c) Philipp Stute aka. Tom Trialanderror
// http://www.limelabs.eu
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed WITHOUT ANY WARRANTY OF ANY KIND, INCLUDING
// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHOR
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER  LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along
// with this program. If not, see http://www.gnu.org/licenses/ 
// 
// The above copyright notice and permission notice shall be included in
// all copies or substantial portions of the Software.
//
// This Arduino sketch is the software for the Lime Labs MIDI Garden. 
// Version 0.7.0
// Please visit http://www.limelabs.eu/midi-garden for details on the awesome
// device this program has been written for. 

//--------------------------------------------------------------------------
//-                                                                INCLUDE -
#include "Wire.h"
#include <stdint.h>
#include <SPI.h>
#include <SD.h>
#include <DAC_MCP49xx.h>

/*
  MIDI library code is placed under the terms of GNU GPL 3.0.
  Copyright (c) Francoius Best (Forty Seven Effects)
*/
#include <MIDI.h>
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midiB);

/* 
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
MPU6050 mpu;
MPU6050 accelgyro;

/*
 This is the core graphics library for all our displays, providing a common
 set of graphics primitives (points, lines, circles, etc.).
 
 It needs to be paired with a hardware-specific library for each display 
 device we carry (to handle the lower-level functions).
 
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 Written by Limor Fried/Ladyada for Adafruit Industries.
 MIT license, all text above must be included in any redistribution
 */
#include "Adafruit_GFX.h" // Core graphics library

/*
 This is a library for the Adafruit HX8357 display products.
 
 This library works with the Adafruit 3.5" Breakout
 ----> http://www.adafruit.com/products/2050
 Check out the links above for our tutorials and wiring diagrams.
 These displays use SPI to communicate, 4 or 5 pins are required
 to interface (RST is optional).
 
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 Written by Limor Fried/Ladyada for Adafruit Industries.
 MIT license, all text above must be included in any redistribution
 */
#include <Adafruit_TFTLCD.h>
#include <TouchScreen.h>

//--------------------------------------------------------------------------
//-                                                         PIN DEFINITION -

// TOUCHSCREEN PINS
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 23   // can be a digital pin
#define XP 22   // can be a digital pin

// TFT PINS
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET -1
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// SD CARD
#define SD_CS 44

// DEFINITION OF SS_PIN AND DAC SETUP
#define SS_PIN1 46
#define SS_PIN2 47
#define SS_PIN3 48
#define SS_PIN4 49
DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, SS_PIN1);
DAC_MCP49xx dac2(DAC_MCP49xx::MCP4922, SS_PIN2);
DAC_MCP49xx dac3(DAC_MCP49xx::MCP4922, SS_PIN3);
DAC_MCP49xx dac4(DAC_MCP49xx::MCP4922, SS_PIN4);

// ANALOG INPUT PINS
const int16_t analogInPin0 = A4; 
const int16_t analogInPin1 = A5;
const int16_t analogInPin2 = A6;
const int16_t analogInPin3 = A7;
const int16_t analogInPin4 = A8;
const int16_t analogInPin5 = A9;
const int16_t analogInPin6 = A10;
const int16_t analogInPin7 = A11;

// LEDs
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
const uint8_t errorLED = 37;
const uint8_t out2LED = 38;

//--------------------------------------------------------------------------
//-                                           CALIBRARTE TOUCH SCREEN HERE -
#define TS_MINX 150
#define TS_MINY 80
#define TS_MAXX 895
#define TS_MAXY 926

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 278);

#define MINPRESSURE 15
#define MAXPRESSURE 1000

//--------------------------------------------------------------------------
//-                                                       GLOBAL VARIABLES -
// MOTION PROCESSING UNIT VARIABLES
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

// MOTION SENSOR UNIT (MSU) CALIBRATION ROUTINE VARIABLES
int buffersize=1000;     // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset=0,ay_offset=0,az_offset=0,gx_offset=0,gy_offset=0,gz_offset=0;

// MPU INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// PRESET VARIABLES
char patchNames[10][8] = {"AmpEnvS", "FltRes", "Osc1Wav", "Aftert", "Mod Wh", "Osc2Lv", "CC 15", "CC 33", "N/A", "N/A"};
uint8_t sensorTypeIdentifier[10] = { 16, 17, 18, 6, 0, 1, 5, 2, 3, 4};
boolean isDmp[10] = {true, true, true, false, false, false, false, false, false, false};
boolean isMidiOut1[10] = {true, true, true, true, true, true, true, true, true, true};
boolean isMidiOut2[10] = {false, false, false, false, false, false, false, false, false, false};
boolean isMidiOut3[10] = {false, false, false, false, false, false, false, false, false, false};
uint8_t commandIdentifier[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t midiChan[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t firstDataByte[10] = {30, 21, 9, 1, 1, 17, 16, 15, 33, 0};
uint8_t firstDataByteB[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t secondDataByte[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float dmpMinInValue[10] = {-1.3, -1.3, -3.1416, 0, 0, 0, 0, 0, 0, 0};
float dmpMaxInValue[10] = {1.3, 1.3, 3.1416, 0, 0, 0, 0, 0, 0, 0};
uint16_t minInValue[10] = {0, 0, 0, 0, 0, 0, 0, 50, 0, 0};
uint16_t maxInValue[10] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 568, 1023, 1023};
int16_t minOutValue[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t maxOutValue[10] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127};
int16_t maxOutLimit[10] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127};
boolean mute[10] = {true, true, true, true, true, true, true, true, true, true};
uint8_t liveMode = 2;


char sensorTypes[][8] = {"A-0", "A-1", "A-2", "A-3", "A-4", "A-5", "A-6", "A-7", "CV0", "CV1", "CV2", "CV3", "CV4", "CV5", "CV6", "CV7", "Roll 1", "Pitch 1", "Yaw 1", "Roll 2", "Pitch 2", "Yaw 2", "Acc X 1", "Acc Y 1", "Acc Z 1", "Acc X 2", "Acc Y 2", "Acc Z 2"};
int16_t analogPinValue[8];
uint8_t currentAnalogIn;

// MENU HANDLING VARIABLES
uint8_t currentPatch;
uint16_t currentColor;
uint8_t userFriendlyPatchNumber;

// VARIABLES FOR TOUCHSCREEN DEBOUNCING
boolean touchState;
boolean lastTouchState = false;
long lastDebounceTime = 0;
long debounceDelay = 20; 
boolean touched;

// VARIABLES FOR KEYPAD
uint16_t currentValue = 0;
uint8_t currentEntry = 13;
boolean inKeyPadMode = false;
uint8_t currentValueIdentifier = 0;

//VARIABLES FOR ONSCREEN KEYBOARD
boolean inKeyboardMode = false;
uint8_t currentTextIdentifier = 0;
char charBuffer[8];
char currentChar = 22;
uint8_t cursorPosition = 0;
boolean uppercase = true;

// MIDI HANDLING VARIABLES
const uint8_t nrpnMsb = 99;
const uint8_t nrpnLsb = 98;
const uint8_t rpnMsb = 101;
const uint8_t rpnLsb = 100;
const uint8_t nrpnDivisor = 128;

// LIMITING CONSTANTS
const uint8_t limit7bit = 127;
const int16_t limit10bit = 1023;
const int16_t limit14bit = 16383;


// COLOR DEFINITION
#define     FARBE(r, g, b)              (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))
#define     BLACK                       FARBE(0x00, 0x00, 0x00)
#define     DKGRAY                      FARBE(0x40, 0x40, 0x40)
#define     GRAY                        FARBE(0x80, 0x80, 0x80)
#define     LTGRAY                      FARBE(0xB0, 0xB0, 0xB0)
#define     WHITE                       FARBE(0xFF, 0xFF, 0xFF)
#define     LIME                        FARBE(0x69, 0x9D, 0x05)
#define     COLOR8                      FARBE(0xB4, 0x93, 0xF7)
#define     COLOR1                      FARBE(0xB5, 0xE1, 0x00)
#define     COLOR2                      FARBE(0xE5, 0x00, 0x7D)
#define     COLOR3                      FARBE(0xFA, 0x69, 0x00)
#define     COLOR9                      FARBE(0xFE, 0xAD, 0x12)
#define     COLOR6                      FARBE(0x30, 0xBD, 0x3D)
#define     COLOR5                      FARBE(0xFF, 0xEF, 0x0F)
#define     COLOR7                      FARBE(0xF3, 0x05, 0x40)
#define     COLOR4                      FARBE(0x00, 0xDA, 0xFF)
#define     COLOR0                      FARBE(0x2F, 0xE6, 0x94)

#define	RED 0xF800
#define	GREEN 0x07E0

// GRAPHICS STUFF
uint16_t locator;
uint16_t locator2;
int16_t iBuff;

// SD CARD STUFF
File myFile;
char pName[8];
char pNameB[8];
uint8_t fileId;

//====================================================================================
//-                                                                       VOID setup =
void setup(void) {

  pinMode(LED_PIN, OUTPUT);
  pinMode (errorLED, OUTPUT);
  pinMode (out2LED, OUTPUT);
  pinMode(10, OUTPUT); // must be left as an output or SD library functions will not work

  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
  TWBR = 24; // 400kHz I2C clock
  midiA.begin(); // launch MIDI, by default listening to channel 1
  midiB.begin();
  Serial.begin(115200);

  tft.begin(0x8357);
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  
  // Set the SPI frequency to 1 MHz (on 16 MHz Arduinos), to be safe.
  // DIV2 = 8 MHz works for me, though, even on a breadboard.
  dac.setSPIDivider(SPI_CLOCK_DIV2);
  dac.setPortWrite(false);
  
  if (!SD.begin(SD_CS)) {
    tft.setTextSize(2);
    tft.setTextColor(RED);
    tft.setCursor(120, 8);
    tft.print("SD Card initialization failed!");
    Serial.println(F("SD Card initialization failed!"));
    return;
  }

  mpu.initialize(); //initialize device
  devStatus = mpu.dmpInitialize(); // load and configure the DMP  
}

//====================================================================================
//-                                                                        VOID loop =
void loop() {
  boolean inSetupMode = true;
  
  if (!SD.exists("00.txt")) {
  writeSDPreset();
  delay(1000);
  }
  
  if (!SD.exists("S.txt")) {
  writeSDSettings();
  delay(1000);
  }
  readSDSettings();
  
  // apply offset values
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  mpu.setZAccelOffset(az_offset);

  if (devStatus == 0) { // makes sure it worked (returns 0 if so)
    mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
    attachInterrupt(0, dmpDataReady, RISING); // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true; // set DMP Ready flag so the main loop() function knows it's ok to use it
    packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
  } 
  
  // draw logos, OS version and website
  bmpDraw("mg.bmp", 158, 6);
  tft.setTextSize(2);
  tft.setTextColor(LIME);
  tft.setCursor(380, 62);
  tft.print("OS 0.7.0");
  bmpDraw("limelabs.bmp", 289, 248);
  tft.setCursor(295, 300);
  tft.print("www.limelabs.eu");
  
  // print buttons
  tft.setTextColor(BLACK);
  tft.fillRect(0, 0, 78, 77, LIME);
  tft.setCursor(10, 18);
  tft.print("BLANK");
  tft.setCursor(4, 40);
  tft.print("PRESET");
  tft.fillRect(0, 81, 78, 77, LIME);
  tft.setCursor(16, 99);
  tft.print("LOAD");
  tft.setCursor(4, 121);
  tft.print("PRESET");
  tft.fillRect(0, 162, 78, 77, LIME);
  tft.setCursor(4, 180);
  tft.print("SYSTEM");
  tft.setCursor(10, 202);
  tft.print("SETUP");
  tft.fillRect(0, 243, 78, 77, LIME);
  tft.setCursor(10, 272);
  tft.print("ABOUT");
  
  // touchscreen loop
  while ( inSetupMode == true ) {
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x < 77) {
        if (p.y < 78) { 
          fileId = 00;
          readSDPreset();
          inSetupMode = false;
          switch (liveMode) {
               case 1:
                 liveModeA();
                 break;
               case 2:
                 liveModeB();
                 break;
               case 3:
                 liveModeC();
                 break;
          }
        } 
        else if (p.y < 159) {
          inSetupMode = false;
          loadA(1, 1);
        } 
        else if (p.y < 240) {
          inSetupMode = false;
          // do something if SYSTEM is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.y < 320) {
          inSetupMode = false;
          // do something if ABOUT is hit
          about();
        } 
      }
    }
  }
}

//====================================================================================
//-                                                                   VOID liveModeA =
void liveModeA() {
  boolean isLive = true;
  buildLiveModeA();
  
  // live loop
  while(isLive == true) { 
    touchscreenLiveMode(); 
    
    readAnalogIns(); 

    if(isDmp[0] == true || isDmp[1] == true || isDmp[2] == true || isDmp[3] == true || isDmp[4] == true || isDmp[5] == true || isDmp[6] == true || isDmp[7] == true || isDmp[8] == true || isDmp[9] == true) {
      calculateMotionSendValues(); 
    }
    
    drawLiveValuesA();
    locator = locator + 1;
    if (locator>=479){
      locator = 0;
    }
  }
}

//====================================================================================
//-                                                                   VOID liveModeB =
void liveModeB() {
  boolean isLive = true;
  locator = 97;
  buildLiveModeB();
  
  // live loop
  while(isLive == true) {
    touchscreenLiveModeB(); 
    
    readAnalogIns(); 

    if(isDmp[0] == true || isDmp[1] == true || isDmp[2] == true || isDmp[3] == true || isDmp[4] == true || isDmp[5] == true || isDmp[6] == true || isDmp[7] == true || isDmp[8] == true || isDmp[9] == true) {
      calculateMotionSendValues(); 
    }
    
    drawLiveValuesB();
    locator = locator + 1;
    if (locator >= 381){
      locator = 97;
    }
  }
}

//====================================================================================
//-                                                                   VOID liveModeC =
void liveModeC() {
  boolean isLive = true;
  buildLiveModeC();
  locator = 1;
  locator2 = 241;
  
  // live loop
  while(isLive == true) {
    touchscreenLiveModeC(); 
    
    readAnalogIns(); 

    if(isDmp[0] == true || isDmp[1] == true || isDmp[2] == true || isDmp[3] == true || isDmp[4] == true || isDmp[5] == true || isDmp[6] == true || isDmp[7] == true || isDmp[8] == true || isDmp[9] == true) {
      calculateMotionSendValues(); 
    }
    
    drawLiveValuesC();
    locator = locator + 1;
    locator2 = locator2 +1;
    if (locator>=238){
      locator = 1;
      locator2 = 241;
    }
  }
}

//====================================================================================
//-                                                                  VOID patchSetup =
void patchSetup(int currentPatch){
  boolean inSetupMode = true;
  char arrowDown = 25;

  if(currentPatch == 0){ currentColor = COLOR1; } 
  else if (currentPatch == 1){ currentColor = COLOR2; } 
  else if (currentPatch == 2){ currentColor = COLOR3; } 
  else if (currentPatch == 3){ currentColor = COLOR4; } 
  else if (currentPatch == 4){ currentColor = COLOR5; } 
  else if (currentPatch == 5){ currentColor = COLOR6; } 
  else if (currentPatch == 6){ currentColor = COLOR7; } 
  else if (currentPatch == 7){ currentColor = COLOR8; } 
  else if (currentPatch == 8){ currentColor = COLOR9; } 
  else if (currentPatch == 9){ currentColor = COLOR0; }

  userFriendlyPatchNumber = currentPatch + 1;

  tft.fillScreen(BLACK);

  // build buttons and labelling
  tft.fillRect(4, 4, 234, 48, currentColor);
  tft.setCursor(8, 10); // text for button 1
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("Patch");
  tft.setCursor(8, 31);
  tft.print("Name");
  tft.fillRect(242, 4, 234, 48, currentColor);
  tft.setCursor(247, 10);
  tft.print("Patch");
  tft.fillRect(4, 56, 234, 48, currentColor);
  tft.setCursor(8, 62);
  tft.print("Input");
  tft.fillRect(242, 56, 234, 48, currentColor);
  tft.setCursor(247, 62);
  tft.print("Output");
  tft.fillRect(4, 108, 234, 48, currentColor);
  tft.setCursor(8, 114);
  tft.print("Invert");
  tft.setCursor(8, 135);
  tft.print("Patch");
  tft.fillRect(242, 108, 234, 48, currentColor);
  tft.setCursor(247, 114);
  tft.print("Command");
  tft.fillRect(4, 160, 234, 48, currentColor);
  tft.setCursor(8, 166);
  tft.print("Min In");
  tft.setCursor(8, 187);
  tft.print("Max In");
  tft.fillRect(242, 160, 234, 48, currentColor);
  tft.setCursor(247, 166);
  switch (commandIdentifier[currentPatch]) {
        case 1:
          tft.print("Control");
          tft.setCursor(247, 187);
          tft.print("Change #");
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 2:
          tft.print("MSB #");
          tft.fillRect(242, 212, 234, 48, currentColor);
          tft.setCursor(247, 218);
          tft.print("LSB #");
          break;
        case 3:
          tft.print("MSB #");
          tft.fillRect(242, 212, 234, 48, currentColor);
          tft.setCursor(247, 218);
          tft.print("LSB #");
          break;
        case 4:
          tft.print("MSB #");
          tft.fillRect(242, 212, 234, 48, currentColor);
          tft.setCursor(247, 218);
          tft.print("LSB #");
          break;
        case 5:
          tft.print("MSB #");
          tft.fillRect(242, 212, 234, 48, currentColor);
          tft.setCursor(247, 218);
          tft.print("LSB #");
          break;
        case 6:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 7:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 8:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 9:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 10:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 11:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
        case 12:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
         case 13:
          tft.fillRect(242, 212, 234, 48, currentColor);
          break;
  }

  tft.fillRect(4, 212, 234, 48, currentColor);
  tft.setCursor(8, 218);
  tft.print("Min Out");
  tft.setCursor(8, 239);
  tft.print("Max Out");

  // print Current Values 
  tft.setCursor(108, 18);
  tft.setTextSize(3);
  tft.print(patchNames[currentPatch]);
  tft.setCursor(108, 70);
  tft.print(sensorTypes[sensorTypeIdentifier[currentPatch]]);
  tft.setCursor(215, 122);
  tft.print(arrowDown);
  tft.setTextSize(2);
  if(sensorTypeIdentifier[currentPatch] <=15) {
    tft.setCursor(173, 166);
    for(int x = countDigits(minInValue[currentPatch]); x < 5; x++) {
      tft.print(" ");
    }  
    tft.print(minInValue[currentPatch]);
    tft.setCursor(173, 187);
    for(int x = countDigits(maxInValue[currentPatch]); x < 5; x++) {
      tft.print(" ");
    }
    tft.print(maxInValue[currentPatch]);
  }
  else if(sensorTypeIdentifier[currentPatch] >= 16 && sensorTypeIdentifier[currentPatch] <= 21){
    if(dmpMinInValue[currentPatch] < 0) {
      tft.setCursor(161, 166);
    }
    else {
      tft.setCursor(173, 166);
    }
    tft.print(dmpMinInValue[currentPatch],3);
    if(dmpMaxInValue[currentPatch] >= 0) {
      tft.setCursor(173, 187);
    }
    else {
      tft.setCursor(161, 187);
    }
    tft.print(dmpMaxInValue[currentPatch],3);
  }
  tft.setCursor(173, 218);
  for(int x = countDigits(minOutValue[currentPatch]); x < 5; x++) {
    tft.print(" ");
  }  
  tft.print(minOutValue[currentPatch]);
  tft.setCursor(173, 239);
  for(int x = countDigits(maxOutValue[currentPatch]); x < 5; x++) {
    tft.print(" ");
  }
  tft.print(maxOutValue[currentPatch]);
  tft.setTextSize(3);
  tft.setCursor(436, 18);
  if(userFriendlyPatchNumber < 10) {
    tft.print(" ");
  }
  tft.print(userFriendlyPatchNumber);
  if (midiChan[currentPatch] > 0 && midiChan[currentPatch] < 17){
    tft.setCursor(247, 83);
    tft.setTextSize(2);
    if(commandIdentifier[currentPatch] <= 5) {
      if(isMidiOut1[currentPatch] == true) {
        tft.print("MIDI Out 1 CH");
      }
      if(isMidiOut2[currentPatch] == true) {
        tft.print("MIDI Out 2 CH");
      }
      tft.setTextSize(3);
      tft.setCursor(436, 70);
      if(midiChan[currentPatch] < 10) {
        tft.print("0");
      }
      tft.print(midiChan[currentPatch]);
    }
    else {
      tft.setTextSize(3);
      tft.setCursor(412, 70);
      tft.print("CV");
      tft.print(commandIdentifier[currentPatch]-6);
    }
    tft.setCursor(398, 122);
    switch (commandIdentifier[currentPatch]) {
          case 1:
            tft.print("  CC");
            break;
          case 2:
            tft.print("NRPN");
            tft.setCursor(418, 226);
            for(int x = countDigits(firstDataByteB[currentPatch]); x < 3; x++) {
              tft.print("0");
            }
            tft.print(firstDataByteB[currentPatch]);
            break;
          case 3:
            tft.print(" RPN");
            tft.setCursor(418, 226);
            for(int x = countDigits(firstDataByteB[currentPatch]); x < 3; x++) {
              tft.print("0");
            }
            tft.print(firstDataByteB[currentPatch]);
            break;
          case 4:
            tft.print("NR 7");
            tft.setCursor(418, 226);
            for(int x = countDigits(firstDataByteB[currentPatch]); x < 3; x++) {
              tft.print("0");
            }
            tft.print(firstDataByteB[currentPatch]);
            break;
          case 5:
            tft.print("CC14");
            tft.setCursor(418, 226);
            for(int x = countDigits(firstDataByteB[currentPatch]); x < 3; x++) {
              tft.print("0");
            }
            tft.print(firstDataByteB[currentPatch]);
            break;
          case 6:
            tft.print("  CV");
            break;
          case 7:
            tft.print("  CV");
            break;
          case 8:
            tft.print("  CV");
            break;
          case 9:
            tft.print("  CV");
            break;
          case 10:
            tft.print("  CV");
            break;
          case 11:
            tft.print("  CV");
            break;
          case 12:
            tft.print("  CV");
            break;
          case 13:
            tft.print("  CV");
            break;
    }
    tft.setCursor(418, 174);
    for(int x = countDigits(firstDataByte[currentPatch]); x < 3; x++) {
      tft.print("0");
    }
    tft.print(firstDataByte[currentPatch]);
  }

  // Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 270);
  tft.print("LOAD/SAVE");
  tft.setCursor(127, 291);
  tft.print("PROGRAM");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // touchscreen loop
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
    if (touched != lastTouchState) {
      lastDebounceTime = millis();
    } 
 
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (touched != touchState) {
        touchState = touched;
        if (touchState == true) {
          p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
          p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
          swap(p.x, p.y);
          p.y = (320 - p.y);
      
          if (p.x > 3 && p.x < 238) {
            if (p.y > 3 &&  p.y < 56) { 
              // do something if Patch Name Button is hit
              inSetupMode = false;
              currentTextIdentifier = 1;
              keyboardEntryLoop();
            } 
            else if (p.y > 55 && p.y < 108) {
              // do something if Input Select Button is hit
              inSetupMode = false;
              selectInputTypePage();
            } 
            else if (p.y > 107 && p.y < 160) {
              // do something if Invert Sensor Button is hit
              if(sensorTypeIdentifier[currentPatch] <= 15) {
                invertInt();
                
                tft.fillRect(158, 160, 80, 48, currentColor);
                
                tft.setTextColor(BLACK);
                tft.setTextSize(2);
                tft.setCursor(173, 166);
                for(int x = countDigits(minInValue[currentPatch]); x < 5; x++) {
                  tft.print(" ");
                }  
                tft.print(minInValue[currentPatch]);
                tft.setCursor(173, 187);
                for(int x = countDigits(maxInValue[currentPatch]); x < 5; x++) {
                  tft.print(" ");
                }
                tft.print(maxInValue[currentPatch]);
              }
              
              else if(sensorTypeIdentifier[currentPatch] >= 16 && sensorTypeIdentifier[currentPatch] <= 21) {
                invertFloat();
                
                tft.fillRect(158, 160, 80, 48, currentColor);
                
                tft.setTextColor(BLACK);
                tft.setTextSize(2);
                if(dmpMinInValue[currentPatch] < 0) {
                  tft.setCursor(161, 166);
                }
                else {
                  tft.setCursor(173, 166);
                }
                tft.print(dmpMinInValue[currentPatch],3);
                if(dmpMaxInValue[currentPatch] >= 0) {
                  tft.setCursor(173, 187);
                }
                else {
                  tft.setCursor(161, 187);
                }
                tft.print(dmpMaxInValue[currentPatch],3);
              }
            } 
            
            else if (p.y > 159 && p.y < 212) {
              // do something if Min In Max In Button is hit
              inSetupMode = false;
              if(sensorTypeIdentifier[currentPatch] <= 15) {
                setInputRange();
              }
              else if(sensorTypeIdentifier[currentPatch] == 16 || sensorTypeIdentifier[currentPatch] == 17 || sensorTypeIdentifier[currentPatch] == 19 || sensorTypeIdentifier[currentPatch] == 20) {
                dmpRangeLearn();
              }
            } 
            else if (p.y > 211 && p.y < 264) {
              // do something if Min Out Max Out Button is hit
              inSetupMode = false;
              setOutputRangePage();
            } 
          } 
          else if (p.x > 243 && p.x < 447) {
            if (p.y > 3 &&  p.y < 56) { 
              // do something if Patch Number Button is hit
              inSetupMode = false;
              changeActivePatchPage();
            } 
            else if (p.y > 55 && p.y < 108) {
              // do something if Output Select Button is hit
              inSetupMode = false;
              selectOutput();
            } 
            else if (p.y > 107 && p.y < 160) {
              // do something if Command Button is hit
              if(commandIdentifier[currentPatch] <= 5) {
                inSetupMode = false;
                selectCommandPage();
              }
            } 
            else if (p.y > 159 && p.y < 212) {
              // do something if CC#/MSB# button is hit
              inSetupMode = false;
              setMsbPage();
            } 
            else if (p.y > 211 && p.y < 264) {
              // do something if Command is NRPN or RPN and this button is hit
              if(commandIdentifier[currentPatch] == 2 || commandIdentifier[currentPatch] == 3 || commandIdentifier[currentPatch] == 4 || commandIdentifier[currentPatch] == 5) {
                inSetupMode = false;
                setLsbPage();
              }
            } 
          }
          if (p.y > 263 && p.y < 320) {
            if (p.x > 3 &&  p.x < 119) { 
              // do something if SYSTEM SETUP button is hit
              inSetupMode = false;
              systemSetup();
            } 
            else if (p.x > 122 && p.x < 238) {
              // do something if LOAD/SAVE PROGRAM is hit
              inSetupMode = false;
              loadSave();
            } 
            else if (p.x > 241 && p.x < 476) {
              // do something if LIVE MODE Button is hit
              switch (liveMode) {
                    case 1:
                      inSetupMode = false;
                      liveModeA();
                      break;
                    case 2:
                      inSetupMode = false;
                      liveModeB();
                      break;
                    case 3:
                      inSetupMode = false;
                      liveModeC();
                      break;
              }
            } 
          }
        }
      }
    }
    lastTouchState = touched;
    if (p.z < MINPRESSURE) { touched = false; }
  }
}

//====================================================================================
//-                                                                 VOID systemSetup =
void systemSetup(){
  boolean inSetupMode = true;
  tft.fillScreen(BLACK);

  // build buttons and labelling
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.setCursor(126, 16);
  tft.print("System Setup");
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  tft.fillRect(4, 56, 234, 100, LIME);
  tft.setCursor(8, 62);
  tft.print("- - - ");
  tft.fillRect(242, 56, 234, 100, LIME);
  tft.setCursor(246, 62);
  tft.print("Select Live Mode");
  tft.fillRect(4, 160, 234, 100, LIME);
  tft.setCursor(8, 164);
  tft.print("Color Schemes");
  tft.fillRect(242, 160, 234, 100, LIME);
  tft.setCursor(246, 164);
  tft.print("Calibrate MSU");
  
  // print Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 270);
  tft.print("PATCH");
  tft.setCursor(50, 291);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 270);
  tft.print("LOAD/SAVE");
  tft.setCursor(127, 291);
  tft.print("PROGRAM");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");
  
  // touchscreen loop
  while ( inSetupMode == true ) {
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x < 240) {
        if (p.y > 55 &&  p.y < 158) { 
          //
        } 
        else if (p.y > 157 && p.y < 264) {
          //
        } 
      }
      else if (p.x > 239) {
        if (p.y > 55 &&  p.y < 158) { 
          selectLiveMode();
        } 
        else if (p.y > 157 && p.y < 264) {
          // do something if CALIBRATE MSU is hit
          inSetupMode = false;
          calibrateMSU();
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if PATCH SETUP button is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if LOAD/SAVE PROGRAM is hit
          inSetupMode = false;
          loadSave();
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                              VOID selectLiveMode =
void selectLiveMode() {
  boolean inSetupMode = true;
  tft.fillScreen(BLACK);

  // build buttons and labelling
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.setCursor(126, 16);
  tft.print("MIDI Garden - hello");
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  tft.fillRect(4, 56, 234, 100, LIME);
  tft.setCursor(8, 62);
  tft.print("Live A");
  tft.fillRect(242, 56, 234, 100, LIME);
  tft.setCursor(246, 62);
  tft.print("Live B");
  tft.fillRect(4, 160, 234, 100, LIME);
  tft.setCursor(8, 164);
  tft.print("Live C");
  tft.fillRect(242, 160, 234, 100, LIME);
  tft.setCursor(246, 164);
  tft.print("System Setup");
  
  // touch screen loop
  while ( inSetupMode == true ) {
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x < 240) {
        if (p.y > 55 &&  p.y < 158) { 
          liveMode = 1;
          liveModeA();
          break;
        } 
        else if (p.y > 157 && p.y < 264) {
          liveMode = 3;
          liveModeC();
          break;
        } 
      }
      else if (p.x > 239) {
        if (p.y > 55 &&  p.y < 158) { 
          liveMode = 2;
          liveModeB();
          break;
        } 
        else if (p.y > 157 && p.y < 264) {
          //
        } 
      }
    }
  }
}

//====================================================================================
//-                                                                    VOID loadSave =
void loadSave(){
  boolean inSetupMode = true;
  tft.fillScreen(BLACK);

  // buttons and labelling
  tft.setTextColor(LIME);
  tft.setTextSize(3);
  tft.setCursor(86, 16);
  tft.print("Load/Save Program");
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.fillRect(4, 56, 234, 204, LIME);
  tft.setCursor(8, 62);
  tft.print("LOAD");
  tft.fillRect(242, 56, 234, 204, LIME);
  tft.setCursor(246, 62);
  tft.print("SAVE");
  
  // Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 270);
  tft.print("LOAD/SAVE");
  tft.setCursor(127, 291);
  tft.print("PROGRAM");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");
  
  while ( inSetupMode == true ) {
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x < 240) {
        if (p.y > 55 &&  p.y < 261) { 
          // do something if LOAD button is hit
          inSetupMode = false;
          loadA(1, 2);
        } 
      }
      else if (p.x > 239) {
        if (p.y > 55 &&  p.y < 261) { 
          // do something if SAVE button is hit
          inSetupMode = false;
          currentTextIdentifier = 2;
          keyboardEntryLoop();
        } 
      }
    }
  }
}

//====================================================================================
//                                                                        VOID loadA =
void loadA(uint8_t pId, uint8_t origin){

  boolean inSetupMode = true; // tells the program to remain in the while loop
  char lArr = 27;
  char rArr = 26;

  tft.fillScreen(BLACK);
  
  // title
  tft.setTextColor(LIME);
  tft.setTextSize(3);  
  tft.setCursor(9, 17);
  tft.print("LOAD PRESET");
  
  // print navigation
  tft.drawRect(242, 4, 63, 48, LIME);
  tft.setCursor(264, 17);
  tft.print(lArr);
  tft.drawRect(309, 4, 100, 48, LIME);
  tft.setTextSize(2); 
  tft.setCursor(326, 21);
  tft.print("CANCEL");
  tft.setTextSize(3); 
  tft.drawRect(413, 4, 63, 48, LIME);
  tft.setCursor(435, 17);
  tft.print(rArr);
  
  // print preset buttons
  tft.fillRect(4, 56, 234, 48, LIME);
  tft.setTextColor(BLACK);
  tft.setCursor(9, 69); 
  switch (pId) {
        case 1:
          fileId = 1;
          tft.print("01");
          break;
        case 2:
          fileId = 11;
          tft.print("11");
          break;
        case 3:
          fileId = 21;
          tft.print("21");
          break;
        case 4:
          fileId = 31;
          tft.print("31");
          break;
        case 5:
          fileId = 41;
          tft.print("41");
          break;
  }
  readSDpName();
  tft.setCursor(56, 69);
  tft.print(pName);
  tft.fillRect(242, 56, 234, 48, LIME);
  tft.setCursor(247, 69);
  switch (pId) {
        case 1:
          fileId = 6;
          tft.print("06");
          break;
        case 2:
          fileId = 16;
          tft.print("16");
          break;
        case 3:
          fileId = 26;
          tft.print("26");
          break;
        case 4:
          fileId = 36;
          tft.print("36");
          break;
        case 5:
          fileId = 46;
          tft.print("46");
          break;
  }
  readSDpName();
  tft.setCursor(294, 69);
  tft.print(pName);
  tft.fillRect(4, 108, 234, 48, LIME);
  tft.setCursor(9, 121);
  switch (pId) {
        case 1:
          fileId = 2;
          tft.print("02");
          break;
        case 2:
          fileId = 12;
          tft.print("12");
          break;
        case 3:
          fileId = 22;
          tft.print("22");
          break;
        case 4:
          fileId = 32;
          tft.print("32");
          break;
        case 5:
          fileId = 42;
          tft.print("42");
          break;
  }
  readSDpName();
  tft.setCursor(56, 121);
  tft.print(pName);
  tft.fillRect(242, 108, 234, 48, LIME);
  tft.setCursor(247, 121);
  switch (pId) {
        case 1:
          fileId = 7;
          tft.print("07");
          break;
        case 2:
          fileId = 17;
          tft.print("17");
          break;
        case 3:
          fileId = 27;
          tft.print("27");
          break;
        case 4:
          fileId = 37;
          tft.print("37");
          break;
        case 5:
          fileId = 47;
          tft.print("47");
          break;
  }
  readSDpName();
  tft.setCursor(294, 121);
  tft.print(pName);
  tft.fillRect(4, 160, 234, 48, LIME);
  tft.setCursor(9, 173);
  switch (pId) {
        case 1:
          fileId = 3;
          tft.print("03");
          break;
        case 2:
          fileId = 13;
          tft.print("13");
          break;
        case 3:
          fileId = 23;
          tft.print("23");
          break;
        case 4:
          fileId = 33;
          tft.print("33");
          break;
        case 5:
          fileId = 43;
          tft.print("43");
          break;
  }
  readSDpName();
  tft.setCursor(56, 173);
  tft.print(pName);
  tft.fillRect(242, 160, 234, 48, LIME);
  tft.setCursor(247, 173);
  switch (pId) {
        case 1:
          fileId = 8;
          tft.print("08");
          break;
        case 2:
          fileId = 18;
          tft.print("18");
          break;
        case 3:
          fileId = 28;
          tft.print("28");
          break;
        case 4:
          fileId = 38;
          tft.print("38");
          break;
        case 5:
          fileId = 48;
          tft.print("48");
          break;
  }
  readSDpName();
  tft.setCursor(294, 173);
  tft.print(pName);
  tft.fillRect(4, 212, 234, 48, LIME);
  tft.setCursor(9, 225);
  switch (pId) {
        case 1:
          fileId = 4;
          tft.print("04");
          break;
        case 2:
          fileId = 14;
          tft.print("14");
          break;
        case 3:
          fileId = 24;
          tft.print("24");
          break;
        case 4:
          fileId = 34;
          tft.print("34");
          break;
        case 5:
          fileId = 44;
          tft.print("44");
          break;
  }
  readSDpName();
  tft.setCursor(56, 225);
  tft.print(pName);
  tft.fillRect(242, 212, 234, 48, LIME);
  tft.setCursor(247, 225);
  switch (pId) {
        case 1:
          fileId = 9;
          tft.print("09");
          break;
        case 2:
          fileId = 19;
          tft.print("19");
          break;
        case 3:
          fileId = 29;
          tft.print("29");
          break;
        case 4:
          fileId = 39;
          tft.print("39");
          break;
        case 5:
          fileId = 49;
          tft.print("49");
          break;
  }
  readSDpName();
  tft.setCursor(294, 225);
  tft.print(pName);
  tft.fillRect(4, 264, 234, 48, LIME);
  tft.setCursor(9, 277);
  switch (pId) {
        case 1:
          fileId = 5;
          tft.print("05");
          break;
        case 2:
          fileId = 15;
          tft.print("15");
          break;
        case 3:
          fileId = 25;
          tft.print("25");
          break;
        case 4:
          fileId = 35;
          tft.print("35");
          break;
        case 5:
          fileId = 45;
          tft.print("45");
          break;
  }
  readSDpName();
  tft.setCursor(56, 277);
  tft.print(pName);
  tft.fillRect(242, 264, 234, 48, LIME);
  tft.setCursor(247, 277);
  switch (pId) {
        case 1:
          fileId = 10;
          tft.print("10");
          break;
        case 2:
          fileId = 20;
          tft.print("20");
          break;
        case 3:
          fileId = 30;
          tft.print("30");
          break;
        case 4:
          fileId = 40;
          tft.print("40");
          break;
        case 5:
          fileId = 50;
          tft.print("50");
          break;
  }
  readSDpName();
  tft.setCursor(294, 277);
  tft.print(pName);

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
    if (touched != lastTouchState) {
      lastDebounceTime = millis();
    } 
  
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (touched != touchState) {
        touchState = touched;
        if (touchState == true) {
          p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
          p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
          swap(p.x, p.y);
          p.y = (320 - p.y);
          if (p.y < 55) {
            if (p.x > 239 &&  p.x < 308) { 
              // do something if arrowLeft is pressed
              if (pId > 1) {
                inSetupMode = false;
                pId--;
                loadA(pId, origin);
              }
              else {
                pId = 5;
                loadA(pId, origin);
              }
            } 
            else if (p.x > 307 &&  p.x < 412) { 
              // do something if CANCEL is pressed
              if (origin == 1) {
                inSetupMode = false;
                loop();
              }
              else {
                inSetupMode = false;
                loadSave();
              }
            } 
            else if (p.x > 411) { 
              // do something if arrowRight is pressed
              if (pId < 5) {
                inSetupMode = false;
                pId++;
                loadA(pId, origin);
              }
              else {
                pId = 1;
                loadA(pId, origin);
              }
            } 
          }
          else if (p.x > 3 && p.x < 240) {
            if (p.y > 55 &&  p.y < 108) { 
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 1;
                      break;
                    case 2:
                      fileId = 11;
                      break;
                    case 3:
                      fileId = 21;
                      break;
                    case 4:
                      fileId = 31;
                      break;
                    case 5:
                      fileId = 41;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 107 && p.y < 160) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 2;
                      break;
                    case 2:
                      fileId = 12;
                      break;
                    case 3:
                      fileId = 22;
                      break;
                    case 4:
                      fileId = 32;
                      break;
                    case 5:
                      fileId = 42;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 159 && p.y < 212) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 3;
                      break;
                    case 2:
                      fileId = 13;
                      break;
                    case 3:
                      fileId = 23;
                      break;
                    case 4:
                      fileId = 33;
                      break;
                    case 5:
                      fileId = 43;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 211 && p.y < 264) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 4;
                      break;
                    case 2:
                      fileId = 14;
                      break;
                    case 3:
                      fileId = 24;
                      break;
                    case 4:
                      fileId = 34;
                      break;
                    case 5:
                      fileId = 44;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 263 && p.y < 316) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 5;
                      break;
                    case 2:
                      fileId = 15;
                      break;
                    case 3:
                      fileId = 25;
                      break;
                    case 4:
                      fileId = 35;
                      break;
                    case 5:
                      fileId = 45;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
          }
          else if (p.x > 239 && p.x < 480) {
            if (p.y > 55 &&  p.y < 108) { 
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 6;
                      break;
                    case 2:
                      fileId = 16;
                      break;
                    case 3:
                      fileId = 26;
                      break;
                    case 4:
                      fileId = 36;
                      break;
                    case 5:
                      fileId = 46;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 107 && p.y < 160) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 7;
                      break;
                    case 2:
                      fileId = 17;
                      break;
                    case 3:
                      fileId = 27;
                      break;
                    case 4:
                      fileId = 37;
                      break;
                    case 5:
                      fileId = 47;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 159 && p.y < 212) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 8;
                      break;
                    case 2:
                      fileId = 18;
                      break;
                    case 3:
                      fileId = 28;
                      break;
                    case 4:
                      fileId = 38;
                      break;
                    case 5:
                      fileId = 48;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 211 && p.y < 264) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 9;
                      break;
                    case 2:
                      fileId = 19;
                      break;
                    case 3:
                      fileId = 29;
                      break;
                    case 4:
                      fileId = 39;
                      break;
                    case 5:
                      fileId = 49;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
            else if (p.y > 263 && p.y < 316) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 10;
                      break;
                    case 2:
                      fileId = 20;
                      break;
                    case 3:
                      fileId = 30;
                      break;
                    case 4:
                      fileId = 40;
                      break;
                    case 5:
                      fileId = 50;
                      break;
              }
              readSDPreset();
              switch(liveMode) {
                    case 1:
                      liveModeA();
                      break;
                    case 2: 
                      liveModeB();
                      break;
                    case 3: 
                      liveModeC();
                      break;
              }
            } 
          }
        }
      }
    }
    lastTouchState = touched;
    if (p.z < MINPRESSURE) { touched = false; }
  }
}

//====================================================================================
//                                                                        VOID saveA =
void saveA(uint8_t pId){

  boolean inSetupMode = true; // tells the program to remain in the while loop
  char lArr = 27;
  char rArr = 26;

  tft.fillScreen(BLACK);
  
  // title
  tft.setTextColor(LIME);
  tft.setTextSize(3);  
  tft.setCursor(9, 17);
  tft.print("SAVE ");
  tft.print(pName);
  
  // print navigation
  tft.drawRect(242, 4, 63, 48, LIME);
  tft.setCursor(264, 17);
  tft.print(lArr);
  tft.drawRect(309, 4, 100, 48, LIME);
  tft.setTextSize(2); 
  tft.setCursor(326, 21);
  tft.print("CANCEL");
  tft.setTextSize(3); 
  tft.drawRect(413, 4, 63, 48, LIME);
  tft.setCursor(435, 17);
  tft.print(rArr);
  
  // print preset buttons
  tft.fillRect(4, 56, 234, 48, LIME);
  tft.setTextColor(BLACK);
  tft.setCursor(9, 69); 
  switch (pId) {
        case 1:
          fileId = 1;
          tft.print("01");
          break;
        case 2:
          fileId = 11;
          tft.print("11");
          break;
        case 3:
          fileId = 21;
          tft.print("21");
          break;
        case 4:
          fileId = 31;
          tft.print("31");
          break;
        case 5:
          fileId = 41;
          tft.print("41");
          break;
  }
  readSDpName();
  tft.setCursor(56, 69);
  tft.print(pName);
  tft.fillRect(242, 56, 234, 48, LIME);
  tft.setCursor(247, 69);
  switch (pId) {
        case 1:
          fileId = 6;
          tft.print("06");
          break;
        case 2:
          fileId = 16;
          tft.print("16");
          break;
        case 3:
          fileId = 26;
          tft.print("26");
          break;
        case 4:
          fileId = 36;
          tft.print("36");
          break;
        case 5:
          fileId = 46;
          tft.print("46");
          break;
  }
  readSDpName();
  tft.setCursor(294, 69);
  tft.print(pName);
  tft.fillRect(4, 108, 234, 48, LIME);
  tft.setCursor(9, 121);
  switch (pId) {
        case 1:
          fileId = 2;
          tft.print("02");
          break;
        case 2:
          fileId = 12;
          tft.print("12");
          break;
        case 3:
          fileId = 22;
          tft.print("22");
          break;
        case 4:
          fileId = 32;
          tft.print("32");
          break;
        case 5:
          fileId = 42;
          tft.print("42");
          break;
  }
  readSDpName();
  tft.setCursor(56, 121);
  tft.print(pName);
  tft.fillRect(242, 108, 234, 48, LIME);
  tft.setCursor(247, 121);
  switch (pId) {
        case 1:
          fileId = 7;
          tft.print("07");
          break;
        case 2:
          fileId = 17;
          tft.print("17");
          break;
        case 3:
          fileId = 27;
          tft.print("27");
          break;
        case 4:
          fileId = 37;
          tft.print("37");
          break;
        case 5:
          fileId = 47;
          tft.print("47");
          break;
  }
  readSDpName();
  tft.setCursor(294, 121);
  tft.print(pName);
  tft.fillRect(4, 160, 234, 48, LIME);
  tft.setCursor(9, 173);
  switch (pId) {
        case 1:
          fileId = 3;
          tft.print("03");
          break;
        case 2:
          fileId = 13;
          tft.print("13");
          break;
        case 3:
          fileId = 23;
          tft.print("23");
          break;
        case 4:
          fileId = 33;
          tft.print("33");
          break;
        case 5:
          fileId = 43;
          tft.print("43");
          break;
  }
  readSDpName();
  tft.setCursor(56, 173);
  tft.print(pName);
  tft.fillRect(242, 160, 234, 48, LIME);
  tft.setCursor(247, 173);
  switch (pId) {
        case 1:
          fileId = 8;
          tft.print("08");
          break;
        case 2:
          fileId = 18;
          tft.print("18");
          break;
        case 3:
          fileId = 28;
          tft.print("28");
          break;
        case 4:
          fileId = 38;
          tft.print("38");
          break;
        case 5:
          fileId = 48;
          tft.print("48");
          break;
  }
  readSDpName();
  tft.setCursor(294, 173);
  tft.print(pName);
  tft.fillRect(4, 212, 234, 48, LIME);
  tft.setCursor(9, 225);
  switch (pId) {
        case 1:
          fileId = 4;
          tft.print("04");
          break;
        case 2:
          fileId = 14;
          tft.print("14");
          break;
        case 3:
          fileId = 24;
          tft.print("24");
          break;
        case 4:
          fileId = 34;
          tft.print("34");
          break;
        case 5:
          fileId = 44;
          tft.print("44");
          break;
  }
  readSDpName();
  tft.setCursor(56, 225);
  tft.print(pName);
  tft.fillRect(242, 212, 234, 48, LIME);
  tft.setCursor(247, 225);
  switch (pId) {
        case 1:
          fileId = 9;
          tft.print("09");
          break;
        case 2:
          fileId = 19;
          tft.print("19");
          break;
        case 3:
          fileId = 29;
          tft.print("29");
          break;
        case 4:
          fileId = 39;
          tft.print("39");
          break;
        case 5:
          fileId = 49;
          tft.print("49");
          break;
  }
  readSDpName();
  tft.setCursor(294, 225);
  tft.print(pName);
  tft.fillRect(4, 264, 234, 48, LIME);
  tft.setCursor(9, 277);
  switch (pId) {
        case 1:
          fileId = 5;
          tft.print("05");
          break;
        case 2:
          fileId = 15;
          tft.print("15");
          break;
        case 3:
          fileId = 25;
          tft.print("25");
          break;
        case 4:
          fileId = 35;
          tft.print("35");
          break;
        case 5:
          fileId = 45;
          tft.print("45");
          break;
  }
  readSDpName();
  tft.setCursor(56, 277);
  tft.print(pName);
  tft.fillRect(242, 264, 234, 48, LIME);
  tft.setCursor(247, 277);
  switch (pId) {
        case 1:
          fileId = 10;
          tft.print("10");
          break;
        case 2:
          fileId = 20;
          tft.print("20");
          break;
        case 3:
          fileId = 30;
          tft.print("30");
          break;
        case 4:
          fileId = 40;
          tft.print("40");
          break;
        case 5:
          fileId = 50;
          tft.print("50");
          break;
  }
  readSDpName();
  tft.setCursor(294, 277);
  tft.print(pName);

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
    if (touched != lastTouchState) {
      lastDebounceTime = millis();
    } 
  
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (touched != touchState) {
        touchState = touched;
        if (touchState == true) {
          p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
          p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
          swap(p.x, p.y);
          p.y = (320 - p.y);
          if (p.y < 55) {
            if (p.x > 239 &&  p.x < 308) { 
              // do something if arrowLeft is pressed
              if (pId > 1) {
                inSetupMode = false;
                pId--;
                saveA(pId);
              }
              else {
                pId = 5;
                saveA(pId);
              }
            } 
            else if (p.x > 307 &&  p.x < 412) { 
              // do something if CANCEL is pressed
              inSetupMode = false;
              loadSave();
            } 
            else if (p.x > 411) { 
              // do something if arrowRight is pressed
              if (pId < 5) {
                inSetupMode = false;
                pId++;
                saveA(pId);
              }
              else {
                pId = 1;
                saveA(pId);
              }
            } 
          }
          // SAVE PRESET BUTTONS
          else if (p.x > 3 && p.x < 240) {
            if (p.y > 55 &&  p.y < 108) { 
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 1;
                      break;
                    case 2:
                      fileId = 11;
                      break;
                    case 3:
                      fileId = 21;
                      break;
                    case 4:
                      fileId = 31;
                      break;
                    case 5:
                      fileId = 41;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 107 && p.y < 160) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 2;
                      break;
                    case 2:
                      fileId = 12;
                      break;
                    case 3:
                      fileId = 22;
                      break;
                    case 4:
                      fileId = 32;
                      break;
                    case 5:
                      fileId = 42;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 159 && p.y < 212) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 3;
                      break;
                    case 2:
                      fileId = 13;
                      break;
                    case 3:
                      fileId = 23;
                      break;
                    case 4:
                      fileId = 33;
                      break;
                    case 5:
                      fileId = 43;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 211 && p.y < 264) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 4;
                      break;
                    case 2:
                      fileId = 14;
                      break;
                    case 3:
                      fileId = 24;
                      break;
                    case 4:
                      fileId = 34;
                      break;
                    case 5:
                      fileId = 44;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 263 && p.y < 316) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 5;
                      break;
                    case 2:
                      fileId = 15;
                      break;
                    case 3:
                      fileId = 25;
                      break;
                    case 4:
                      fileId = 35;
                      break;
                    case 5:
                      fileId = 45;
                      break;
              }
              confirmOverwrite();
            } 
          }
          else if (p.x > 239 && p.x < 480) {
            if (p.y > 55 &&  p.y < 108) { 
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 6;
                      break;
                    case 2:
                      fileId = 16;
                      break;
                    case 3:
                      fileId = 26;
                      break;
                    case 4:
                      fileId = 36;
                      break;
                    case 5:
                      fileId = 46;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 107 && p.y < 160) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 7;
                      break;
                    case 2:
                      fileId = 17;
                      break;
                    case 3:
                      fileId = 27;
                      break;
                    case 4:
                      fileId = 37;
                      break;
                    case 5:
                      fileId = 47;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 159 && p.y < 212) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 8;
                      break;
                    case 2:
                      fileId = 18;
                      break;
                    case 3:
                      fileId = 28;
                      break;
                    case 4:
                      fileId = 38;
                      break;
                    case 5:
                      fileId = 48;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 211 && p.y < 264) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 9;
                      break;
                    case 2:
                      fileId = 19;
                      break;
                    case 3:
                      fileId = 29;
                      break;
                    case 4:
                      fileId = 39;
                      break;
                    case 5:
                      fileId = 49;
                      break;
              }
              confirmOverwrite();
            } 
            else if (p.y > 263 && p.y < 316) {
              inSetupMode = false;
              switch (pId) {
                    case 1:
                      fileId = 10;
                      break;
                    case 2:
                      fileId = 20;
                      break;
                    case 3:
                      fileId = 30;
                      break;
                    case 4:
                      fileId = 40;
                      break;
                    case 5:
                      fileId = 50;
                      break;
              }
              confirmOverwrite();
            } 
          }
        }
      }
    }
    lastTouchState = touched;
    if (p.z < MINPRESSURE) { touched = false; }
  }
}

//====================================================================================
//-                                                            VOID confirmOverwrite =
void confirmOverwrite(){
  boolean inSetupMode = true;
  tft.fillScreen(BLACK);
  
  readSDpName();

  // build buttons and labelling
  tft.drawRect(4, 4, 472, 48, RED);
  tft.setTextColor(RED);
  tft.setTextSize(3);
  tft.setCursor(16, 16);
  tft.print("WARNING!");
  tft.setTextSize(2);
  tft.setTextColor(LTGRAY);
  tft.setCursor(8, 84);
  tft.print("Do you really want to overwrite");
  tft.setCursor(8, 106);
  tft.print("Preset ");
  if(fileId < 10) tft.print("0");
  tft.print(fileId);
  tft.print(" ");
  tft.setTextColor(GREEN);
  tft.print(pName);
  tft.setTextColor(LTGRAY);
  tft.setCursor(8, 128);
  tft.print("with ");
  tft.setTextColor(GREEN);
  tft.print(pNameB);
  tft.setTextColor(LTGRAY);
  tft.print("?");
  tft.setCursor(8, 162);
  tft.print("Once you hit YES there is no way back!");
  tft.setCursor(8, 206);
  tft.print("Proceed?");
  
  // print Bottom Menu Buttons
  tft.fillRect(4, 264, 234, 48, RED);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(50, 291);
  tft.print("NO");
  tft.fillRect(242, 264, 234, 48, GREEN);
  tft.setCursor(364, 291);
  tft.print("YES");
  
  // touchscreen loop
  while ( inSetupMode == true ) {
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.y > 263 && p.y < 320) {
        if (p.x < 240) {
          // do something if NO is hit
          inSetupMode = false;
          saveA(1);
        } 
        else if (p.x >= 240) {
          // do something if YES is hit
          inSetupMode = false;
          deleteSDFile();
          writeSDPreset();
          switch (liveMode) {
                case 1:
                  liveModeA();
                  break;
                case 2:
                  liveModeB();
                  break;
                case 3:
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                         VOID selectInputTypePage =
void selectInputTypePage(){
  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Select Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Input");
  tft.fillRect(4, 56, 115, 100, currentColor);

  // build buttons
  tft.setTextSize(2);
  tft.setTextColor(BLACK);
  tft.fillRect(4, 56, 234, 100, currentColor);
  tft.setCursor(8, 62);
  tft.print("Gyro");
  tft.fillRect(242, 56, 234, 100, currentColor);
  tft.setCursor(246, 62);
  tft.print("Acceleration");
  tft.fillRect(4, 160, 234, 100, currentColor);
  tft.setCursor(8, 164);
  tft.print("Analog Sensor");
  tft.fillRect(242, 160, 234, 100, currentColor);
  tft.setCursor(246, 164);
  tft.print("CV Input");

  // build bottom menu buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  //TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x < 240) {
        if (p.y > 55 &&  p.y < 158) { 
          // do something if GYRO is hit
          inSetupMode = false;
          selectGyroInputPage();
          break;
        } 
        else if (p.y > 157 && p.y < 264) {
          // do something if ANALOG SENSOR is hit
          inSetupMode = false;
          selectAnalogInputPage();          
          break;
        } 
      }
      else if (p.x > 239) {
        if (p.y > 55 &&  p.y < 158) { 
          // do something if ACCELERATION is hit
          inSetupMode = false;
          selectAccInputPage();
          break;
        } 
        else if (p.y > 157 && p.y < 264) {
          // do something if CV INPUT is hit
          inSetupMode = false;
          selectCvInputPage();
          break;
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                       VOID selectAnalogInputPage =
void selectAnalogInputPage(){
  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);
  
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Select Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Input");
  tft.fillRect(4, 56, 115, 100, currentColor);
  
  // build buttons and their writing
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print(sensorTypes[0]);
  tft.fillRect(123, 56, 115, 100, currentColor);
  tft.setCursor(127, 72);
  tft.print(sensorTypes[1]);
  tft.fillRect(4, 160, 115, 100, currentColor);
  tft.setCursor(8, 176);
  tft.print(sensorTypes[4]);
  tft.fillRect(123, 160, 115, 100, currentColor);
  tft.setCursor(127, 176);
  tft.print(sensorTypes[5]);

  tft.fillRect(242, 56, 115, 100, currentColor);
  tft.setCursor(246, 72);
  tft.print(sensorTypes[2]);
  tft.fillRect(361, 56, 115, 100, currentColor);
  tft.setCursor(365, 72);
  tft.print(sensorTypes[3]);
  tft.fillRect(242, 160, 115, 100, currentColor);
  tft.setCursor(246, 176);
  tft.print(sensorTypes[6]);
  tft.fillRect(361, 160, 115, 100, currentColor);
  tft.setCursor(365, 176);
  tft.print(sensorTypes[7]);

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  //TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 119) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 0; // sets Identifier to A-0 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 4; // sets Identifier to A-4 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 122 && p.x < 238) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 1; // sets Identifier to A-1 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 5; // sets Identifier to A-5 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 243 && p.x < 359) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 2; // sets Identifier to A-2
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 6; // sets Identifier to A-6
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 359 && p.x < 479) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 3; // sets Identifier to A-3
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 7; // sets Identifier to A-7
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                           VOID selectCvInputPage =
void selectCvInputPage(){
  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Select Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Input");
  tft.fillRect(4, 56, 115, 100, currentColor);
  
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print(sensorTypes[8]);
  tft.fillRect(123, 56, 115, 100, currentColor);
  tft.setCursor(127, 72);
  tft.print(sensorTypes[9]);
  tft.fillRect(4, 160, 115, 100, currentColor);
  tft.setCursor(8, 176);
  tft.print(sensorTypes[12]);
  tft.fillRect(123, 160, 115, 100, currentColor);
  tft.setCursor(127, 176);
  tft.print(sensorTypes[13]);

  tft.fillRect(242, 56, 115, 100, currentColor);
  tft.setCursor(246, 72);
  tft.print(sensorTypes[10]);
  tft.fillRect(361, 56, 115, 100, currentColor);
  tft.setCursor(365, 72);
  tft.print(sensorTypes[11]);
  tft.fillRect(242, 160, 115, 100, currentColor);
  tft.setCursor(246, 176);
  tft.print(sensorTypes[14]);
  tft.fillRect(361, 160, 115, 100, currentColor);
  tft.setCursor(365, 176);
  tft.print(sensorTypes[15]);

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  //TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 119) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 8; // sets Identifier to CV0 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 12; // sets Identifier to CV4 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 122 && p.x < 238) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 9; // sets Identifier to CV1 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 13; // sets Identifier to CV5 
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 243 && p.x < 359) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 10; // sets Identifier to CV2
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 14; // sets Identifier to CV6
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 359 && p.x < 479) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 11; // sets Identifier to CV3
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 15; // sets Identifier to CV7
          isDmp[currentPatch] = false;
          patchSetup(currentPatch);
          break;
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                         VOID selectGyroInputPage =
void selectGyroInputPage(){
  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Select Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Input");
  tft.fillRect(4, 56, 115, 100, currentColor);
  
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print(sensorTypes[18]); // Yaw 1
  tft.fillRect(123, 56, 115, 100, currentColor);
  tft.setCursor(127, 72);
  tft.print(sensorTypes[17]); // Pitch 1
  tft.fillRect(4, 160, 115, 100, currentColor);
  tft.setCursor(8, 176);
  tft.print(sensorTypes[21]); // Yaw 2
  tft.fillRect(123, 160, 115, 100, currentColor);
  tft.setCursor(127, 176);
  tft.print(sensorTypes[20]); // Pitch 2

  tft.fillRect(242, 56, 115, 100, currentColor);
  tft.setCursor(246, 72);
  tft.print(sensorTypes[16]); // Roll 1
  tft.fillRect(361, 56, 115, 100, currentColor);

  tft.fillRect(242, 160, 115, 100, currentColor);
  tft.setCursor(246, 176);
  tft.print(sensorTypes[19]); // Roll 2
  tft.fillRect(361, 160, 115, 100, currentColor);

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  //TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 119) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 18; // sets Identifier to Yaw 1
          isDmp[currentPatch] = true;
          dmpMinInValue[currentPatch] = -3.1416;
          dmpMaxInValue[currentPatch] = 3.1416;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 21; // sets Identifier to Yaw 2 
          isDmp[currentPatch] = true;
          dmpMinInValue[currentPatch] = -3.1416;
          dmpMaxInValue[currentPatch] = 3.1416;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 122 && p.x < 238) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 17; // sets Identifier to Pitch 1 
          isDmp[currentPatch] = true;
          dmpRangeLearn();
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 20; // sets Identifier to Pitch 2 
          isDmp[currentPatch] = true;
          dmpRangeLearn();
          break;
        } 
      }
      else if (p.x > 243 && p.x < 359) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 16; // sets Identifier to Roll 1
          isDmp[currentPatch] = true;
          dmpRangeLearn();
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 19; // sets Identifier to Roll 2
          isDmp[currentPatch] = true;
          dmpRangeLearn();
          break;
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                               VOID dmpRangeLearn =
void dmpRangeLearn(){
  boolean inSetupMode = true; 
  float calibrationValue;
  float bottomValue;
  float newBottomValue;
  float topValue;
  float newTopValue;
  int16_t leftBar = 0;
  int16_t rightBar = 0;
  
  if (sensorTypeIdentifier[currentPatch] == 16 || sensorTypeIdentifier[currentPatch] == 19) { // if roll axis
    bottomValue = 1.3;
    newBottomValue = 1.3;
    topValue = -1.3;
    newTopValue = -1.3;
  }
  else if (sensorTypeIdentifier[currentPatch] == 17 || sensorTypeIdentifier[currentPatch] == 20) { // if pitch axis
    bottomValue = 1.3;
    newBottomValue = 1.3;
    topValue = -1.3;
    newTopValue = -1.3;
  }
  
  tft.fillScreen(BLACK);
  
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Range Setup");
  
  // build bar frames
  tft.drawRect(4, 56, 27, 204, currentColor);
  //tft.setCursor(8, 72);
  //tft.print(sensorTypes[0]);
  tft.drawRect(61, 56, 27, 204, currentColor);
  tft.setTextColor(currentColor);
  tft.setTextSize(2);
  //tft.setCursor(8, 72);
  //tft.print(sensorTypes[0]);
  
  // build buttons
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.fillRect(123, 56, 115, 100, currentColor);
  tft.setCursor(127, 72);
  tft.print("SET MIN");
  tft.fillRect(123, 160, 115, 100, currentColor);
  tft.setCursor(127, 176);
  tft.print("SET MAX");
  
  // draw help section
  tft.drawRect(242, 56, 234, 204, currentColor);
  tft.setTextColor(currentColor);
  tft.setTextSize(2);
  tft.setCursor(246, 60);
  tft.print("Left bar: raw");
  tft.setCursor(246, 78);
  tft.print("Right bar: scaled");
  tft.setCursor(246, 96);
  tft.print("");
  tft.setCursor(246, 114);
  tft.print("For custom range");
  tft.setCursor(246, 132);
  tft.print("move sensor to the");
  tft.setCursor(246, 150);
  tft.print("desired zero position");
  tft.setCursor(246, 168);
  tft.print("and press SET MIN.");
  tft.setCursor(246, 186);
  tft.print("Then move it to the");
  tft.setCursor(246, 204);
  tft.print("desired 100% position");
  tft.setCursor(246, 222);
  tft.print("and press SET MAX.");
  tft.setCursor(246, 240);
  tft.print("Repeat if you like.");


  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 234, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(8, 291);
  tft.print("SET FULL RANGE");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(341, 291);
  tft.print("SAVE & EXIT");

  // LOOP
  while(inSetupMode == true){
    if (!dmpReady) {
      digitalWrite(errorLED, HIGH);
      return; // if programming failed, don't try to do anything
    }
    mpuInterrupt = false; // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount(); // get current FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow 
      mpu.resetFIFO(); // reset so we can continue cleanly
    } 
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
    // update left bar
    if (sensorTypeIdentifier[currentPatch] == 16) {
      calibrationValue = ypr[2];
    }
    else if (sensorTypeIdentifier[currentPatch] == 17) {
      calibrationValue = ypr[1];
    }
  
    // UPDATE LEFT BARGRAPH
    tft.drawFastHLine(5, 258 - leftBar, 25, BLACK);
    leftBar = mapfloat(calibrationValue, topValue, bottomValue, 0, 201);
    leftBar = constrain(leftBar, 0, 201);
    tft.drawFastHLine(5, 258 - leftBar, 25, WHITE);
    
    // UPDATE RIGHT BARGRAPH
    tft.drawFastHLine(62, 258 - rightBar, 25, BLACK);
    rightBar = mapfloat(calibrationValue, newTopValue, newBottomValue, 0, 201);
    rightBar = constrain(rightBar, 0, 201);
    tft.drawFastHLine(62, 258 - rightBar, 25, WHITE);
    
    // TOUCHSCREEN 
    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
    if (touched != lastTouchState) {
      lastDebounceTime = millis();
    } 
 
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (touched != touchState) {
        touchState = touched;
        if (touchState == true) {
          p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
          p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
          swap(p.x, p.y);
          p.y = (320 - p.y);
      
          if (p.x > 122 && p.x < 238) {
            if (p.y > 55 &&  p.y < 160) { 
              newTopValue = calibrationValue; // SET MIN
            } 
            else if (p.y > 159 && p.y < 264) {
              newBottomValue = calibrationValue; // SET MAX 
            } 
          }
          if (p.y > 263 && p.y < 320) {
            if (p.x > 3 &&  p.x < 241) { 
              // do something if SET FULL RANGE is hit
              inSetupMode = false;
              dmpMinInValue[currentPatch] = bottomValue;
              dmpMaxInValue[currentPatch] = topValue;
              patchSetup(currentPatch);
            } 
            else if (p.x > 240 && p.x < 476) {
              // do something if SAVE & EXIT is hit
              dmpMinInValue[currentPatch] = newBottomValue;  
              dmpMaxInValue[currentPatch] = newTopValue; 
              patchSetup(currentPatch);
              switch (liveMode) {
                    case 1:
                      inSetupMode = false;
                      liveModeA();
                      break;
                    case 2:
                      inSetupMode = false;
                      liveModeB();
                      break;
                    case 3:
                      inSetupMode = false;
                      liveModeC();
                      break;
              }
            } 
          }
        }
      }
    }
    lastTouchState = touched;
    if (p.z < MINPRESSURE) { touched = false; }
  }
}


//====================================================================================
//-                                                          VOID selectAccInputPage =
void selectAccInputPage(){
  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(50, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Select Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Input");
  tft.fillRect(4, 56, 115, 100, currentColor);
  
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print(sensorTypes[22]); // Acc X 1
  tft.fillRect(123, 56, 115, 100, currentColor);
  tft.setCursor(127, 72);
  tft.print(sensorTypes[23]); // Acc Y 1
  tft.fillRect(4, 160, 115, 100, currentColor);
  tft.setCursor(8, 176);
  tft.print(sensorTypes[25]); // Acc X 2
  tft.fillRect(123, 160, 115, 100, currentColor);
  tft.setCursor(127, 176);
  tft.print(sensorTypes[26]); // Acc Y 2

  tft.fillRect(242, 56, 115, 100, currentColor);
  tft.setCursor(246, 72);
  tft.print(sensorTypes[24]); // Acc Z 1
  tft.fillRect(361, 56, 115, 100, currentColor);

  tft.fillRect(242, 160, 115, 100, currentColor);
  tft.setCursor(246, 176);
  tft.print(sensorTypes[27]); // Acc Z 2
  tft.fillRect(361, 160, 115, 100, currentColor);

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  //TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 119) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 22; // sets Identifier to Acc X 1
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 25; // sets Identifier to Acc X 2 
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 122 && p.x < 238) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 23; // sets Identifier to Acc Y 1 
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 26; // sets Identifier to Acc Y 2 
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 243 && p.x < 359) {
        if (p.y > 55 &&  p.y < 160) { 
          sensorTypeIdentifier[currentPatch] = 24; // sets Identifier to Acc Z 1
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 264) {
          sensorTypeIdentifier[currentPatch] = 27; // sets Identifier to Acc Z 2
          isDmp[currentPatch] = true;
          patchSetup(currentPatch);
          break;
        } 
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                            VOID selectOutputPage =
void selectOutput(){

  boolean inSetupMode = true;

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.fillRect(4, 4, 234, 48, GRAY);
  tft.fillRect(212, 52, 26, 208, GRAY);
  tft.drawFastHLine(4, 4, 234, currentColor);
  tft.drawFastHLine(4, 5, 234, currentColor);
  tft.drawFastVLine(238, 4, 256, currentColor);
  tft.drawFastVLine(237, 4, 256, currentColor);
  tft.setCursor(8, 16); 
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("MIDI Out 1");
  tft.fillRect(242, 4, 234, 48, GRAY);
  tft.fillRect(450, 52, 26, 208, GRAY);
  tft.drawFastHLine(242, 4, 234, currentColor);
  tft.drawFastHLine(242, 5, 234, currentColor);
  tft.drawFastVLine(476, 4, 256, currentColor);
  tft.drawFastVLine(475, 4, 256, currentColor);
  tft.setCursor(246, 16); 
  tft.print("MIDI Out 2");

  tft.fillRect(4, 56, 48, 48, LTGRAY);
  tft.setTextSize(3);
  tft.setCursor(11, 68);
  tft.print("01");
  tft.fillRect(56, 56, 48, 48, LTGRAY);
  tft.setCursor(63, 68);
  tft.print("02");
  tft.fillRect(108, 56, 48, 48, LTGRAY);
  tft.setCursor(115, 68);
  tft.print("03");
  tft.fillRect(160, 56, 48, 48, LTGRAY);
  tft.setCursor(167, 68);
  tft.print("04");
  tft.fillRect(4, 108, 48, 48, LTGRAY);
  tft.setCursor(11, 120);
  tft.print("05");
  tft.fillRect(56, 108, 48, 48, LTGRAY);
  tft.setCursor(63, 120);
  tft.print("06");
  tft.fillRect(108, 108, 48, 48, LTGRAY);
  tft.setCursor(115, 120);
  tft.print("07");
  tft.fillRect(160, 108, 48, 48, LTGRAY);
  tft.setCursor(167, 120);
  tft.print("08");
  tft.fillRect(4, 160, 48, 48, LTGRAY);
  tft.setCursor(11, 172);
  tft.print("09");
  tft.fillRect(56, 160, 48, 48, LTGRAY);
  tft.setCursor(63, 172);
  tft.print("10");
  tft.fillRect(108, 160, 48, 48, LTGRAY);
  tft.setCursor(115, 172);
  tft.print("11");
  tft.fillRect(160, 160, 48, 48, LTGRAY);
  tft.setCursor(167, 172);
  tft.print("12");
  tft.fillRect(4, 212, 48, 48, LTGRAY);
  tft.setCursor(11, 224);
  tft.print("13");
  tft.fillRect(56, 212, 48, 48, LTGRAY);
  tft.setCursor(63, 224);
  tft.print("14");
  tft.fillRect(108, 212, 48, 48, LTGRAY);
  tft.setCursor(115, 224);
  tft.print("15");
  tft.fillRect(160, 212, 48, 48, LTGRAY);
  tft.setCursor(167, 224);
  tft.print("16");

  tft.fillRect(242, 56, 48, 48, LTGRAY);
  tft.setTextSize(3);
  tft.setCursor(249, 68);
  tft.print("01");
  tft.fillRect(294, 56, 48, 48, LTGRAY);
  tft.setCursor(301, 68);
  tft.print("02");
  tft.fillRect(346, 56, 48, 48, LTGRAY);
  tft.setCursor(353, 68);
  tft.print("03");
  tft.fillRect(398, 56, 48, 48, LTGRAY);
  tft.setCursor(405, 68);
  tft.print("04");
  tft.fillRect(242, 108, 48, 48, LTGRAY);
  tft.setCursor(249, 120);
  tft.print("05");
  tft.fillRect(294, 108, 48, 48, LTGRAY);
  tft.setCursor(301, 120);
  tft.print("06");
  tft.fillRect(346, 108, 48, 48, LTGRAY);
  tft.setCursor(353, 120);
  tft.print("07");
  tft.fillRect(398, 108, 48, 48, LTGRAY);
  tft.setCursor(405, 120);
  tft.print("08");
  tft.fillRect(242, 160, 48, 48, LTGRAY);
  tft.setCursor(249, 172);
  tft.print("09");
  tft.fillRect(294, 160, 48, 48, LTGRAY);
  tft.setCursor(301, 172);
  tft.print("10");
  tft.fillRect(346, 160, 48, 48, LTGRAY);
  tft.setCursor(353, 172);
  tft.print("11");
  tft.fillRect(398, 160, 48, 48, LTGRAY);
  tft.setCursor(405, 172);
  tft.print("12");
  tft.fillRect(242, 212, 48, 48, LTGRAY);
  tft.setCursor(249, 224);
  tft.print("13");
  tft.fillRect(294, 212, 48, 48, LTGRAY);
  tft.setCursor(301, 224);
  tft.print("14");
  tft.fillRect(346, 212, 48, 48, LTGRAY);
  tft.setCursor(353, 224);
  tft.print("15");
  tft.fillRect(398, 212, 48, 48, LTGRAY);
  tft.setCursor(405, 224);
  tft.print("16");

  // Bottom Menu Buttons
  tft.fillRect(6, 266, 111, 44, LTGRAY);
  tft.drawRect(4, 264, 115, 48, currentColor);
  tft.drawRect(5, 265, 113, 46, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(25, 282);
  tft.print("CV OUT");
  tft.setTextColor(LTGRAY);
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 2 && p.x < 53) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 1;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 5;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 9;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 13;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 54 && p.x < 105) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 2;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 6;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 10;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 14;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 106 && p.x < 157) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 3;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 7;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 11;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 15;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 158 && p.x < 209) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 4;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 8;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 12;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = true;
          isMidiOut2[currentPatch] = false;
          midiChan[currentPatch] = 16;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }

      else if (p.x > 242 && p.x < 293) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 1;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 5;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 9;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 13;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 294 && p.x < 345) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 2;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 6;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 10;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 14;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 346 && p.x < 397) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 3;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 7;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 11;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 15;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      else if (p.x > 398 && p.x < 449) {
        if (p.y > 55 &&  p.y < 108) { 
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 4;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 8;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 12;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          isMidiOut1[currentPatch] = false;
          isMidiOut2[currentPatch] = true;
          midiChan[currentPatch] = 16;
          if(commandIdentifier[currentPatch] >= 6) commandIdentifier[currentPatch] = 1;
          patchSetup(currentPatch);
          break;
        }
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if CV OUT button is hit
          inSetupMode = false;
          selectCvOutPage();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//                                                              VOID selectCvOutPage =
void selectCvOutPage(){

  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(52, 16); // text for button 1
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Output");
  tft.fillRect(4, 56, 234, 48, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print("CV0");
  tft.fillRect(4, 108, 234, 48, currentColor);
  tft.setCursor(8, 124);
  tft.print("CV1");
  tft.fillRect(4, 160, 234, 48, currentColor);
  tft.setCursor(8, 176);
  tft.print("CV2");
  tft.fillRect(4, 212, 234, 48, currentColor);
  tft.setCursor(8, 228);
  tft.print("CV3");
  tft.fillRect(242, 56, 234, 48, currentColor);
  tft.setCursor(246, 72);
  tft.print("CV4");
  tft.fillRect(242, 108, 234, 48, currentColor);
  tft.setCursor(246, 124);
  tft.print("CV5");
  tft.fillRect(242, 160, 234, 48, currentColor);
  tft.setCursor(246, 176);
  tft.print("CV6");
  tft.fillRect(242, 212, 234, 48, currentColor);
  tft.setCursor(246, 228);
  tft.print("CV7");

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
        
      if (p.x > 3 && p.x < 238) {
        if (p.y > 55 && p.y < 108) {
          commandIdentifier[currentPatch] = 6; //sets command identifier for current patch to CV Out 0
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          commandIdentifier[currentPatch] = 7; //sets command identifier for current patch to CV Out 1
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          commandIdentifier[currentPatch] = 8; //sets command identifier for current patch to CV Out 2
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          commandIdentifier[currentPatch] = 9; //sets command identifier for current patch to CV Out 3
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
      } 
      else if (p.x > 243 && p.x < 447) {
        if (p.y > 55 &&  p.y < 108) { 
          commandIdentifier[currentPatch] = 10; //sets command identifier for current patch to CV Out 4
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          commandIdentifier[currentPatch] = 11; //sets command identifier for current patch to CV Out 5
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          commandIdentifier[currentPatch] = 12; //sets command identifier for current patch to CV Out 6
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          commandIdentifier[currentPatch] = 13; //sets command identifier for current patch to CV Out 7
          maxOutValue[currentPatch] = 4095;
          maxOutLimit[currentPatch] = 4095;
          patchSetup(currentPatch);
          break;
        }
      } 
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}


//====================================================================================
//-                                                       VOID changeActivePatchPage =
void changeActivePatchPage(){

  boolean inSetupMode = true; // tells the program to remain in the while loop

  tft.fillScreen(BLACK);
  
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);  
  tft.setCursor(60, 16);
  tft.print("Select Patch to edit");

  // build buttons
  tft.fillRect(4, 56, 234, 48, COLOR1);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 62); 
  tft.print("Patch 01");
  tft.setCursor(8, 83);
  tft.print(patchNames[0]);
  tft.fillRect(242, 56, 234, 48, COLOR6);
  tft.setCursor(247, 62);
  tft.print("Patch 06");
  tft.setCursor(247, 83);
  tft.print(patchNames[5]);
  tft.fillRect(4, 108, 234, 48, COLOR2);
  tft.setCursor(8, 114);
  tft.print("Patch 02");
  tft.setCursor(8, 135);
  tft.print(patchNames[1]);
  tft.fillRect(242, 108, 234, 48, COLOR7);
  tft.setCursor(247, 114);
  tft.print("Patch 07");
  tft.setCursor(247, 135);
  tft.print(patchNames[6]);
  tft.fillRect(4, 160, 234, 48, COLOR3);
  tft.setCursor(8, 166);
  tft.print("Patch 03");
  tft.setCursor(8, 187);
  tft.print(patchNames[2]);
  tft.fillRect(242, 160, 234, 48, COLOR8);
  tft.setCursor(247, 166);
  tft.print("Patch 08");
  tft.setCursor(247, 187);
  tft.print(patchNames[7]);
  tft.fillRect(4, 212, 234, 48, COLOR4);
  tft.setCursor(8, 218);
  tft.print("Patch 04");
  tft.setCursor(8, 239);
  tft.print(patchNames[3]);
  tft.fillRect(242, 212, 234, 48, COLOR9);
  tft.setCursor(247, 218);
  tft.print("Patch 09");
  tft.setCursor(247, 239);
  tft.print(patchNames[8]);
  tft.fillRect(4, 264, 234, 48, COLOR5);
  tft.setCursor(8, 270);
  tft.print("Patch 05");
  tft.setCursor(8, 291);
  tft.print(patchNames[4]);
  tft.fillRect(242, 264, 234, 48, COLOR0);
  tft.setCursor(247, 270);
  tft.print("Patch 10");
  tft.setCursor(247, 291);
  tft.print(patchNames[9]);

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      if (p.x > 3 && p.x < 240) {
        if (p.y > 55 &&  p.y < 108) { 
          currentPatch = 0;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          currentPatch = 1;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          currentPatch = 2;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          currentPatch = 3;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 263 && p.y < 316) {
          currentPatch = 4;
          patchSetup(currentPatch);
          break;
        } 
      }
      else if (p.x > 239 && p.x < 480) {
        if (p.y > 55 &&  p.y < 108) { 
          currentPatch = 5;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          currentPatch = 6;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          currentPatch = 7;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          currentPatch = 8;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 263 && p.y < 316) {
          currentPatch = 9;
          patchSetup(currentPatch);
          break;
        } 
      }
    }
  }
}

//====================================================================================
//                                                            VOID selectCommandPage =
void selectCommandPage(){

  boolean inSetupMode = true; 

  tft.fillScreen(BLACK);

  // build buttons and their writing
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(52, 16); // text for button 1
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Command Type");
  tft.fillRect(4, 56, 234, 48, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 72);
  tft.print("CC");
  tft.fillRect(4, 108, 234, 48, currentColor);
  tft.setCursor(8, 124);
  tft.print("NRPN 14bit");
  tft.fillRect(4, 160, 234, 48, currentColor);
  tft.setCursor(8, 176);
  tft.print("NRPN 7bit");
  tft.fillRect(4, 212, 234, 48, currentColor);
  tft.setCursor(8, 228);
  tft.print("RPN 14bit");
  tft.fillRect(242, 56, 234, 48, currentColor);
  tft.setCursor(246, 72);
  tft.print("CC 14bit");

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
        
      if (p.x > 3 && p.x < 238) {
        if (p.y > 55 && p.y < 108) {
          commandIdentifier[currentPatch] = 1; //sets command identifier for current patch to CC
          maxOutValue[currentPatch] = 127;
          maxOutLimit[currentPatch] = 127;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 107 && p.y < 160) {
          commandIdentifier[currentPatch] = 2; //sets command identifier for current patch to NRPN
          maxOutValue[currentPatch] = 16383;
          maxOutLimit[currentPatch] = 16383;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 159 && p.y < 212) {
          commandIdentifier[currentPatch] = 4; //sets command identifier for current patch to NRPN 7bit
          maxOutValue[currentPatch] = 127;
          maxOutLimit[currentPatch] = 127;
          patchSetup(currentPatch);
          break;
        } 
        else if (p.y > 211 && p.y < 264) {
          commandIdentifier[currentPatch] = 3; //sets command identifier for current patch to RPN
          maxOutValue[currentPatch] = 16383;
          maxOutLimit[currentPatch] = 16383;
          patchSetup(currentPatch);
          break;
        } 
      } 
      else if (p.x > 243 && p.x < 447) {
        if (p.y > 55 &&  p.y < 108) { 
          commandIdentifier[currentPatch] = 5; //sets command identifier for current patch to CC 14bit
          maxOutValue[currentPatch] = 16383;
          maxOutLimit[currentPatch] = 16383;
          patchSetup(currentPatch);
          break;
        } 
      } 
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                          VOID setOutputRangePage =
void setOutputRangePage() {
  boolean inSetupMode = true; // tells the program to remain in the while loop

  tft.fillScreen(BLACK);

  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(55, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.print(" Output Range");

  tft.drawRect(4, 56, 234, 100, currentColor);
  tft.fillRect(110, 56, 128, 22, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(114, 60);
  tft.print("Min. Value");
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 100);
  tft.print(minOutValue[currentPatch]);

  tft.drawRect(4, 160, 234, 100, currentColor);
  tft.fillRect(110, 160, 128, 22, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(114, 164);
  tft.print("Max. Value");
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 204);
  tft.print(maxOutValue[currentPatch]);

  tft.drawRect(242, 56, 234, 204, currentColor);
  tft.setTextColor(currentColor);
  tft.setTextSize(2);
  tft.setCursor(246, 60);
  tft.print("<<<<<<<<<<<<<<<<<<<");
  tft.setCursor(246, 78);
  tft.print("Set output value");
  tft.setCursor(246, 96);
  tft.print("range for Patch");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  tft.setCursor(246, 114);
  tft.print("here.");

  tft.setCursor(246, 150);
  tft.print("If you want the");
  tft.setCursor(246, 168);
  tft.print("highest possible");
  tft.setCursor(246, 186);
  tft.print("value sent by this");
  tft.setCursor(246, 204);
  tft.print("patch to be 96, enter");
  tft.setCursor(246, 222);
  tft.print("96 at 'Max. Value'.");
  tft.setCursor(246, 240);
  tft.print("<<<<<<<<<<<<<<<<<<<");

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("CANCEL");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 238) {
        if (p.y > 55 &&  p.y < 159) { 
          currentValueIdentifier = 1; // tells the keypad which value shall be edited
          keyPadEntry();
          break;
        } 
        else if (p.y > 158 && p.y < 262) {
          currentValueIdentifier = 2;
          keyPadEntry();
          break;
        }
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if CANCEL PROGRAM is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                               VOID setInputRange =
void setInputRange() {
  boolean inSetupMode = true; 
  tft.fillScreen(BLACK);

  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(68, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);
  tft.print("Patch ");
  tft.print(currentPatch);
  tft.print(" Input Range");

  tft.drawRect(4, 56, 234, 100, currentColor);
  tft.fillRect(110, 56, 128, 22, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(114, 60);
  tft.print("Min. Value");
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 100);
  tft.print(minInValue[currentPatch]);

  tft.drawRect(4, 160, 234, 100, currentColor);
  tft.fillRect(110, 160, 128, 22, currentColor);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(114, 164);
  tft.print("Max. Value");
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 204);
  tft.print(maxInValue[currentPatch]);

  tft.drawRect(242, 56, 234, 204, currentColor);
  tft.setTextColor(currentColor);
  tft.setTextSize(2);
  tft.setCursor(246, 60);
  tft.print("<<<<<<<<<<<<<<<<<<<");
  tft.setCursor(246, 78);
  tft.print("Please enter values");
  tft.setCursor(246, 96);
  tft.print("between 0 and 1023.");
  
  tft.setCursor(246, 132);
  tft.print("Make shure not to.");
  tft.setCursor(246, 150);
  tft.print("enter two identical");
  tft.setCursor(246, 168);
  tft.print("values.");
  tft.setCursor(246, 186);
  tft.print("For Input Range");
  tft.setCursor(246, 204);
  tft.print("Min. Value may");
  tft.setCursor(246, 222);
  tft.print("exceed Max. Value.");
  tft.setCursor(246, 240);
  tft.print("<<<<<<<<<<<<<<<<<<<");

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 282);
  tft.print("BACK");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(386, 270);
  tft.print("EXIT TO");
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");

  // TOUCH SCREEN LOOP
  while(inSetupMode == true){

    TSPoint p = ts.getPoint();
  
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);

    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
      swap(p.x, p.y);
      p.y = (320 - p.y);
      
      if (p.x > 3 && p.x < 238) {
        if (p.y > 55 &&  p.y < 159) { 
          currentValueIdentifier = 3; // tells the keypad which value shall be edited
          keyPadEntry();
          break;
        } 
        else if (p.y > 158 && p.y < 262) {
          currentValueIdentifier = 4;
          keyPadEntry();
          break;
        }
      }
      if (p.y > 263 && p.y < 320) {
        if (p.x > 3 &&  p.x < 119) { 
          // do something if SYSTEM SETUP button is hit
          inSetupMode = false;
          systemSetup();
        } 
        else if (p.x > 122 && p.x < 238) {
          // do something if BACK is hit
          inSetupMode = false;
          patchSetup(currentPatch);
        } 
        else if (p.x > 241 && p.x < 476) {
          // do something if LIVE MODE Button is hit
          switch (liveMode) {
                case 1:
                  inSetupMode = false;
                  liveModeA();
                  break;
                case 2:
                  inSetupMode = false;
                  liveModeB();
                  break;
                case 3:
                  inSetupMode = false;
                  liveModeC();
                  break;
          }
        } 
      }
    }
  }
}

//====================================================================================
//-                                                         VOID touchScreenLiveMode =
void touchscreenLiveMode(){

  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
  if (touched != lastTouchState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (touched != touchState) {
      touchState = touched;
      if (touchState == true) {
     
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
        swap(p.x, p.y);
        p.y = (320 - p.y);
        
        if (p.y < 52 && p.y > 3) {
          if (p.x < 48) { 
            currentPatch = 0;
            patchSetup(currentPatch);
          } 
          else if (p.x < 96) {
            currentPatch = 1;
            patchSetup(currentPatch);
          } 
          else if (p.x < 144) {
            currentPatch = 2;
            patchSetup(currentPatch);
          } 
          else if (p.x < 192) {
            currentPatch = 3;
            patchSetup(currentPatch);
          } 
          else if (p.x < 240) {
            currentPatch = 4;
            patchSetup(currentPatch);
          } 
          else if (p.x < 288) {
            currentPatch = 5;
            patchSetup(currentPatch);
          } 
          else if (p.x < 336) {
            currentPatch = 6;
            patchSetup(currentPatch);
          } 
          else if (p.x < 384) {
            currentPatch = 7;
            patchSetup(currentPatch);
          } 
          else if (p.x < 432) {
            currentPatch = 8;
            patchSetup(currentPatch);
          } 
          else if (p.x < 480) {
            currentPatch = 9;
            patchSetup(currentPatch);
          } 
        }
        else if (p.y < 183 && p.y > 55) {
          if (p.x < 48) { 
            if(mute[0] == false) {
              tft.fillRect(0, 40, 48, 12, WHITE);
              tft.setCursor(4, 41);
              tft.print("MUTED");
              mute[0] = true;
            } 
            else if(mute[0] == true) {
              tft.fillRect(0, 40, 48, 12, COLOR1);
              mute[0] = false;
            }
          } 
          else if (p.x < 96) {
            if(mute[1] == false) {
              tft.fillRect(48, 40, 48, 12, WHITE);
              tft.setCursor(52, 41);
              tft.print("MUTED");
              mute[1] = true;
            } 
            else if(mute[1] == true) {
              tft.fillRect(48, 40, 48, 12, COLOR2);
              mute[1] = false;
            }
          } 
          else if (p.x < 144) {
            if(mute[2] == false) {
              tft.fillRect(96, 40, 48, 12, WHITE);
              tft.setCursor(100, 41);
              tft.print("MUTED");
              mute[2] = true;
            } 
            else if(mute[2] == true) {
              tft.fillRect(96, 40, 48, 12, COLOR3);
              mute[2] = false;
            }
          } 
          else if (p.x < 192) {
            if(mute[3] == false) {
              tft.fillRect(144, 40, 48, 12, WHITE);
              tft.setCursor(148, 41);
              tft.print("MUTED");
              mute[3] = true;
            } 
            else if(mute[3] == true) {
              tft.fillRect(144, 40, 48, 12, COLOR4);
              mute[3] = false;
            }
          } 
          else if (p.x < 240) {
            if(mute[4] == false) {
              tft.fillRect(192, 40, 48, 12, WHITE);
              tft.setCursor(196, 41);
              tft.print("MUTED");
              mute[4] = true;
            } 
            else if(mute[4] == true) {
              tft.fillRect(192, 40, 48, 12, COLOR5);
              mute[4] = false;
            }
          } 
          else if (p.x < 288) {
            if(mute[5] == false) {
              tft.fillRect(240, 40, 48, 12, WHITE);
              tft.setCursor(244, 41);
              tft.print("MUTED");
              mute[5] = true;
            } 
            else if(mute[5] == true) {
              tft.fillRect(240, 40, 48, 12, COLOR6);
              mute[5] = false;
            }
          } 
          else if (p.x < 336) {
            if(mute[6] == false) {
              tft.fillRect(288, 40, 48, 12, WHITE);
              tft.setCursor(292, 41);
              tft.print("MUTED");
              mute[6] = true;
            } 
            else if(mute[6] == true) {
              tft.fillRect(288, 40, 48, 12, COLOR7);
              mute[6] = false;
            }
          } 
          else if (p.x < 384) {
            if(mute[7] == false) {
              tft.fillRect(336, 40, 48, 12, WHITE);
              tft.setCursor(340, 41);
              tft.print("MUTED");
              mute[7] = true;
            }
            else if(mute[7] == true) {
              tft.fillRect(336, 40, 48, 12, COLOR8);
              mute[7] = false;
            }
          } 
          else if (p.x < 432) {
            if(mute[8] == false) {
              tft.fillRect(384, 40, 48, 12, WHITE);
              tft.setCursor(388, 41);
              tft.print("MUTED");
              mute[8] = true;
            } 
            else if(mute[8] == true) {
              tft.fillRect(384, 40, 48, 12, COLOR9);
              mute[8] = false;
            }
          } 
          else if (p.x < 480) {
            if(mute[9] == false) {
              tft.fillRect(432, 40, 48, 12, WHITE);
              tft.setCursor(436, 41);
              tft.print("MUTED");
              mute[9] = true;
            } 
            else if(mute[9] == true) {
              tft.fillRect(432, 40, 48, 12, COLOR0);
              mute[9] = false;
            }
          } 
        }
        else if (p.y < 316 && p.y > 187) {
          if (p.x < 48) { 
            if(sensorTypeIdentifier[0] <= 15) {
              currentPatch = 0;
              invertInt();
            } 
            else if(sensorTypeIdentifier[0] >= 16 && sensorTypeIdentifier[0] <= 21) {
              currentPatch = 0;
              invertFloat();
            }
          } 
          else if (p.x < 96) {
            if(sensorTypeIdentifier[1] <= 15) {
              currentPatch = 1;
              invertInt();
            } 
            else if(sensorTypeIdentifier[1] >= 16 && sensorTypeIdentifier[1] <= 21) {
              currentPatch = 1;
              invertFloat();
            }
          } 
          else if (p.x < 144) {
            if(sensorTypeIdentifier[2] <= 15) {
              currentPatch = 2;
              invertInt();
            } 
            else if(sensorTypeIdentifier[2] >= 16 && sensorTypeIdentifier[2] <= 21) {
              currentPatch = 2;
              invertFloat();
            }
          } 
          else if (p.x < 192) {
            if(sensorTypeIdentifier[3] <= 15) {
              currentPatch = 3;
              invertInt();
            } 
            else if(sensorTypeIdentifier[3] >= 16 && sensorTypeIdentifier[3] <= 21) {
              currentPatch = 3;
              invertFloat();
            }
          } 
          else if (p.x < 240) {
            if(sensorTypeIdentifier[4] <= 15) {
              currentPatch = 4;
              invertInt();
            } 
            else if(sensorTypeIdentifier[4] >= 16 && sensorTypeIdentifier[4] <= 21) {
              currentPatch = 5;
              invertFloat();
            }
          } 
          else if (p.x < 288) {
            if(sensorTypeIdentifier[5] <= 15) {
              currentPatch = 5;
              invertInt();
            } 
            else if(sensorTypeIdentifier[5] >= 16 && sensorTypeIdentifier[5] <= 21) {
              currentPatch = 6;
              invertFloat();
            }
          } 
          else if (p.x < 336) {
            if(sensorTypeIdentifier[6] <= 15) {
              currentPatch = 6;
              invertInt();
            } 
            else if(sensorTypeIdentifier[6] >= 16 && sensorTypeIdentifier[6] <= 21) {
              currentPatch = 6;
              invertFloat();
            }
          } 
          else if (p.x < 384) {
            if(sensorTypeIdentifier[7] <= 15) {
              currentPatch = 7;
              invertInt();
            }
            else if(sensorTypeIdentifier[7] >= 16 && sensorTypeIdentifier[7] <= 21) {
              currentPatch = 7;
              invertFloat();
            }
          } 
          else if (p.x < 432) {
            if(sensorTypeIdentifier[8] <= 15) {
              currentPatch = 8;
              invertInt();
            } 
            else if(sensorTypeIdentifier[8] >= 16 && sensorTypeIdentifier[8] <= 21) {
              currentPatch = 8;
              invertFloat();
            }
          } 
          else if (p.x < 480) {
            if(sensorTypeIdentifier[9] <= 15) {
              currentPatch = 9;
              invertInt();
            } 
            else if(sensorTypeIdentifier[9] >= 16 && sensorTypeIdentifier[9] <= 21) {
              currentPatch = 9;
              invertFloat();
            }
          } 
        }
      }
    }
  }
  lastTouchState = touched;
  if (p.z < MINPRESSURE) { touched = false; }
}

//====================================================================================
//-                                                        VOID touchScreenLiveModeB =
void touchscreenLiveModeB(){

  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
  
  if (touched != lastTouchState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (touched != touchState) {
      touchState = touched;
      if (touchState == true) {
     
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
        swap(p.x, p.y);
        p.y = (320 - p.y);
        
        if (p.x >= 96 && p.x <= 160) {
          tft.setTextSize(3);
          if (p.y < 64) { 
            if(mute[0] == false) {
              tft.setTextColor(COLOR1);
              tft.setCursor(75, 41);
              tft.print("="); 
              mute[0] = true;
            } 
            else if(mute[0] == true) {
              tft.fillRect(75, 47, 15, 9, BLACK);
              mute[0] = false;
            }
          } 
          else if (p.y < 128) {
            if(mute[1] == false) {
              tft.setTextColor(COLOR2);
              tft.setCursor(75, 105);
              tft.print("="); 
              mute[1] = true;
            } 
            else if(mute[1] == true) {
              tft.fillRect(75, 111, 15, 9, BLACK);
              mute[1] = false;
            }
          } 
          else if (p.y < 192) {
            if(mute[2] == false) {
              tft.setTextColor(COLOR3);
              tft.setCursor(75, 169);
              tft.print("=");
              mute[2] = true;
            } 
            else if(mute[2] == true) {
              tft.fillRect(75, 175, 15, 9, BLACK);
              mute[2] = false;
            }
          } 
          else if (p.y < 256) {
            if(mute[3] == false) {
              tft.setTextColor(COLOR4);
              tft.setCursor(75, 233);
              tft.print("=");
              mute[3] = true;
            } 
            else if(mute[3] == true) {
              tft.fillRect(75, 239, 15, 9, BLACK);
              mute[3] = false;
            }
          } 
          else if (p.y < 320) {
            if(mute[4] == false) {
              tft.setTextColor(COLOR5);
              tft.setCursor(75, 297);
              tft.print("=");
              mute[4] = true;
            } 
            else if(mute[4] == true) {
              tft.fillRect(75, 303, 15, 9, BLACK);
              mute[4] = false;
            }
          } 
        }
        else if (p.x <= 382 && p.x >= 318) {
          tft.setTextSize(3);
          if (p.y < 64) { 
            if(mute[5] == false) {
              tft.setTextColor(COLOR6);
              tft.setCursor(458, 41);
              tft.print("=");
              mute[5] = true;
            } 
            else if(mute[5] == true) {
              tft.fillRect(458, 47, 15, 9, BLACK);
              mute[5] = false;
            }
          } 
          else if (p.y < 128) {
            if(mute[6] == false) {
              tft.setTextColor(COLOR7);
              tft.setCursor(458, 105);
              tft.print("=");
              mute[6] = true;
            } 
            else if(mute[6] == true) {
              tft.fillRect(458, 111, 15, 9, BLACK);
              mute[6] = false;
            }
          } 
          else if (p.y < 192) {
            if(mute[7] == false) {
              tft.setTextColor(COLOR8);
              tft.setCursor(458, 169);
              tft.print("=");
              mute[7] = true;
            } 
            else if(mute[7] == true) {
              tft.fillRect(458, 175, 15, 9, BLACK);
              mute[7] = false;
            }
          } 
          else if (p.y < 256) {
            if(mute[8] == false) {
              tft.setTextColor(COLOR9);
              tft.setCursor(458, 233);
              tft.print("=");
              mute[8] = true;
            } 
            else if(mute[8] == true) {
              tft.fillRect(458, 239, 15, 9, BLACK);
              mute[8] = false;
            }
          } 
          else if (p.y < 320) {
            if(mute[9] == false) {
              tft.setTextColor(COLOR0);
              tft.setCursor(458, 297);
              tft.print("=");
              mute[9] = true;
            } 
            else if(mute[9] == true) {
              tft.fillRect(458, 303, 15, 9, BLACK);
              mute[9] = false;
            }
          } 
        }
        else if (p.x < 96) {
          if (p.y < 64) { 
            currentPatch = 0;
            patchSetup(currentPatch);
          } 
          else if (p.y < 128) {
            currentPatch = 1;
            patchSetup(currentPatch);
          } 
          else if (p.y < 192) {
            currentPatch = 2;
            patchSetup(currentPatch);
          } 
          else if (p.y < 256) {
            currentPatch = 3;
            patchSetup(currentPatch);
          } 
          else if (p.y < 320) {
            currentPatch = 4;
            patchSetup(currentPatch);
          }
        }
        else if (p.x >382) {
          if (p.y < 64) { 
            currentPatch = 5;
            patchSetup(currentPatch);
          } 
          else if (p.y < 128) {
            currentPatch = 6;
            patchSetup(currentPatch);
          } 
          else if (p.y < 192) {
            currentPatch = 7;
            patchSetup(currentPatch);
          } 
          else if (p.y < 256) {
            currentPatch = 8;
            patchSetup(currentPatch);
          } 
          else if (p.y < 320) {
            currentPatch = 9;
            patchSetup(currentPatch);
          }
        }
      }
    }
  }
  lastTouchState = touched;
  if (p.z < MINPRESSURE) { touched = false; }
}

//====================================================================================
//-                                                        VOID touchScreenLiveModeC =
void touchscreenLiveModeC(){

  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
  if (touched != lastTouchState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (touched != touchState) {
      touchState = touched;
      if (touchState == true) {
     
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
        swap(p.x, p.y);
        p.y = (320 - p.y);
        
        if (p.y >= 214 && p.y <= 261) {
          if (p.x < 48) { 
            currentPatch = 0;
            patchSetup(currentPatch);
          } 
          else if (p.x < 96) {
            currentPatch = 1;
            patchSetup(currentPatch);
          } 
          else if (p.x < 144) {
            currentPatch = 2;
            patchSetup(currentPatch);
          } 
          else if (p.x < 192) {
            currentPatch = 3;
            patchSetup(currentPatch);
          } 
          else if (p.x < 240) {
            currentPatch = 4;
            patchSetup(currentPatch);
          } 
          else if (p.x < 288) {
            currentPatch = 5;
            patchSetup(currentPatch);
          } 
          else if (p.x < 336) {
            currentPatch = 6;
            patchSetup(currentPatch);
          } 
          else if (p.x < 384) {
            currentPatch = 7;
            patchSetup(currentPatch);
          } 
          else if (p.x < 432) {
            currentPatch = 8;
            patchSetup(currentPatch);
          } 
          else if (p.x < 480) {
            currentPatch = 9;
            patchSetup(currentPatch);
          } 
        }
        
        else if (p.y <= 213 && p.y >= 168) {
          
          tft.setTextColor(LTGRAY);
          tft.setTextSize(3); 
          
          if (p.x < 48) { 
            tft.setCursor(16, 180);
            if(mute[0] == false) {
              tft.setTextColor(COLOR1);
              tft.print("=");
              mute[0] = true;
            } 
            else if(mute[0] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[0] = false;
            }
          } 
          else if (p.x < 96) {
            tft.setCursor(64, 180);
            if(mute[1] == false) {
              tft.setTextColor(COLOR2);
              tft.print("=");
              mute[1] = true;
            } 
            else if(mute[1] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[1] = false;
            }
          } 
          else if (p.x < 144) {
            tft.setCursor(112, 180);
            if(mute[2] == false) {
              tft.setTextColor(COLOR3);
              tft.print("=");
              mute[2] = true;
            } 
            else if(mute[2] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[2] = false;
            }
          } 
          else if (p.x < 192) {
            tft.setCursor(160, 180);
            if(mute[3] == false) {
              tft.setTextColor(COLOR4);
              tft.print("=");
              mute[3] = true;
            } 
            else if(mute[3] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[3] = false;
            }
          } 
          else if (p.x < 240) {
            tft.setCursor(208, 180);
            if(mute[4] == false) {
              tft.setTextColor(COLOR5);
              tft.print("=");
              mute[4] = true;
            } 
            else if(mute[4] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[4] = false;
            }
          } 
          else if (p.x < 288) {
            tft.setCursor(256, 180);
            if(mute[5] == false) {
              tft.setTextColor(COLOR6);
              tft.print("=");
              mute[5] = true;
            } 
            else if(mute[5] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[5] = false;
            }
          } 
          else if (p.x < 336) {
            tft.setCursor(304, 180);
            if(mute[6] == false) {
              tft.setTextColor(COLOR7);
              tft.print("=");
              mute[6] = true;
            } 
            else if(mute[6] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[6] = false;
            }
          } 
          else if (p.x < 384) {
            tft.setCursor(352, 180);
            if(mute[7] == false) {
              tft.setTextColor(COLOR8);
              tft.print("=");
              mute[7] = true;
            }
            else if(mute[7] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[7] = false;
            }
          } 
          else if (p.x < 432) {
            tft.setCursor(400, 180);
            if(mute[8] == false) {
              tft.setTextColor(COLOR9);
              tft.print("=");
              mute[8] = true;
            } 
            else if(mute[8] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[8] = false;
            }
          } 
          else if (p.x < 480) {
            tft.setCursor(448, 180);
            if(mute[9] == false) {
              tft.setTextColor(COLOR0);
              tft.print("=");
              mute[9] = true;
            } 
            else if(mute[9] == true) {
              tft.setTextColor(GRAY);
              tft.print("=");
              mute[9] = false;
            }
          } 
        }
        else if (p.y <= 308 && p.y >= 262) {
          if (p.x < 48) { 
            if(sensorTypeIdentifier[0] <= 15) {
              currentPatch = 0;
              invertInt();
            } 
            else if(sensorTypeIdentifier[0] >= 16 && sensorTypeIdentifier[0] <= 21) {
              currentPatch = 0;
              invertFloat();
            }
          } 
          else if (p.x < 96) {
            if(sensorTypeIdentifier[1] <= 15) {
              currentPatch = 1;
              invertInt();
            } 
            else if(sensorTypeIdentifier[1] >= 16 && sensorTypeIdentifier[1] <= 21) {
              currentPatch = 1;
              invertFloat();
            }
          } 
          else if (p.x < 144) {
            if(sensorTypeIdentifier[2] <= 15) {
              currentPatch = 2;
              invertInt();
            } 
            else if(sensorTypeIdentifier[2] >= 16 && sensorTypeIdentifier[2] <= 21) {
              currentPatch = 2;
              invertFloat();
            }
          } 
          else if (p.x < 192) {
            if(sensorTypeIdentifier[3] <= 15) {
              currentPatch = 3;
              invertInt();
            } 
            else if(sensorTypeIdentifier[3] >= 16 && sensorTypeIdentifier[3] <= 21) {
              currentPatch = 3;
              invertFloat();
            }
          } 
          else if (p.x < 240) {
            if(sensorTypeIdentifier[4] <= 15) {
              currentPatch = 4;
              invertInt();
            } 
            else if(sensorTypeIdentifier[4] >= 16 && sensorTypeIdentifier[4] <= 21) {
              currentPatch = 5;
              invertFloat();
            }
          } 
          else if (p.x < 288) {
            if(sensorTypeIdentifier[5] <= 15) {
              currentPatch = 5;
              invertInt();
            } 
            else if(sensorTypeIdentifier[5] >= 16 && sensorTypeIdentifier[5] <= 21) {
              currentPatch = 6;
              invertFloat();
            }
          } 
          else if (p.x < 336) {
            if(sensorTypeIdentifier[6] <= 15) {
              currentPatch = 6;
              invertInt();
            } 
            else if(sensorTypeIdentifier[6] >= 16 && sensorTypeIdentifier[6] <= 21) {
              currentPatch = 6;
              invertFloat();
            }
          } 
          else if (p.x < 384) {
            if(sensorTypeIdentifier[7] <= 15) {
              currentPatch = 7;
              invertInt();
            }
            else if(sensorTypeIdentifier[7] >= 16 && sensorTypeIdentifier[7] <= 21) {
              currentPatch = 7;
              invertFloat();
            }
          } 
          else if (p.x < 432) {
            if(sensorTypeIdentifier[8] <= 15) {
              currentPatch = 8;
              invertInt();
            } 
            else if(sensorTypeIdentifier[8] >= 16 && sensorTypeIdentifier[8] <= 21) {
              currentPatch = 8;
              invertFloat();
            }
          } 
          else if (p.x < 480) {
            if(sensorTypeIdentifier[9] <= 15) {
              currentPatch = 9;
              invertInt();
            } 
            else if(sensorTypeIdentifier[9] >= 16 && sensorTypeIdentifier[9] <= 21) {
              currentPatch = 9;
              invertFloat();
            }
          } 
        }
      }
    }
  }
  lastTouchState = touched;
  if (p.z < MINPRESSURE) { touched = false; }
}

//====================================================================================
//-                                                                  VOID setMsbPage =
void setMsbPage() { // !sets the value for CC, too!

  tft.fillScreen(BLACK);

  currentValueIdentifier = 5; // tells the keypad which value shall be edited
  keyPadEntry();
}

//====================================================================================
//-                                                                  VOID setLsbPage =
void setLsbPage() {

  tft.fillScreen(BLACK);

  currentValueIdentifier = 6; // tells the keypad which value shall be edited
  keyPadEntry();
}

//====================================================================================
//-                                                                 VOID keyPadEntry =
void keyPadEntry() {
  inKeyPadMode = true;
  currentValue = 0;
  tft.drawRect(4, 4, 472, 48, LTGRAY);
  tft.setCursor(65, 16);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(3);  
  tft.print("Patch ");
  if(userFriendlyPatchNumber < 10) tft.print("0");
  tft.print(userFriendlyPatchNumber);
  if(currentValueIdentifier == 1 || currentValueIdentifier == 2) tft.print(" Output Range");
  else if(currentValueIdentifier == 5 && commandIdentifier[currentPatch] == 1) tft.print(" CC#");
  else if(currentValueIdentifier == 5 && commandIdentifier[currentPatch] == 2) tft.print(" MSB#");
  else if(currentValueIdentifier == 6) tft.print(" LSB#");
  tft.setTextColor(BLACK);
  buildValueWindow();
  buildKeyPad();

  while(inKeyPadMode == true) {
    if(currentEntry < 10) {
      currentValue = currentValue * 10;
      currentValue = currentValue + currentEntry;
      updateValueWindow();
      currentEntry = 13;      
    } 
    else if(currentEntry == 10) {
      currentValue = 0;
      updateValueWindow();
    } 
    else if(currentEntry == 11) {
      if(currentValueIdentifier == 1) {
        currentEntry = 13; // necessary for being able to use the keypad more than once
        if(currentValue > maxOutLimit[currentPatch]) currentValue = maxOutLimit[currentPatch];
        minOutValue[currentPatch] = currentValue;
        inKeyPadMode = false;
        setOutputRangePage();
      } 
      else if(currentValueIdentifier == 2) {
        currentEntry = 13;
        if(currentValue > maxOutLimit[currentPatch]) currentValue = maxOutLimit[currentPatch];
        maxOutValue[currentPatch] = currentValue;
        inKeyPadMode = false;
        setOutputRangePage();
      } 
      else if(currentValueIdentifier == 3) {
        currentEntry = 13;
        if(currentValue > limit10bit) currentValue = limit10bit;
        minInValue[currentPatch] = currentValue;
        inKeyPadMode = false;
        setInputRange();
      } 
      else if(currentValueIdentifier == 4) {
        currentEntry = 13;
        if(currentValue > limit10bit) currentValue = limit10bit;
        maxInValue[currentPatch] = currentValue;
        inKeyPadMode = false;
        setInputRange();
      } 
      else if(currentValueIdentifier == 5) {
        currentEntry = 13;
        if(currentValue > limit7bit) currentValue = limit7bit;
        firstDataByte[currentPatch] = currentValue;
        inKeyPadMode = false;
        patchSetup(currentPatch);
      } 
      else if(currentValueIdentifier == 6) {
        currentEntry = 13;
        if(currentValue > limit7bit) currentValue = limit7bit;
        firstDataByteB[currentPatch] = currentValue;
        inKeyPadMode = false;
        patchSetup(currentPatch);
      }
    }
    requestKeyPad();
  }
}

//====================================================================================
//-                                                            VOID buildValueWindow =
void buildValueWindow() {
  tft.fillRect(0, 56, 239, 204, BLACK);
  tft.drawRect(4, 56, 234, 100, currentColor);
  tft.fillRect(110, 56, 128, 22, currentColor);
  tft.setTextSize(2);
  tft.setCursor(114, 60);
  if(currentValueIdentifier == 1) {
    tft.print("Min. Value");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(minOutValue[currentPatch]);
  } 
  else if(currentValueIdentifier == 2) {
    tft.print("Max. Value");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(maxOutValue[currentPatch]);
  } 
  else if(currentValueIdentifier == 3) {
    tft.print("Min. Value");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(minInValue[currentPatch]);
  } 
  else if(currentValueIdentifier == 4) {
    tft.print("Max. Value");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(maxInValue[currentPatch]);
  } 
  else if(currentValueIdentifier == 5) {
    if(commandIdentifier[currentPatch] == 1) tft.print("CC#");
    else if(commandIdentifier[currentPatch] == 2 || commandIdentifier[currentPatch] == 3) tft.print("MSB#");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(firstDataByte[currentPatch]);
  } 
  else if(currentValueIdentifier == 6) {
    tft.print("LSB");
    tft.setTextColor(GRAY);
    tft.setTextSize(3);
    tft.setCursor(10, 100);
    tft.print(firstDataByteB[currentPatch]);
  }
}

//====================================================================================
//-                                                           VOID updateValueWindow =
void updateValueWindow() {
  tft.fillRect(10, 100, 120, 28, BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 100);
  tft.print(currentValue);
}

//====================================================================================
//-                                                                 VOID buildKeyPad =
void buildKeyPad() {
  tft.fillRect(243, 57, 232, 202, BLACK);
  tft.drawRect(242, 56, 234, 204, currentColor);
  tft.fillRect(247, 60, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(275, 75);
  tft.print("1");
  tft.fillRect(323, 60, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(351, 75);
  tft.print("2");
  tft.fillRect(399, 60, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(427, 75);
  tft.print("3");

  tft.fillRect(247, 110, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(275, 125);
  tft.print("4");
  tft.fillRect(323, 110, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(351, 125);
  tft.print("5");
  tft.fillRect(399, 110, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(427, 125);
  tft.print("6");

  tft.fillRect(247, 160, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(275, 175);
  tft.print("7");
  tft.fillRect(323, 160, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(351, 175);
  tft.print("8");
  tft.fillRect(399, 160, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(427, 175);
  tft.print("9");

  tft.fillRect(247, 210, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(258, 225);
  tft.print("DEL");
  tft.fillRect(323, 210, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(351, 225);
  tft.print("0");
  tft.fillRect(399, 210, 72, 46, currentColor);
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(418, 225);
  tft.print("OK");

  // build Bottom Menu Buttons
  tft.drawRect(4, 264, 115, 48, LTGRAY);
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(50, 282);
  tft.print("SETUP");
  tft.drawRect(123, 264, 115, 48, LTGRAY);
  tft.setCursor(127, 270);
  tft.print("LOAD/SAVE");
  tft.setCursor(127, 291);
  tft.print("PROGRAM");
  tft.drawRect(242, 264, 234, 48, LTGRAY);
  tft.setCursor(364, 291);
  tft.print("LIVE MODE");
}

//====================================================================================
//-                                                               VOID requestKeyPad =
void requestKeyPad() {

  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
  if (touched != lastTouchState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (touched != touchState) {
      touchState = touched;
      if (touchState == true) {
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
        swap(p.x, p.y);
        p.y = (320 - p.y);
        
        if (p.x < 320 && p.x > 246) {
          if (p.y < 107 && p.y > 59) { 
            currentEntry = 1;
          } 
          else if (p.y < 157 && p.y > 109) {
            currentEntry = 4;
          } 
          else if (p.y < 207 && p.y > 159) {
            currentEntry = 7;
          } 
          else if (p.y < 257 && p.y > 209) {
            currentEntry = 10;
          } 
        }
        else if (p.x < 396 && p.x > 322) {
          if (p.y < 107 && p.y > 59) { 
            currentEntry = 2;
          } 
          else if (p.y < 157 && p.y > 109) {
            currentEntry = 5;
          } 
          else if (p.y < 207 && p.y > 159) {
            currentEntry = 8;
          } 
          else if (p.y < 257 && p.y > 209) {
            currentEntry = 0;
          } 
        }
        else if (p.x < 472 && p.x > 398) {
          if (p.y < 107 && p.y > 59) { 
            currentEntry = 3;
          } 
          else if (p.y < 157 && p.y > 109) {
            currentEntry = 6;
          } 
          else if (p.y < 207 && p.y > 159) {
            currentEntry = 9;
          } 
          else if (p.y < 257 && p.y > 209) {
            currentEntry = 11;
          } 
        }
        else if (p.y > 263 && p.y < 320) {
          if (p.x > 122 && p.x < 238) {
            // do something if CANCEL Button is hit
            if(currentValueIdentifier == 1 || currentValueIdentifier == 2) {
              inKeyPadMode = false;
              setOutputRangePage();
            } 
            else if(currentValueIdentifier == 5 || currentValueIdentifier == 6) {
              inKeyPadMode = false;
              patchSetup(currentPatch);
            }
          } 
          if(p.x > 241 && p.x < 476) {
            // do something if LIVE MODE Button is hit
            inKeyPadMode = false;
            switch (liveMode) {
                  case 1:
                    liveModeA();
                    break;
                  case 2:
                    liveModeB();
                    break;
                  case 3:
                    liveModeC();
                    break;
            }
          } 
        }
      }
    }
  }
  lastTouchState = touched;
  if (p.z < MINPRESSURE) { touched = false; }
}

//====================================================================================
//-                                                           VOID keyboardEntryLoop =
void keyboardEntryLoop() {
  inKeyboardMode = true;
  
  tft.fillScreen(BLACK);
  if(currentTextIdentifier == 2) currentColor = LTGRAY;
  tft.drawRect(1, 1, 478, 30, currentColor);
  tft.setCursor(105, 5);
  tft.setTextColor(currentColor);
  tft.setTextSize(3);  
  if(currentTextIdentifier == 1) {
    tft.print("Rename Patch ");
    if(userFriendlyPatchNumber < 10) tft.print("0");
    tft.print(userFriendlyPatchNumber);
    
    tft.setTextColor(GRAY);
    tft.setCursor(10, 45);
    tft.print(patchNames[currentPatch]);
  }
  else if(currentTextIdentifier == 2) {
    tft.print("Enter Preset Name");
    
    tft.setTextColor(GRAY);
    tft.setCursor(10, 45);
    tft.print(pNameB);
  }
  
  buildKeyboard();
  printPositionMarker();
  
  while(inKeyboardMode == true) {
    
    if(currentChar >= 32 && currentChar <= 126) {
      charBuffer[cursorPosition] = currentChar;
      currentChar = 22;
      updateTextWindow();
      if(cursorPosition <=5) cursorPosition++;
      printPositionMarker();
    }
    else if (currentChar == 8) {
      if (cursorPosition >= 1 && cursorPosition <=5) {
        cursorPosition--;
        charBuffer[cursorPosition] = 32;
      }
      else if (cursorPosition == 6) {
        charBuffer[cursorPosition] = 32;
        cursorPosition--;
      }
      currentChar = 22;
      updateTextWindow();
      printPositionMarker();
    }
    else if(currentChar == 13) {
      if(currentTextIdentifier == 1) {
        patchNames[currentPatch][0] = charBuffer[0];
        patchNames[currentPatch][1] = charBuffer[1];
        patchNames[currentPatch][2] = charBuffer[2];
        patchNames[currentPatch][3] = charBuffer[3];
        patchNames[currentPatch][4] = charBuffer[4];
        patchNames[currentPatch][5] = charBuffer[5];
        patchNames[currentPatch][6] = charBuffer[6];
        charBuffer[0] = 32;
        charBuffer[1] = 32;
        charBuffer[2] = 32;
        charBuffer[3] = 32;
        charBuffer[4] = 32;
        charBuffer[5] = 32;
        charBuffer[6] = 32;
        currentChar = 22;
        cursorPosition = 0;
        uppercase = true;
        inKeyboardMode = false;
        patchSetup(currentPatch);
      }
      else if(currentTextIdentifier == 2) {
        pNameB[0] = charBuffer[0];
        pNameB[1] = charBuffer[1];
        pNameB[2] = charBuffer[2];
        pNameB[3] = charBuffer[3];
        pNameB[4] = charBuffer[4];
        pNameB[5] = charBuffer[5];
        pNameB[6] = charBuffer[6];
        charBuffer[0] = 32;
        charBuffer[1] = 32;
        charBuffer[2] = 32;
        charBuffer[3] = 32;
        charBuffer[4] = 32;
        charBuffer[5] = 32;
        charBuffer[6] = 32;
        currentChar = 22;
        cursorPosition = 0;
        uppercase = true;
        inKeyboardMode = false;
        saveA(1);
      }
    }
    else if(currentChar == 7) {
      charBuffer[0] = 32;
      charBuffer[1] = 32;
      charBuffer[2] = 32;
      charBuffer[3] = 32;
      charBuffer[4] = 32;
      charBuffer[5] = 32;
      charBuffer[6] = 32;
      currentChar = 22;
      cursorPosition = 0;
      uppercase = true;
      inKeyboardMode = false;
      patchSetup(currentPatch); // BACK
    }
    requestKeyboard();
  }
}

//====================================================================================
//-                                                               VOID buildKeyboard =
void buildKeyboard() {
  
  tft.fillRect(1, 81, 46, 46, currentColor);
  tft.fillRect(49, 81, 46, 46, currentColor);
  tft.fillRect(97, 81, 46, 46, currentColor);
  tft.fillRect(145, 81, 46, 46, currentColor);
  tft.fillRect(193, 81, 46, 46, currentColor);
  tft.fillRect(241, 81, 46, 46, currentColor);
  tft.fillRect(289, 81, 46, 46, currentColor);
  tft.fillRect(337, 81, 46, 46, currentColor);
  tft.fillRect(385, 81, 46, 46, currentColor);
  tft.fillRect(433, 81, 46, 46, currentColor);
  tft.fillRect(1, 129, 46, 46, currentColor);
  tft.fillRect(49, 129, 46, 46, currentColor);
  tft.fillRect(97, 129, 46, 46, currentColor);
  tft.fillRect(145, 129, 46, 46, currentColor);
  tft.fillRect(193, 129, 46, 46, currentColor);
  tft.fillRect(241, 129, 46, 46, currentColor);
  tft.fillRect(289, 129, 46, 46, currentColor);
  tft.fillRect(337, 129, 46, 46, currentColor);
  tft.fillRect(385, 129, 46, 46, currentColor);
  tft.fillRect(433, 129, 46, 46, currentColor);
  tft.fillRect(25, 177, 46, 46, currentColor);
  tft.fillRect(73, 177, 46, 46, currentColor);
  tft.fillRect(121, 177, 46, 46, currentColor);
  tft.fillRect(169, 177, 46, 46, currentColor);
  tft.fillRect(217, 177, 46, 46, currentColor);
  tft.fillRect(265, 177, 46, 46, currentColor);
  tft.fillRect(313, 177, 46, 46, currentColor);
  tft.fillRect(361, 177, 46, 46, currentColor);
  tft.fillRect(409, 177, 46, 46, currentColor);
  tft.drawRect(1, 225, 46, 46, currentColor);
  tft.fillRect(49, 225, 46, 46, currentColor);
  tft.fillRect(97, 225, 46, 46, currentColor);
  tft.fillRect(145, 225, 46, 46, currentColor);
  tft.fillRect(193, 225, 46, 46, currentColor);
  tft.fillRect(241, 225, 46, 46, currentColor);
  tft.fillRect(289, 225, 46, 46, currentColor);
  tft.fillRect(337, 225, 46, 46, currentColor);
  tft.drawRect(385, 225, 94, 46, currentColor);
  tft.setCursor(401, 237);
  drawBackspaceArrow();
  tft.drawRect(1, 273, 118, 46, currentColor);
  tft.setTextColor(currentColor);
  tft.setCursor(7, 285);
  tft.print("CANCEL");
  tft.fillRect(121, 273, 238, 46, currentColor);
  tft.drawRect(361, 273, 118, 46, currentColor);
  tft.setCursor(377, 285);
  tft.print("ENTER");
  
  if(uppercase == true) {
    tft.setTextColor(BLACK);
    tft.setTextSize(3);
    tft.setCursor(17, 93);
    tft.print("1");
    tft.setCursor(65, 93);
    tft.print("2");
    tft.setCursor(113, 93);
    tft.print("3");
    tft.setCursor(161, 93);
    tft.print("4");
    tft.setCursor(209, 93);
    tft.print("5");
    tft.setCursor(257, 93);
    tft.print("6");
    tft.setCursor(305, 93);
    tft.print("7");
    tft.setCursor(353, 93);
    tft.print("8");
    tft.setCursor(401, 93);
    tft.print("9");
    tft.setCursor(449, 93);
    tft.print("0");
    tft.setCursor(17, 141);
    tft.print("Q");
    tft.setCursor(65, 141);
    tft.print("W");
    tft.setCursor(113, 141);
    tft.print("E");
    tft.setCursor(161, 141);
    tft.print("R");
    tft.setCursor(209, 141);
    tft.print("T");
    tft.setCursor(257, 141);
    tft.print("Y");
    tft.setCursor(305, 141);
    tft.print("U");
    tft.setCursor(353, 141);
    tft.print("I");
    tft.setCursor(401, 141);
    tft.print("O");
    tft.setCursor(449, 141);
    tft.print("P");
    tft.setCursor(41, 189);
    tft.print("A");
    tft.setCursor(89, 189);
    tft.print("S");
    tft.setCursor(137, 189);
    tft.print("D");
    tft.setCursor(185, 189);
    tft.print("F");
    tft.setCursor(233, 189);
    tft.print("G");
    tft.setCursor(281, 189);
    tft.print("H");
    tft.setCursor(329, 189);
    tft.print("J");
    tft.setCursor(377, 189);
    tft.print("K");
    tft.setCursor(425, 189);
    tft.print("L");
    drawShiftArrow();
    tft.setCursor(65, 237);
    tft.print("Z");
    tft.setCursor(113, 237);
    tft.print("X");
    tft.setCursor(161, 237);
    tft.print("C");
    tft.setCursor(209, 237);
    tft.print("V");
    tft.setCursor(257, 237);
    tft.print("B");
    tft.setCursor(305, 237);
    tft.print("N");
    tft.setCursor(353, 237);
    tft.print("M");
  } 
  else {
     tft.setTextColor(BLACK);
    tft.setTextSize(3);
    tft.setCursor(17, 93);
    tft.print("+");
    tft.setCursor(65, 93);
    tft.print("-");
    tft.setCursor(113, 93);
    tft.print(".");
    tft.setCursor(161, 93);
    tft.print("#");
    tft.setCursor(209, 93);
    tft.print("@");
    tft.setCursor(257, 93);
    tft.print("^");
    tft.setCursor(305, 93);
    tft.print("/");
    tft.setCursor(353, 93);
    tft.print("*");
    tft.setCursor(401, 93);
    tft.print("~");
    tft.setCursor(449, 93);
    tft.print("=");
    tft.setCursor(17, 141);
    tft.print("q");
    tft.setCursor(65, 141);
    tft.print("w");
    tft.setCursor(113, 141);
    tft.print("e");
    tft.setCursor(161, 141);
    tft.print("r");
    tft.setCursor(209, 141);
    tft.print("t");
    tft.setCursor(257, 141);
    tft.print("y");
    tft.setCursor(305, 141);
    tft.print("u");
    tft.setCursor(353, 141);
    tft.print("i");
    tft.setCursor(401, 141);
    tft.print("o");
    tft.setCursor(449, 141);
    tft.print("p");
    tft.setCursor(41, 189);
    tft.print("a");
    tft.setCursor(89, 189);
    tft.print("s");
    tft.setCursor(137, 189);
    tft.print("d");
    tft.setCursor(185, 189);
    tft.print("f");
    tft.setCursor(233, 189);
    tft.print("g");
    tft.setCursor(281, 189);
    tft.print("h");
    tft.setCursor(329, 189);
    tft.print("j");
    tft.setCursor(377, 189);
    tft.print("k");
    tft.setCursor(425, 189);
    tft.print("l");
    tft.setCursor(65, 237);
    tft.print("z");
    tft.setCursor(113, 237);
    tft.print("x");
    tft.setCursor(161, 237);
    tft.print("c");
    tft.setCursor(209, 237);
    tft.print("v");
    tft.setCursor(257, 237);
    tft.print("b");
    tft.setCursor(305, 237);
    tft.print("n");
    tft.setCursor(353, 237);
    tft.print("m");
  }
}

//====================================================================================
//-                                                             VOID requestKeyboard =
void requestKeyboard() {

  TSPoint p = ts.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) { touched = true; }
  if (touched != lastTouchState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (touched != touchState) {
      touchState = touched;
      if (touchState == true) {
     
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
        swap(p.x, p.y);
        p.y = (320 - p.y);
        
        if (p.y < 128 && p.y > 79) { // the touchscreen coordinates have no disturbing gaps. proof.
          if (p.x < 48) { 
            if (uppercase == true) { currentChar = 49; } // 1
            else { currentChar = 43; } // +
          } 
          else if (p.x < 96) {
            if (uppercase == true) { currentChar = 50; } // 2 
            else { currentChar = 45; } // -
          } 
          else if (p.x < 144) {
            if (uppercase == true) { currentChar = 51; } // 3            
            else { currentChar = 46; } // .
          } 
          else if (p.x < 192) {
            if (uppercase == true) { currentChar = 52; } // 4
            else { currentChar = 35; } // #
          } 
          else if (p.x < 240) { 
            if (uppercase == true) {currentChar = 53; } // 5
            else { currentChar = 64; } // @
          } 
          else if (p.x < 288) {
            if (uppercase == true) { currentChar = 54; } // 6
            else { currentChar = 94; } // ^
          } 
          else if (p.x < 336) {
            if (uppercase == true) { currentChar = 55; } // 7
            else { currentChar = 47; } // /
          } 
          else if (p.x < 384) {
            if (uppercase == true) { currentChar = 56; } // 8
            else { currentChar = 42; } // *
          } 
          else if (p.x < 432) {
            if (uppercase == true) { currentChar = 57; } // 9
            else { currentChar = 126; } // ~
          } 
          else if (p.x < 480) {
            if (uppercase == true) { currentChar = 48; } // 0
            else { currentChar = 61; } // =
          }
        } 
        else if (p.y < 176 && p.y > 127) {
          if (p.x < 48) { 
            if (uppercase == true) { currentChar = 81; } // Q
            else { currentChar = 113; } // q
          } 
          else if (p.x < 96) {
            if (uppercase == true) { currentChar = 87; } // W
            else { currentChar = 119; } // w 
          } 
          else if (p.x < 144) {
            if (uppercase == true) { currentChar = 69; } // E
            else { currentChar = 101; } // e
          } 
          else if (p.x < 192) {
            if (uppercase == true) { currentChar = 82; } // R
            else { currentChar = 114; } // r
          } 
          else if (p.x < 240) {
            if (uppercase == true) { currentChar = 84; } // T
            else { currentChar = 116; } // t
          } 
          else if (p.x < 288) {
            if (uppercase == true) { currentChar = 89; } // Y
            else { currentChar = 121; } // y
          } 
          else if (p.x < 336) {
            if (uppercase == true) { currentChar = 85; } // U
            else { currentChar = 117; } // u
          } 
          else if (p.x < 384) {
            if (uppercase == true) { currentChar = 73; } // I
            else { currentChar = 105; } // i
          } 
          else if (p.x < 432) {
            if (uppercase == true) { currentChar = 79; } // O
            else { currentChar = 111; } // o
          } 
          else if (p.x < 480) {
            if (uppercase == true) { currentChar = 80; } // P
            else { currentChar = 112; } // p
          }
        } 
        else if (p.y < 224 && p.y > 175) {
          if (p.x < 24) {
            //EMPTY SPACE
          } 
          else if (p.x < 72) {
            if (uppercase == true) { currentChar = 65; } // A
            else { currentChar = 97; } // a
          } 
          else if (p.x < 120) {
            if (uppercase == true) { currentChar = 83; } // S
            else { currentChar = 115; } // s
          } 
          else if (p.x < 168) {
            if (uppercase == true) { currentChar = 68; } // D
            else { currentChar = 100; } // d
          } 
          else if (p.x < 216) {
            if (uppercase == true) { currentChar = 70; } // F
            else { currentChar = 102; } // f
          } 
          else if (p.x < 264) {
            if (uppercase == true) { currentChar = 71; } // G
            else { currentChar = 103; } // g
          } 
          else if (p.x < 312) {
            if (uppercase == true) { currentChar = 72; } // H
            else { currentChar = 104; } // h
          } 
          else if (p.x < 360) {
            if (uppercase == true) { currentChar = 74; } // J
            else { currentChar = 106; } // j
          } 
          else if (p.x < 408) {
            if (uppercase == true) { currentChar = 75; } // K
            else { currentChar = 107; } // k
          } 
          else if (p.x < 456) {
            if (uppercase == true) { currentChar = 76; } // L
            else { currentChar = 108; } // l
          }
        } 
        else if (p.y < 272 && p.y > 223) {
          if (p.x < 48) { 
            // SHIFT
            if (uppercase == true) {
              uppercase = false;
              buildKeyboard();
            }
            else {
              uppercase = true;
              buildKeyboard();
            }
             
          } 
          else if (p.x < 96) {
            if (uppercase == true) { currentChar = 90; } // Z
            else { currentChar = 122; } // z
          } 
          else if (p.x < 144) {
            if (uppercase == true) { currentChar = 88; } // X
            else { currentChar = 120; } // x
          } 
          else if (p.x < 192) {
            if (uppercase == true) { currentChar = 67; } // C
            else { currentChar = 99; } // c
          } 
          else if (p.x < 240) {
            if (uppercase == true) { currentChar = 86; } // V
            else { currentChar = 118; } // v
          } 
          else if (p.x < 288) {
            if (uppercase == true) { currentChar = 66; } // B
            else { currentChar = 98; } // b
          } 
          else if (p.x < 336) {
            if (uppercase == true) { currentChar = 78; } // N
            else { currentChar = 110; } // n
          } 
          else if (p.x < 384) {
            if (uppercase == true) { currentChar = 77; } // M
            else { currentChar = 109; } // m
          } 
          else if (p.x < 480) {
            currentChar = 8; // BACKSPACE
          }
        } 
        else if (p.y < 320 && p.y > 271) {
          if (p.x < 120) { 
            currentChar = 7; // CANCEL
          } 
          else if (p.x < 360) {
            //SPACE KEY
            currentChar = 32;
          } 
          else if (p.x < 480) {
            currentChar = 13; // ENTER
          } 
        }
      }
    }
  }
  lastTouchState = touched;
  if (p.z < MINPRESSURE) { touched = false; }
}

//====================================================================================
//-                                                            VOID updateTextWindow =
void updateTextWindow() {
  tft.fillRect(9, 45, 145, 30, BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 45);
  tft.print(charBuffer);
}

//====================================================================================
//-                                                         VOID printPositionMarker =
void printPositionMarker() {
  switch (cursorPosition) {
    case 0:
      tft.fillRect(10, 70, 15, 3, currentColor);
      break;
    case 1:
      tft.fillRect(28, 70, 15, 3, currentColor);
      break;
      case 2:
      tft.fillRect(46, 70, 15, 3, currentColor);
      break;
      case 3:
      tft.fillRect(64, 70, 15, 3, currentColor);
      break;
      case 4:
      tft.fillRect(82, 70, 15, 3, currentColor);
      break;
      case 5:
      tft.fillRect(100, 70, 15, 3, currentColor);
      break;
      case 6:
      tft.fillRect(118, 70, 15, 3, currentColor);
      break;
      case 7:
      tft.fillRect(136, 70, 15, 3, currentColor);
      break;
  }
}

//====================================================================================
//-                                                   VOID calculateMotionSendValues =
void calculateMotionSendValues(){

  if (!dmpReady) {
    digitalWrite(errorLED, HIGH);
    return; // if programming failed, don't try to do anything
  }
  mpuInterrupt = false; // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount(); // get current FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check for overflow 
    mpu.resetFIFO(); // reset so we can continue cleanly

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Obtain YPR angles from buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    if(mute[0] == false) {
      if(isDmp[0] == true) {
        switch (sensorTypeIdentifier[0]) {
              case 16:
                secondDataByte[0] = mapfloat(ypr[2], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
              case 17:
                secondDataByte[0] = mapfloat(ypr[1], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
              case 18: 
                secondDataByte[0] = mapfloat(ypr[0], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
              case 19:
                secondDataByte[0] = mapfloat(ypr[2], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
              case 20: 
                secondDataByte[0] = mapfloat(ypr[1], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
              case 21:
                secondDataByte[0] = mapfloat(ypr[0], dmpMaxInValue[0], dmpMinInValue[0], minOutValue[0], maxOutValue[0]);
                break;
        }
        secondDataByte[0] = constrain(secondDataByte[0], minOutValue[0], maxOutValue[0]);
        switch (commandIdentifier[0]) {
              case 1: 
                if(isMidiOut1[0] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[0], secondDataByte[0], midiChan[0]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[0], secondDataByte[0], midiChan[0]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[0] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                  midiA.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiA.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                  midiB.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiB.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[0] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[0], midiChan[0]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[0], midiChan[0]);
                  midiA.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiA.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[0], midiChan[0]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[0], midiChan[0]);
                  midiB.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiB.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[0] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                  midiA.sendControlChange(6, secondDataByte[0], midiChan[0]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                  midiB.sendControlChange(6, secondDataByte[0], midiChan[0]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[0] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[0], secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiA.sendControlChange(firstDataByteB[0], secondDataByte[0] % nrpnDivisor, midiChan[0]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[0], secondDataByte[0] / nrpnDivisor, midiChan[0]);
                  midiB.sendControlChange(firstDataByteB[0], secondDataByte[0] % nrpnDivisor, midiChan[0]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[0]);                
                break;
              case 7:
                dac.outputB(secondDataByte[0]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[0]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[0]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[0]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[0]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[0]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[0]);                
                break;
        }
      }
    }
    if(mute[1] == false) {
      if(isDmp[1] == true) {
        switch (sensorTypeIdentifier[1]) {
              case 16:
                secondDataByte[1] = mapfloat(ypr[2], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
              case 17:
                secondDataByte[1] = mapfloat(ypr[1], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
              case 18: 
                secondDataByte[1] = mapfloat(ypr[0], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
              case 19:
                secondDataByte[1] = mapfloat(ypr[2], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
              case 20: 
                secondDataByte[1] = mapfloat(ypr[1], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
              case 21:
                secondDataByte[1] = mapfloat(ypr[0], dmpMaxInValue[1], dmpMinInValue[1], minOutValue[1], maxOutValue[1]);
                break;
        }
        secondDataByte[1] = constrain(secondDataByte[1], minOutValue[1], maxOutValue[1]);
        switch (commandIdentifier[1]) {
              case 1: 
                if(isMidiOut1[1] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[1], secondDataByte[1], midiChan[1]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[1], secondDataByte[1], midiChan[1]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[1] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                  midiA.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiA.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                  midiB.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiB.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[1] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[1], midiChan[1]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[1], midiChan[1]);
                  midiA.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiA.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[1], midiChan[1]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[1], midiChan[1]);
                  midiB.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiB.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[1] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                  midiA.sendControlChange(6, secondDataByte[1], midiChan[1]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                  midiB.sendControlChange(6, secondDataByte[1], midiChan[1]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[1] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[1], secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiA.sendControlChange(firstDataByteB[1], secondDataByte[1] % nrpnDivisor, midiChan[1]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[1], secondDataByte[1] / nrpnDivisor, midiChan[1]);
                  midiB.sendControlChange(firstDataByteB[1], secondDataByte[1] % nrpnDivisor, midiChan[1]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[1]);                
                break;
              case 7:
                dac.outputB(secondDataByte[1]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[1]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[1]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[1]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[1]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[1]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[1]);                
                break;
        }
      }
    }
    if(mute[2] == false) {
      if(isDmp[2] == true) {
        switch (sensorTypeIdentifier[2]) {
              case 16:
                secondDataByte[2] = mapfloat(ypr[2], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
              case 17:
                secondDataByte[2] = mapfloat(ypr[1], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
              case 18: 
                secondDataByte[2] = mapfloat(ypr[0], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
              case 19:
                secondDataByte[2] = mapfloat(ypr[2], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
              case 20: 
                secondDataByte[2] = mapfloat(ypr[1], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
              case 21:
                secondDataByte[2] = mapfloat(ypr[0], dmpMaxInValue[2], dmpMinInValue[2], minOutValue[2], maxOutValue[2]);
                break;
        }
        secondDataByte[2] = constrain(secondDataByte[2], minOutValue[2], maxOutValue[2]);
        switch (commandIdentifier[2]) {
              case 1: 
                if(isMidiOut1[2] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[2], secondDataByte[2], midiChan[2]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[2], secondDataByte[2], midiChan[2]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[2] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                  midiA.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiA.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                  midiB.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiB.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[2] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[2], midiChan[2]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[2], midiChan[2]);
                  midiA.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiA.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[2], midiChan[2]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[2], midiChan[2]);
                  midiB.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiB.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[2] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                  midiA.sendControlChange(6, secondDataByte[2], midiChan[2]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                  midiB.sendControlChange(6, secondDataByte[2], midiChan[2]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[2] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[2], secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiA.sendControlChange(firstDataByteB[2], secondDataByte[2] % nrpnDivisor, midiChan[2]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[2], secondDataByte[2] / nrpnDivisor, midiChan[2]);
                  midiB.sendControlChange(firstDataByteB[2], secondDataByte[2] % nrpnDivisor, midiChan[2]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[2]);                
                break;
              case 7:
                dac.outputB(secondDataByte[2]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[2]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[2]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[2]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[2]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[2]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[2]);                
                break;
        }
      }
    }
    if(mute[3] == false) {
      if(isDmp[3] == true) {
        switch (sensorTypeIdentifier[3]) {
              case 16:
                secondDataByte[3] = mapfloat(ypr[2], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
              case 17:
                secondDataByte[3] = mapfloat(ypr[1], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
              case 18: 
                secondDataByte[3] = mapfloat(ypr[0], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
              case 19:
                secondDataByte[3] = mapfloat(ypr[2], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
              case 20: 
                secondDataByte[3] = mapfloat(ypr[1], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
              case 21:
                secondDataByte[3] = mapfloat(ypr[0], dmpMaxInValue[3], dmpMinInValue[3], minOutValue[3], maxOutValue[3]);
                break;
        }
        secondDataByte[3] = constrain(secondDataByte[3], minOutValue[3], maxOutValue[3]);
        switch (commandIdentifier[3]) {
              case 1: 
                if(isMidiOut1[3] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[3], secondDataByte[3], midiChan[3]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[3], secondDataByte[3], midiChan[3]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[3] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                  midiA.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiA.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                  midiB.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiB.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[3] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[3], midiChan[3]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[3], midiChan[3]);
                  midiA.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiA.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[3], midiChan[3]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[3], midiChan[3]);
                  midiB.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiB.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[3] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                  midiA.sendControlChange(6, secondDataByte[3], midiChan[3]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                  midiB.sendControlChange(6, secondDataByte[3], midiChan[3]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[3] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[3], secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiA.sendControlChange(firstDataByteB[3], secondDataByte[3] % nrpnDivisor, midiChan[3]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[3], secondDataByte[3] / nrpnDivisor, midiChan[3]);
                  midiB.sendControlChange(firstDataByteB[3], secondDataByte[3] % nrpnDivisor, midiChan[3]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[3]);                
                break;
              case 7:
                dac.outputB(secondDataByte[3]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[3]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[3]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[3]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[3]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[3]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[3]);                
                break;
        }
      }
    }
    if(mute[4] == false) {
      if(isDmp[4] == true) {
        switch (sensorTypeIdentifier[4]) {
              case 16:
                secondDataByte[4] = mapfloat(ypr[2], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
              case 17:
                secondDataByte[4] = mapfloat(ypr[1], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
              case 18: 
                secondDataByte[4] = mapfloat(ypr[0], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
              case 19:
                secondDataByte[4] = mapfloat(ypr[2], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
              case 20: 
                secondDataByte[4] = mapfloat(ypr[1], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
              case 21:
                secondDataByte[4] = mapfloat(ypr[0], dmpMaxInValue[4], dmpMinInValue[4], minOutValue[4], maxOutValue[4]);
                break;
        }
        secondDataByte[4] = constrain(secondDataByte[4], minOutValue[4], maxOutValue[4]);
        switch (commandIdentifier[4]) {
              case 1: 
                if(isMidiOut1[4] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[4], secondDataByte[4], midiChan[4]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[4], secondDataByte[4], midiChan[4]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[4] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                  midiA.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiA.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                  midiB.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiB.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[4] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[4], midiChan[4]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[4], midiChan[4]);
                  midiA.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiA.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[4], midiChan[4]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[4], midiChan[4]);
                  midiB.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiB.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[4] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                  midiA.sendControlChange(6, secondDataByte[4], midiChan[4]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                  midiB.sendControlChange(6, secondDataByte[4], midiChan[4]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[4] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[4], secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiA.sendControlChange(firstDataByteB[4], secondDataByte[4] % nrpnDivisor, midiChan[4]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[4], secondDataByte[4] / nrpnDivisor, midiChan[4]);
                  midiB.sendControlChange(firstDataByteB[4], secondDataByte[4] % nrpnDivisor, midiChan[4]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[4]);                
                break;
              case 7:
                dac.outputB(secondDataByte[4]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[4]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[4]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[4]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[4]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[4]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[4]);                
                break;
        }
      }
    }
    if(mute[5] == false) {
      if(isDmp[5] == true) {
        switch (sensorTypeIdentifier[5]) {
              case 16:
                secondDataByte[5] = mapfloat(ypr[2], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
              case 17:
                secondDataByte[5] = mapfloat(ypr[1], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
              case 18: 
                secondDataByte[5] = mapfloat(ypr[0], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
              case 19:
                secondDataByte[5] = mapfloat(ypr[2], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
              case 20: 
                secondDataByte[5] = mapfloat(ypr[1], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
              case 21:
                secondDataByte[5] = mapfloat(ypr[0], dmpMaxInValue[5], dmpMinInValue[5], minOutValue[5], maxOutValue[5]);
                break;
        }
        secondDataByte[5] = constrain(secondDataByte[5], minOutValue[5], maxOutValue[5]);
        switch (commandIdentifier[5]) {
              case 1: 
                if(isMidiOut1[5] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[5], secondDataByte[5], midiChan[5]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[5], secondDataByte[5], midiChan[5]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[5] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                  midiA.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiA.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                  midiB.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiB.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[5] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[5], midiChan[5]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[5], midiChan[5]);
                  midiA.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiA.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[5], midiChan[5]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[5], midiChan[5]);
                  midiB.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiB.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[5] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                  midiA.sendControlChange(6, secondDataByte[5], midiChan[5]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                  midiB.sendControlChange(6, secondDataByte[5], midiChan[5]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[5] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[5], secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiA.sendControlChange(firstDataByteB[5], secondDataByte[5] % nrpnDivisor, midiChan[5]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[5], secondDataByte[5] / nrpnDivisor, midiChan[5]);
                  midiB.sendControlChange(firstDataByteB[5], secondDataByte[5] % nrpnDivisor, midiChan[5]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[5]);                
                break;
              case 7:
                dac.outputB(secondDataByte[5]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[5]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[5]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[5]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[5]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[5]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[5]);                
                break;
        }
      }
    }
    if(mute[6] == false) {
      if(isDmp[6] == true) {
        switch (sensorTypeIdentifier[6]) {
              case 16:
                secondDataByte[6] = mapfloat(ypr[2], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
              case 17:
                secondDataByte[6] = mapfloat(ypr[1], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
              case 18: 
                secondDataByte[6] = mapfloat(ypr[0], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
              case 19:
                secondDataByte[6] = mapfloat(ypr[2], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
              case 20: 
                secondDataByte[6] = mapfloat(ypr[1], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
              case 21:
                secondDataByte[6] = mapfloat(ypr[0], dmpMaxInValue[6], dmpMinInValue[6], minOutValue[6], maxOutValue[6]);
                break;
        }
        secondDataByte[6] = constrain(secondDataByte[6], minOutValue[6], maxOutValue[6]);
        switch (commandIdentifier[6]) {
              case 1: 
                if(isMidiOut1[6] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[6], secondDataByte[6], midiChan[6]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[6], secondDataByte[6], midiChan[6]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[6] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                  midiA.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiA.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                  midiB.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiB.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[6] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[6], midiChan[6]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[6], midiChan[6]);
                  midiA.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiA.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[6], midiChan[6]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[6], midiChan[6]);
                  midiB.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiB.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[6] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                  midiA.sendControlChange(6, secondDataByte[6], midiChan[6]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                  midiB.sendControlChange(6, secondDataByte[6], midiChan[6]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[6] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[6], secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiA.sendControlChange(firstDataByteB[6], secondDataByte[6] % nrpnDivisor, midiChan[6]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[6], secondDataByte[6] / nrpnDivisor, midiChan[6]);
                  midiB.sendControlChange(firstDataByteB[6], secondDataByte[6] % nrpnDivisor, midiChan[6]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[6]);                
                break;
              case 7:
                dac.outputB(secondDataByte[6]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[6]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[6]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[6]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[6]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[6]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[6]);                
                break;
        }
      }
    }
    if(mute[7] == false) {
      if(isDmp[7] == true) {
        switch (sensorTypeIdentifier[7]) {
              case 16:
                secondDataByte[7] = mapfloat(ypr[2], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
              case 17:
                secondDataByte[7] = mapfloat(ypr[1], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
              case 18: 
                secondDataByte[7] = mapfloat(ypr[0], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
              case 19:
                secondDataByte[7] = mapfloat(ypr[2], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
              case 20: 
                secondDataByte[7] = mapfloat(ypr[1], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
              case 21:
                secondDataByte[7] = mapfloat(ypr[0], dmpMaxInValue[7], dmpMinInValue[7], minOutValue[7], maxOutValue[7]);
                break;
        }
        secondDataByte[7] = constrain(secondDataByte[7], minOutValue[7], maxOutValue[7]);
        switch (commandIdentifier[7]) {
              case 1: 
                if(isMidiOut1[7] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[7], secondDataByte[7], midiChan[7]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[7], secondDataByte[7], midiChan[7]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[7] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                  midiA.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiA.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                  midiB.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiB.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[7] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[7], midiChan[7]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[7], midiChan[7]);
                  midiA.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiA.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[7], midiChan[7]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[7], midiChan[7]);
                  midiB.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiB.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[7] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                  midiA.sendControlChange(6, secondDataByte[7], midiChan[7]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                  midiB.sendControlChange(6, secondDataByte[7], midiChan[7]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[7] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[7], secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiA.sendControlChange(firstDataByteB[7], secondDataByte[7] % nrpnDivisor, midiChan[7]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[7], secondDataByte[7] / nrpnDivisor, midiChan[7]);
                  midiB.sendControlChange(firstDataByteB[7], secondDataByte[7] % nrpnDivisor, midiChan[7]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[7]);                
                break;
              case 7:
                dac.outputB(secondDataByte[7]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[7]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[7]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[7]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[7]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[7]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[7]);                
                break;
        }
      }
    }
    if(mute[8] == false) {
      if(isDmp[8] == true) {
        switch (sensorTypeIdentifier[8]) {
              case 16:
                secondDataByte[8] = mapfloat(ypr[2], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
              case 17:
                secondDataByte[8] = mapfloat(ypr[1], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
              case 18: 
                secondDataByte[8] = mapfloat(ypr[0], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
              case 19:
                secondDataByte[8] = mapfloat(ypr[2], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
              case 20: 
                secondDataByte[8] = mapfloat(ypr[1], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
              case 21:
                secondDataByte[8] = mapfloat(ypr[0], dmpMaxInValue[8], dmpMinInValue[8], minOutValue[8], maxOutValue[8]);
                break;
        }
        secondDataByte[8] = constrain(secondDataByte[8], minOutValue[8], maxOutValue[8]);
        switch (commandIdentifier[8]) {
              case 1: 
                if(isMidiOut1[8] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[8], secondDataByte[8], midiChan[8]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[8], secondDataByte[8], midiChan[8]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[8] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                  midiA.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiA.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                  midiB.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiB.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[8] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[8], midiChan[8]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[8], midiChan[8]);
                  midiA.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiA.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[8], midiChan[8]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[8], midiChan[8]);
                  midiB.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiB.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[8] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                  midiA.sendControlChange(6, secondDataByte[8], midiChan[8]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                  midiB.sendControlChange(6, secondDataByte[8], midiChan[8]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[8] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[8], secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiA.sendControlChange(firstDataByteB[8], secondDataByte[8] % nrpnDivisor, midiChan[8]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[8], secondDataByte[8] / nrpnDivisor, midiChan[8]);
                  midiB.sendControlChange(firstDataByteB[8], secondDataByte[8] % nrpnDivisor, midiChan[8]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[8]);                
                break;
              case 7:
                dac.outputB(secondDataByte[8]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[8]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[8]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[8]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[8]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[8]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[8]);                
                break;
        }
      }
    }
    if(mute[9] == false) {
      if(isDmp[9] == true) {
        switch (sensorTypeIdentifier[9]) {
              case 16:
                secondDataByte[9] = mapfloat(ypr[2], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
              case 17:
                secondDataByte[9] = mapfloat(ypr[1], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
              case 18: 
                secondDataByte[9] = mapfloat(ypr[0], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
              case 19:
                secondDataByte[9] = mapfloat(ypr[2], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
              case 20: 
                secondDataByte[9] = mapfloat(ypr[1], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
              case 21:
                secondDataByte[9] = mapfloat(ypr[0], dmpMaxInValue[9], dmpMinInValue[9], minOutValue[9], maxOutValue[9]);
                break;
        }
        secondDataByte[9] = constrain(secondDataByte[9], minOutValue[9], maxOutValue[9]);
        switch (commandIdentifier[9]) {
              case 1: 
                if(isMidiOut1[9] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[9], secondDataByte[9], midiChan[9]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[9], secondDataByte[9], midiChan[9]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 2:
                if(isMidiOut1[9] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                  midiA.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiA.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                  midiB.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiB.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 3:
                if(isMidiOut1[9] == true) {
                  midiA.sendControlChange(rpnMsb, firstDataByte[9], midiChan[9]);
                  midiA.sendControlChange(rpnLsb, firstDataByteB[9], midiChan[9]);
                  midiA.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiA.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(rpnMsb, firstDataByte[9], midiChan[9]);
                  midiB.sendControlChange(rpnLsb, firstDataByteB[9], midiChan[9]);
                  midiB.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiB.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 4:
                if(isMidiOut1[9] == true) {
                  midiA.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                  midiA.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                  midiA.sendControlChange(6, secondDataByte[9], midiChan[9]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                  midiB.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                  midiB.sendControlChange(6, secondDataByte[9], midiChan[9]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 5: 
                if(isMidiOut1[9] == true) {
                  // send command via MIDI Out 1
                  midiA.sendControlChange(firstDataByte[9], secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiA.sendControlChange(firstDataByteB[9], secondDataByte[9] % nrpnDivisor, midiChan[9]);
                }
                else {
                  //send command via MIDI Out 2
                  digitalWrite(out2LED, HIGH);
                  midiB.sendControlChange(firstDataByte[9], secondDataByte[9] / nrpnDivisor, midiChan[9]);
                  midiB.sendControlChange(firstDataByteB[9], secondDataByte[9] % nrpnDivisor, midiChan[9]);
                  digitalWrite(out2LED, LOW);
                }
                break;
              case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[9]);                
                break;
              case 7:
                dac.outputB(secondDataByte[9]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[9]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[9]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[9]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[9]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[9]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[9]);                
                break;
        }
      }
    }
  }
}

//====================================================================================
//-                                                               VOID readAnalogIns =
void readAnalogIns (){

  analogPinValue[0] = analogRead(analogInPin0);
  analogPinValue[1] = analogRead(analogInPin1); 
  analogPinValue[2] = analogRead(analogInPin2);
  analogPinValue[3] = analogRead(analogInPin3);
  analogPinValue[4] = analogRead(analogInPin4);
  analogPinValue[5] = analogRead(analogInPin5);
  analogPinValue[6] = analogRead(analogInPin6);
  analogPinValue[7] = analogRead(analogInPin7);

  // Convert Analog In Values to output compatible ones and send them 
  if(mute[0] == false) {
    if(isDmp[0] == false) {
      secondDataByte[0] = map(analogPinValue[sensorTypeIdentifier[0]], minInValue[0], maxInValue[0], minOutValue[0], maxOutValue[0]);
      secondDataByte[0] = constrain(secondDataByte[0], minOutValue[0], maxOutValue[0]);
      switch (commandIdentifier[0]) {
            case 1: 
              if(isMidiOut1[0] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[0], secondDataByte[0], midiChan[0]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[0], secondDataByte[0], midiChan[0]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[0] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                midiA.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiA.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                midiB.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiB.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[0] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[0], midiChan[0]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[0], midiChan[0]);
                midiA.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiA.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[0], midiChan[0]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[0], midiChan[0]);
                midiB.sendControlChange(6, secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiB.sendControlChange(38, secondDataByte[0] % nrpnDivisor, midiChan[0]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[0] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                midiA.sendControlChange(6, secondDataByte[0], midiChan[0]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[0], midiChan[0]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[0], midiChan[0]);
                midiB.sendControlChange(6, secondDataByte[0], midiChan[0]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[0] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[0], secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiA.sendControlChange(firstDataByteB[0], secondDataByte[0] % nrpnDivisor, midiChan[0]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[0], secondDataByte[0] / nrpnDivisor, midiChan[0]);
                midiB.sendControlChange(firstDataByteB[0], secondDataByte[0] % nrpnDivisor, midiChan[0]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[0]);                
                break;
              case 7:
                dac.outputB(secondDataByte[0]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[0]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[0]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[0]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[0]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[0]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[0]);                
                break;
      }      
    }
  }
  if(mute[1] == false) {
    if(isDmp[1] == false) {
      secondDataByte[1] = map(analogPinValue[sensorTypeIdentifier[1]], minInValue[1], maxInValue[1], minOutValue[1], maxOutValue[1]);
      secondDataByte[1] = constrain(secondDataByte[1], minOutValue[1], maxOutValue[1]);
      switch (commandIdentifier[1]) {
            case 1: 
              if(isMidiOut1[1] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[1], secondDataByte[1], midiChan[1]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[1], secondDataByte[1], midiChan[1]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[1] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                midiA.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiA.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                midiB.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiB.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[1] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[1], midiChan[1]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[1], midiChan[1]);
                midiA.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiA.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[1], midiChan[1]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[1], midiChan[1]);
                midiB.sendControlChange(6, secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiB.sendControlChange(38, secondDataByte[1] % nrpnDivisor, midiChan[1]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[1] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                midiA.sendControlChange(6, secondDataByte[1], midiChan[1]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[1], midiChan[1]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[1], midiChan[1]);
                midiB.sendControlChange(6, secondDataByte[1], midiChan[1]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[1] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[1], secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiA.sendControlChange(firstDataByteB[1], secondDataByte[1] % nrpnDivisor, midiChan[1]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[1], secondDataByte[1] / nrpnDivisor, midiChan[1]);
                midiB.sendControlChange(firstDataByteB[1], secondDataByte[1] % nrpnDivisor, midiChan[1]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[1]);                
                break;
              case 7:
                dac.outputB(secondDataByte[1]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[1]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[1]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[1]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[1]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[1]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[1]);                
                break;
      }      
    }
  }
  if(mute[2] == false) {
    if(isDmp[2] == false) {
      secondDataByte[2] = map(analogPinValue[sensorTypeIdentifier[2]], minInValue[2], maxInValue[2], minOutValue[2], maxOutValue[2]);
      secondDataByte[2] = constrain(secondDataByte[2], minOutValue[2], maxOutValue[2]);
      switch (commandIdentifier[2]) {
            case 1: 
              if(isMidiOut1[2] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[2], secondDataByte[2], midiChan[2]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[2], secondDataByte[2], midiChan[2]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[2] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                midiA.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiA.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                midiB.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiB.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[2] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[2], midiChan[2]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[2], midiChan[2]);
                midiA.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiA.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[2], midiChan[2]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[2], midiChan[2]);
                midiB.sendControlChange(6, secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiB.sendControlChange(38, secondDataByte[2] % nrpnDivisor, midiChan[2]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[2] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                midiA.sendControlChange(6, secondDataByte[2], midiChan[2]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[2], midiChan[2]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[2], midiChan[2]);
                midiB.sendControlChange(6, secondDataByte[2], midiChan[2]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[2] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[2], secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiA.sendControlChange(firstDataByteB[2], secondDataByte[2] % nrpnDivisor, midiChan[2]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[2], secondDataByte[2] / nrpnDivisor, midiChan[2]);
                midiB.sendControlChange(firstDataByteB[2], secondDataByte[2] % nrpnDivisor, midiChan[2]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[2]);                
                break;
              case 7:
                dac.outputB(secondDataByte[2]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[2]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[2]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[2]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[2]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[2]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[2]);                
                break;
      }      
    }
  }
  if(mute[3] == false) {
    if(isDmp[3] == false) {
      secondDataByte[3] = map(analogPinValue[sensorTypeIdentifier[3]], minInValue[3], maxInValue[3], minOutValue[3], maxOutValue[3]);
      secondDataByte[3] = constrain(secondDataByte[3], minOutValue[3], maxOutValue[3]);
      switch (commandIdentifier[3]) {
            case 1: 
              if(isMidiOut1[3] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[3], secondDataByte[3], midiChan[3]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[3], secondDataByte[3], midiChan[3]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[3] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                midiA.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiA.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                midiB.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiB.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[3] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[3], midiChan[3]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[3], midiChan[3]);
                midiA.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiA.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[3], midiChan[3]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[3], midiChan[3]);
                midiB.sendControlChange(6, secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiB.sendControlChange(38, secondDataByte[3] % nrpnDivisor, midiChan[3]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[3] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                midiA.sendControlChange(6, secondDataByte[3], midiChan[3]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[3], midiChan[3]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[3], midiChan[3]);
                midiB.sendControlChange(6, secondDataByte[3], midiChan[3]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[3] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[3], secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiA.sendControlChange(firstDataByteB[3], secondDataByte[3] % nrpnDivisor, midiChan[3]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[3], secondDataByte[3] / nrpnDivisor, midiChan[3]);
                midiB.sendControlChange(firstDataByteB[3], secondDataByte[3] % nrpnDivisor, midiChan[3]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[3]);                
                break;
              case 7:
                dac.outputB(secondDataByte[3]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[3]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[3]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[3]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[3]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[3]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[3]);                
                break;
      }      
    }
  }
  if(mute[4] == false) {
    if(isDmp[4] == false) {
      secondDataByte[4] = map(analogPinValue[sensorTypeIdentifier[4]], minInValue[4], maxInValue[4], minOutValue[4], maxOutValue[4]);
      secondDataByte[4] = constrain(secondDataByte[4], minOutValue[4], maxOutValue[4]);
      switch (commandIdentifier[4]) {
            case 1: 
              if(isMidiOut1[4] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[4], secondDataByte[4], midiChan[4]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[4], secondDataByte[4], midiChan[4]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[4] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                midiA.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiA.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                midiB.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiB.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[4] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[4], midiChan[4]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[4], midiChan[4]);
                midiA.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiA.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[4], midiChan[4]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[4], midiChan[4]);
                midiB.sendControlChange(6, secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiB.sendControlChange(38, secondDataByte[4] % nrpnDivisor, midiChan[4]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[4] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                midiA.sendControlChange(6, secondDataByte[4], midiChan[4]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[4], midiChan[4]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[4], midiChan[4]);
                midiB.sendControlChange(6, secondDataByte[4], midiChan[4]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[4] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[4], secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiA.sendControlChange(firstDataByteB[4], secondDataByte[4] % nrpnDivisor, midiChan[4]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[4], secondDataByte[4] / nrpnDivisor, midiChan[4]);
                midiB.sendControlChange(firstDataByteB[4], secondDataByte[4] % nrpnDivisor, midiChan[4]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[4]);                
                break;
              case 7:
                dac.outputB(secondDataByte[4]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[4]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[4]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[4]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[4]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[4]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[4]);                
                break;
      }      
    }
  }
  if(mute[5] == false) {
    if(isDmp[5] == false) {
      secondDataByte[5] = map(analogPinValue[sensorTypeIdentifier[5]], minInValue[5], maxInValue[5], minOutValue[5], maxOutValue[5]);
      secondDataByte[5] = constrain(secondDataByte[5], minOutValue[5], maxOutValue[5]);
      switch (commandIdentifier[5]) {
            case 1: 
              if(isMidiOut1[5] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[5], secondDataByte[5], midiChan[5]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[5], secondDataByte[5], midiChan[5]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[5] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                midiA.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiA.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                midiB.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiB.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[5] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[5], midiChan[5]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[5], midiChan[5]);
                midiA.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiA.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[5], midiChan[5]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[5], midiChan[5]);
                midiB.sendControlChange(6, secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiB.sendControlChange(38, secondDataByte[5] % nrpnDivisor, midiChan[5]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[5] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                midiA.sendControlChange(6, secondDataByte[5], midiChan[5]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[5], midiChan[5]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[5], midiChan[5]);
                midiB.sendControlChange(6, secondDataByte[5], midiChan[5]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[5] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[5], secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiA.sendControlChange(firstDataByteB[5], secondDataByte[5] % nrpnDivisor, midiChan[5]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[5], secondDataByte[5] / nrpnDivisor, midiChan[5]);
                midiB.sendControlChange(firstDataByteB[5], secondDataByte[5] % nrpnDivisor, midiChan[5]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[5]);                
                break;
              case 7:
                dac.outputB(secondDataByte[5]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[5]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[5]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[5]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[5]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[5]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[5]);                
                break;
      }      
    }
  }
  if(mute[6] == false) {
    if(isDmp[6] == false) {
      secondDataByte[6] = map(analogPinValue[sensorTypeIdentifier[6]], minInValue[6], maxInValue[6], minOutValue[6], maxOutValue[6]);
      secondDataByte[6] = constrain(secondDataByte[6], minOutValue[6], maxOutValue[6]);
      switch (commandIdentifier[6]) {
            case 1: 
              if(isMidiOut1[6] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[6], secondDataByte[6], midiChan[6]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[6], secondDataByte[6], midiChan[6]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[6] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                midiA.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiA.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                midiB.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiB.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[6] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[6], midiChan[6]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[6], midiChan[6]);
                midiA.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiA.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[6], midiChan[6]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[6], midiChan[6]);
                midiB.sendControlChange(6, secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiB.sendControlChange(38, secondDataByte[6] % nrpnDivisor, midiChan[6]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[6] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                midiA.sendControlChange(6, secondDataByte[6], midiChan[6]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[6], midiChan[6]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[6], midiChan[6]);
                midiB.sendControlChange(6, secondDataByte[6], midiChan[6]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[6] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[6], secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiA.sendControlChange(firstDataByteB[6], secondDataByte[6] % nrpnDivisor, midiChan[6]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[6], secondDataByte[6] / nrpnDivisor, midiChan[6]);
                midiB.sendControlChange(firstDataByteB[6], secondDataByte[6] % nrpnDivisor, midiChan[6]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[6]);                
                break;
              case 7:
                dac.outputB(secondDataByte[6]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[6]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[6]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[6]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[6]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[6]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[6]);                
                break;
      }      
    }
  }
  if(mute[7] == false) {
    if(isDmp[7] == false) {
      secondDataByte[7] = map(analogPinValue[sensorTypeIdentifier[7]], minInValue[7], maxInValue[7], minOutValue[7], maxOutValue[7]);
      secondDataByte[7] = constrain(secondDataByte[7], minOutValue[7], maxOutValue[7]);
      switch (commandIdentifier[7]) {
            case 1: 
              if(isMidiOut1[7] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[7], secondDataByte[7], midiChan[7]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[7], secondDataByte[7], midiChan[7]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[7] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                midiA.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiA.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                midiB.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiB.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[7] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[7], midiChan[7]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[7], midiChan[7]);
                midiA.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiA.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[7], midiChan[7]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[7], midiChan[7]);
                midiB.sendControlChange(6, secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiB.sendControlChange(38, secondDataByte[7] % nrpnDivisor, midiChan[7]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[7] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                midiA.sendControlChange(6, secondDataByte[7], midiChan[7]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[7], midiChan[7]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[7], midiChan[7]);
                midiB.sendControlChange(6, secondDataByte[7], midiChan[7]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[7] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[7], secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiA.sendControlChange(firstDataByteB[7], secondDataByte[7] % nrpnDivisor, midiChan[7]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[7], secondDataByte[7] / nrpnDivisor, midiChan[7]);
                midiB.sendControlChange(firstDataByteB[7], secondDataByte[7] % nrpnDivisor, midiChan[7]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[7]);                
                break;
              case 7:
                dac.outputB(secondDataByte[7]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[7]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[7]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[7]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[7]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[7]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[7]);                
                break;
      }      
    }
  }
  if(mute[8] == false) {
    if(isDmp[8] == false) {
      secondDataByte[8] = map(analogPinValue[sensorTypeIdentifier[8]], minInValue[8], maxInValue[8], minOutValue[8], maxOutValue[8]);
      secondDataByte[8] = constrain(secondDataByte[8], minOutValue[8], maxOutValue[8]);
      switch (commandIdentifier[8]) {
            case 1: 
              if(isMidiOut1[8] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[8], secondDataByte[8], midiChan[8]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[8], secondDataByte[8], midiChan[8]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[8] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                midiA.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiA.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                midiB.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiB.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[8] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[8], midiChan[8]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[8], midiChan[8]);
                midiA.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiA.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[8], midiChan[8]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[8], midiChan[8]);
                midiB.sendControlChange(6, secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiB.sendControlChange(38, secondDataByte[8] % nrpnDivisor, midiChan[8]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[8] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                midiA.sendControlChange(6, secondDataByte[8], midiChan[8]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[8], midiChan[8]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[8], midiChan[8]);
                midiB.sendControlChange(6, secondDataByte[8], midiChan[8]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[8] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[8], secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiA.sendControlChange(firstDataByteB[8], secondDataByte[8] % nrpnDivisor, midiChan[8]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[8], secondDataByte[8] / nrpnDivisor, midiChan[8]);
                midiB.sendControlChange(firstDataByteB[8], secondDataByte[8] % nrpnDivisor, midiChan[8]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[8]);                
                break;
              case 7:
                dac.outputB(secondDataByte[8]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[8]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[8]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[8]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[8]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[8]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[8]);                
                break;
      }      
    }
  }
  if(mute[9] == false) {
    if(isDmp[9] == false) {
      secondDataByte[9] = map(analogPinValue[sensorTypeIdentifier[9]], minInValue[9], maxInValue[9], minOutValue[9], maxOutValue[9]);
      secondDataByte[9] = constrain(secondDataByte[9], minOutValue[9], maxOutValue[9]);
      switch (commandIdentifier[9]) {
            case 1: 
              if(isMidiOut1[9] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[9], secondDataByte[9], midiChan[9]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[9], secondDataByte[9], midiChan[9]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 2:
              if(isMidiOut1[9] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                midiA.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiA.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                midiB.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiB.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 3:
              if(isMidiOut1[9] == true) {
                midiA.sendControlChange(rpnMsb, firstDataByte[9], midiChan[9]);
                midiA.sendControlChange(rpnLsb, firstDataByteB[9], midiChan[9]);
                midiA.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiA.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(rpnMsb, firstDataByte[9], midiChan[9]);
                midiB.sendControlChange(rpnLsb, firstDataByteB[9], midiChan[9]);
                midiB.sendControlChange(6, secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiB.sendControlChange(38, secondDataByte[9] % nrpnDivisor, midiChan[9]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 4:
              if(isMidiOut1[9] == true) {
                midiA.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                midiA.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                midiA.sendControlChange(6, secondDataByte[9], midiChan[9]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(nrpnMsb, firstDataByte[9], midiChan[9]);
                midiB.sendControlChange(nrpnLsb, firstDataByteB[9], midiChan[9]);
                midiB.sendControlChange(6, secondDataByte[9], midiChan[9]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 5: 
              if(isMidiOut1[9] == true) {
                // send command via MIDI Out 1
                midiA.sendControlChange(firstDataByte[9], secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiA.sendControlChange(firstDataByteB[9], secondDataByte[9] % nrpnDivisor, midiChan[9]);
              }
              else {
                //send command via MIDI Out 2
                digitalWrite(out2LED, HIGH);
                midiB.sendControlChange(firstDataByte[9], secondDataByte[9] / nrpnDivisor, midiChan[9]);
                midiB.sendControlChange(firstDataByteB[9], secondDataByte[9] % nrpnDivisor, midiChan[9]);
                digitalWrite(out2LED, LOW);
              }
              break;
            case 6:
                // CV OUT 1
                dac.outputA(secondDataByte[9]);                
                break;
              case 7:
                dac.outputB(secondDataByte[9]);                
                break;
              case 8:
                dac2.outputA(secondDataByte[9]);                
                break;
              case 9:
                dac2.outputB(secondDataByte[9]);                
                break;
              case 10:
                dac3.outputA(secondDataByte[9]);                
                break;
              case 11:
                dac3.outputB(secondDataByte[9]);                
                break;
              case 12:
                dac4.outputA(secondDataByte[9]);                
                break;
              case 13:
                dac4.outputB(secondDataByte[9]);                
                break;
      }      
    }
  }
}

//====================================================================================
//-                                                                   GRAPHICS STUFF =

//DRAW LIVE VALUES
void drawLiveValuesA () {
  
  tft.drawFastVLine(locator, 56, 128, BLACK);
  
  switch (commandIdentifier[0]) {
    case 1:
      tft.drawPixel(locator, 183 - secondDataByte[0], COLOR1);
      break;
    case 2:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 3:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 4:
      tft.drawPixel(locator, 183 - secondDataByte[0], COLOR1);
      break;
    case 5:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 6:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 7:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 8:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 9:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 10:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 11:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 12:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
    case 13:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR1);
      break;
  }
  switch (commandIdentifier[1]) {
    case 1:
      tft.drawPixel(locator, 183 - secondDataByte[1], COLOR2);
      break;
    case 2:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 3:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 4:
      tft.drawPixel(locator, 183 - secondDataByte[1], COLOR2);
      break;
    case 5:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 6:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 7:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 8:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 9:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 10:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 11:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 12:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
    case 13:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR2);
      break;
  }
  switch (commandIdentifier[2]) {
    case 1:
      tft.drawPixel(locator, 183 - secondDataByte[2], COLOR3);
      break;
    case 2:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 3:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 4:
      tft.drawPixel(locator, 183 - secondDataByte[2], COLOR3);
      break;
    case 5:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 6:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 7:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 8:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 9:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 10:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 11:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 12:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
    case 13:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR3);
      break;
  }
  switch (commandIdentifier[3]) {
    case 1:
      tft.drawPixel(locator, 183 - secondDataByte[3], COLOR4);
      break;
    case 2:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 3:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 4:
      tft.drawPixel(locator, 183 - secondDataByte[3], COLOR4);
      break;
    case 5:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 6:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 7:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 8:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 9:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 10:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 11:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 12:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
    case 13:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR4);
      break;
  }
  switch (commandIdentifier[4]) {
    case 1:
      tft.drawPixel(locator, 183 - secondDataByte[4], COLOR5);
      break;
    case 2:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 3:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 4:
      tft.drawPixel(locator, 183 - secondDataByte[4], COLOR5);
      break;
    case 5:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 6:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 7:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 8:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 9:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 10:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 11:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 12:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
    case 13:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 183 - iBuff, COLOR5);
      break;
  }
  
  tft.drawFastVLine(locator, 188, 128, BLACK);
  
  switch (commandIdentifier[5]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[5], COLOR6);
      break;
    case 2:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 3:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[5], COLOR6);
      break;
    case 5:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 6:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 7:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 8:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 9:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 10:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 11:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 12:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 13:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
  }
  switch (commandIdentifier[6]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[6], COLOR7);
      break;
    case 2:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 3:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[6], COLOR7);
      break;
    case 5:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 6:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 7:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 8:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 9:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 10:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 11:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 12:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 13:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
  }
  switch (commandIdentifier[7]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[7], COLOR8);
      break;
    case 2:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 3:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[7], COLOR8);
      break;
    case 5:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 6:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 7:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 8:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 9:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 10:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 11:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 12:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 13:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
  }
  switch (commandIdentifier[8]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[8], COLOR9);
      break;
    case 2:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 3:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[8], COLOR9);
      break;
    case 5:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 6:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 7:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 8:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 9:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 10:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 11:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 12:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 13:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
  }
  switch (commandIdentifier[9]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[9], COLOR0);
      break;
    case 2:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 3:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[9], COLOR0);
      break;
    case 5:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 6:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 7:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 8:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 9:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 10:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 11:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 12:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 13:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
  }
}

// DRAW LIVE VALUES B
void drawLiveValuesB (){
  
  tft.drawFastVLine(locator, 30, 128, BLACK);
  
  switch (commandIdentifier[0]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[0], COLOR1);
      break;
    case 2:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 3:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[0], COLOR1);
      break;
    case 5:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 6:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 7:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 8:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 9:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 10:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 11:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 12:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 13:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
  }
  switch (commandIdentifier[1]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[1], COLOR2);
      break;
    case 2:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 3:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[1], COLOR2);
      break;
    case 5:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 6:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 7:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 8:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 9:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 10:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 11:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 12:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 13:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
  }
  switch (commandIdentifier[2]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[2], COLOR3);
      break;
    case 2:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 3:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[2], COLOR3);
      break;
    case 5:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 6:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 7:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 8:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 9:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 10:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 11:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 12:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 13:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
  }
  switch (commandIdentifier[3]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[3], COLOR4);
      break;
    case 2:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 3:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[3], COLOR4);
      break;
    case 5:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 6:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 7:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 8:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 9:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 10:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 11:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 12:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 13:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
  }
  switch (commandIdentifier[4]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[4], COLOR5);
      break;
    case 2:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 3:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[4], COLOR5);
      break;
    case 5:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 6:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 7:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 8:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 9:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 10:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 11:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 12:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 13:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
  }
  
  tft.drawFastVLine(locator, 188, 128, BLACK);
  
  switch (commandIdentifier[5]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[5], COLOR6);
      break;
    case 2:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 3:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[5], COLOR6);
      break;
    case 5:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 6:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 7:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 8:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 9:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 10:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 11:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 12:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
    case 13:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR6);
      break;
  }
  switch (commandIdentifier[6]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[6], COLOR7);
      break;
    case 2:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 3:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[6], COLOR7);
      break;
    case 5:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 6:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 7:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 8:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 9:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 10:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 11:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 12:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
    case 13:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR7);
      break;
  }
  switch (commandIdentifier[7]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[7], COLOR8);
      break;
    case 2:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 3:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[7], COLOR8);
      break;
    case 5:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 6:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 7:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 8:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 9:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 10:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 11:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 12:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
    case 13:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR8);
      break;
  }
  switch (commandIdentifier[8]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[8], COLOR9);
      break;
    case 2:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 3:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[8], COLOR9);
      break;
    case 5:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 6:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 7:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 8:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 9:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 10:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 11:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 12:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
    case 13:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR9);
      break;
  }
  switch (commandIdentifier[9]) {
    case 1:
      tft.drawPixel(locator, 315 - secondDataByte[9], COLOR0);
      break;
    case 2:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 3:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 4:
      tft.drawPixel(locator, 315 - secondDataByte[9], COLOR0);
      break;
    case 5:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 6:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 7:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 8:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 9:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 10:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 11:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 12:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
    case 13:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator, 315 - iBuff, COLOR0);
      break;
  }
}

//DRAW LIVE VALUES C
void drawLiveValuesC (){
  
  tft.drawFastVLine(locator, 30, 128, BLACK);
  
  switch (commandIdentifier[0]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[0], COLOR1);
      break;
    case 2:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 3:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[0], COLOR1);
      break;
    case 5:
      iBuff = secondDataByte[0] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 6:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 7:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 8:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 9:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 10:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 11:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 12:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
    case 13:
      iBuff = secondDataByte[0] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR1);
      break;
  }
  switch (commandIdentifier[1]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[1], COLOR2);
      break;
    case 2:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 3:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[1], COLOR2);
      break;
    case 5:
      iBuff = secondDataByte[1] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 6:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 7:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 8:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 9:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 10:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 11:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 12:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
    case 13:
      iBuff = secondDataByte[1] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR2);
      break;
  }
  switch (commandIdentifier[2]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[2], COLOR3);
      break;
    case 2:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 3:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[2], COLOR3);
      break;
    case 5:
      iBuff = secondDataByte[2] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 6:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 7:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 8:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 9:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 10:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 11:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 12:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
    case 13:
      iBuff = secondDataByte[2] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR3);
      break;
  }
  switch (commandIdentifier[3]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[3], COLOR4);
      break;
    case 2:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 3:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[3], COLOR4);
      break;
    case 5:
      iBuff = secondDataByte[3] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 6:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 7:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 8:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 9:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 10:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 11:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 12:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
    case 13:
      iBuff = secondDataByte[3] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR4);
      break;
  }
  switch (commandIdentifier[4]) {
    case 1:
      tft.drawPixel(locator, 157 - secondDataByte[4], COLOR5);
      break;
    case 2:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 3:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 4:
      tft.drawPixel(locator, 157 - secondDataByte[4], COLOR5);
      break;
    case 5:
      iBuff = secondDataByte[4] >> 7;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 6:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 7:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 8:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 9:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 10:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 11:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 12:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
    case 13:
      iBuff = secondDataByte[4] >> 5;
      tft.drawPixel(locator, 157 - iBuff, COLOR5);
      break;
  }
  
  tft.drawFastVLine(locator2, 30, 128, BLACK);
  
  switch (commandIdentifier[5]) {
    case 1:
      tft.drawPixel(locator2, 157 - secondDataByte[5], COLOR6);
      break;
    case 2:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 3:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 4:
      tft.drawPixel(locator2, 157 - secondDataByte[5], COLOR6);
      break;
    case 5:
      iBuff = secondDataByte[5] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 6:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 7:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 8:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 9:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 10:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 11:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 12:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
    case 13:
      iBuff = secondDataByte[5] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR6);
      break;
  }
  switch (commandIdentifier[6]) {
    case 1:
      tft.drawPixel(locator2, 157 - secondDataByte[6], COLOR7);
      break;
    case 2:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 3:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 4:
      tft.drawPixel(locator2, 157 - secondDataByte[6], COLOR7);
      break;
    case 5:
      iBuff = secondDataByte[6] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 6:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 7:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 8:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 9:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 10:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 11:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 12:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
    case 13:
      iBuff = secondDataByte[6] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR7);
      break;
  }
  switch (commandIdentifier[7]) {
    case 1:
      tft.drawPixel(locator2, 157 - secondDataByte[7], COLOR8);
      break;
    case 2:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 3:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 4:
      tft.drawPixel(locator2, 157 - secondDataByte[7], COLOR8);
      break;
    case 5:
      iBuff = secondDataByte[7] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 6:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 7:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 8:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 9:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 10:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 11:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 12:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
    case 13:
      iBuff = secondDataByte[7] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR8);
      break;
  }
  switch (commandIdentifier[8]) {
    case 1:
      tft.drawPixel(locator2, 157 - secondDataByte[8], COLOR9);
      break;
    case 2:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 3:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 4:
      tft.drawPixel(locator2, 157 - secondDataByte[8], COLOR9);
      break;
    case 5:
      iBuff = secondDataByte[8] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 6:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 7:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 8:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 9:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 10:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 11:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 12:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
    case 13:
      iBuff = secondDataByte[8] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR9);
      break;
  }
  switch (commandIdentifier[9]) {
    case 1:
      tft.drawPixel(locator2, 157 - secondDataByte[9], COLOR0);
      break;
    case 2:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 3:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 4:
      tft.drawPixel(locator2, 157 - secondDataByte[9], COLOR0);
      break;
    case 5:
      iBuff = secondDataByte[9] >> 7;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 6:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 7:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 8:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 9:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 10:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 11:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 12:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
    case 13:
      iBuff = secondDataByte[9] >> 5;
      tft.drawPixel(locator2, 157 - iBuff, COLOR0);
      break;
  }
}

// DRAW GRAPHIC ELEMENTS
void drawShiftArrow(){
  tft.fillRect(22, 237, 3, 3, currentColor);
  tft.fillRect(25, 240, 3, 3, currentColor);
  tft.fillRect(28, 243, 3, 3, currentColor);
  tft.fillRect(31, 246, 3, 3, currentColor);
  tft.fillRect(28, 249, 9, 3, currentColor);
  tft.fillRect(28, 252, 3, 3, currentColor);
  tft.fillRect(16, 255, 15, 3, currentColor);
  tft.fillRect(16, 252, 3, 3, currentColor);
  tft.fillRect(10, 249, 9, 3, currentColor);
  tft.fillRect(13, 246, 3, 3, currentColor);
  tft.fillRect(16, 243, 3, 3, currentColor);
  tft.fillRect(19, 240, 3, 3, currentColor);
}

void drawBackspaceArrow(){
  tft.fillRect(409, 234, 3, 9, currentColor);
  tft.fillRect(410, 240, 9, 3, currentColor);
  tft.fillRect(416, 243, 3, 9, currentColor);
  tft.fillRect(410, 252, 9, 3, currentColor);
  tft.fillRect(407, 252, 3, 9, currentColor);
  tft.fillRect(404, 255, 3, 3, currentColor);
  tft.fillRect(401, 252, 3, 3, currentColor);
  tft.fillRect(398, 249, 3, 3, currentColor);
  tft.fillRect(395, 246, 3, 3, currentColor);
  tft.fillRect(398, 243, 3, 3, currentColor);
  tft.fillRect(401, 240, 3, 3, currentColor);
  tft.fillRect(404, 237, 3, 3, currentColor);
}

//====================================================================================
//-                                                              VOID buildLiveModeA =
void buildLiveModeA(){

  tft.fillScreen(BLACK);

  // draw horizontal bars
  tft.fillRect(0, 0, 480, 4, LTGRAY);
  tft.fillRect(0, 52, 480, 4, LTGRAY);
  tft.fillRect(0, 184, 480, 4, LTGRAY);
  tft.fillRect(0, 316, 480, 2, LTGRAY);
  tft.fillRect(0, 318, 48, 2, COLOR1);
  tft.fillRect(48, 318, 48, 2, COLOR2);
  tft.fillRect(96, 318, 48, 2, COLOR3);
  tft.fillRect(144, 318, 48, 2, COLOR4);
  tft.fillRect(192, 318, 48, 2, COLOR5);
  tft.fillRect(240, 318, 48, 2, COLOR6);
  tft.fillRect(288, 318, 48, 2, COLOR7);
  tft.fillRect(336, 318, 48, 2, COLOR8);
  tft.fillRect(384, 318, 48, 2, COLOR9);
  tft.fillRect(432, 318, 48, 2, COLOR0);

  // draw button boxes
  tft.fillRect(0, 4, 48, 48, COLOR1);
  tft.fillRect(48, 4, 48, 48, COLOR2);
  tft.fillRect(96, 4, 48, 48, COLOR3);
  tft.fillRect(144, 4, 48, 48, COLOR4);
  tft.fillRect(192, 4, 48, 48, COLOR5);
  tft.fillRect(240, 4, 48, 48, COLOR6);
  tft.fillRect(288, 4, 48, 48, COLOR7);
  tft.fillRect(336, 4, 48, 48, COLOR8);
  tft.fillRect(384, 4, 48, 48, COLOR9);
  tft.fillRect(432, 4, 48, 48, COLOR0);

  tft.setTextColor(BLACK);
  tft.setTextSize(1);

  // draw MUTE indicators
  if(mute[0] == true) {
    tft.fillRect(0, 40, 48, 12, WHITE);
    tft.setCursor(4, 41);
    tft.print("MUTED"); 
  }
  if(mute[1] == true) {
    tft.fillRect(48, 40, 48, 12, WHITE);
    tft.setCursor(52, 41);
    tft.print("MUTED"); 
  }
  if(mute[2] == true) {
    tft.fillRect(96, 40, 48, 12, WHITE);
    tft.setCursor(100, 41);
    tft.print("MUTED"); 
  }
  if(mute[3] == true) {
    tft.fillRect(144, 40, 48, 12, WHITE);
    tft.setCursor(148, 41);
    tft.print("MUTED"); 
  }
  if(mute[4] == true) {
    tft.fillRect(192, 40, 48, 12, WHITE);
    tft.setCursor(196, 41);
    tft.print("MUTED"); 
  }
  if(mute[5] == true) {
    tft.fillRect(240, 40, 48, 12, WHITE);
    tft.setCursor(244, 41);
    tft.print("MUTED"); 
  }
  if(mute[6] == true) {
    tft.fillRect(288, 40, 48, 12, WHITE);
    tft.setCursor(292, 41);
    tft.print("MUTED"); 
  }
  if(mute[7] == true) {
    tft.fillRect(336, 40, 48, 12, WHITE);
    tft.setCursor(340, 41);
    tft.print("MUTED"); 
  }
  if(mute[8] == true) {
    tft.fillRect(384, 40, 48, 12, WHITE);
    tft.setCursor(388, 41);
    tft.print("MUTED"); 
  }
  if(mute[9] == true) {
    tft.fillRect(432, 40, 48, 12, WHITE);
    tft.setCursor(436, 41);
    tft.print("MUTED");   
  }

  // draw box labels
  tft.setCursor(4, 8);
  tft.print(patchNames[0]);
  tft.setCursor(4, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[0]]); 
  tft.setCursor(52, 8);
  tft.print(patchNames[1]);
  tft.setCursor(52, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[1]]); 
  tft.setCursor(100, 8);
  tft.print(patchNames[2]);
  tft.setCursor(100, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[2]]); 
  tft.setCursor(148, 8); 
  tft.print(patchNames[3]);
  tft.setCursor(148, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[3]]); 
  tft.setCursor(196, 8); 
  tft.print(patchNames[4]);
  tft.setCursor(196, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[4]]); 
  tft.setCursor(244, 8); 
  tft.print(patchNames[5]);
  tft.setCursor(244, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[5]]); 
  tft.setCursor(292, 8); 
  tft.print(patchNames[6]);
  tft.setCursor(292, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[6]]); 
  tft.setCursor(340, 8); 
  tft.print(patchNames[7]);
  tft.setCursor(340, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[7]]); 
  tft.setCursor(388, 8); 
  tft.print(patchNames[8]); 
  tft.setCursor(388, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[8]]);
  tft.setCursor(436, 8); 
  tft.print(patchNames[9]);
  tft.setCursor(436, 19);
  tft.print(sensorTypes[sensorTypeIdentifier[9]]); 
}

//====================================================================================
//-                                                              VOID buildLiveModeB =
void buildLiveModeB(){

  tft.fillScreen(BLACK);

  // draw bars
  tft.drawFastVLine(96, 0, 320, LTGRAY);
  tft.drawFastVLine(382, 0, 320, LTGRAY);
  tft.drawRect(96, 0, 287, 30, LTGRAY);
  tft.drawRect(96, 158, 287, 30, LTGRAY);
  tft.fillRect(97, 316, 286, 4, LTGRAY);

  // draw buttons
  tft.fillTriangle(1, 62, 17, 62, 1, 46, COLOR1);
  tft.drawRect(0, 0, 96, 64, COLOR1);
  tft.drawRect(1, 1, 94, 62, COLOR1);
  tft.fillTriangle(1, 126, 17, 126, 1, 110, COLOR2);
  tft.drawRect(0, 64, 96, 64, COLOR2);
  tft.drawRect(1, 65, 94, 62, COLOR2);
  tft.fillTriangle(1, 190, 17, 190, 1, 174, COLOR3);
  tft.drawRect(0, 128, 96, 64, COLOR3);
  tft.drawRect(1, 129, 94, 62, COLOR3);
  tft.fillTriangle(1, 254, 17, 254, 1, 238, COLOR4);
  tft.drawRect(0, 192, 96, 64, COLOR4);
  tft.drawRect(1, 193, 94, 62, COLOR4);
  tft.fillTriangle(1, 318, 17, 318, 1, 302, COLOR5);
  tft.drawRect(0, 256, 96, 64, COLOR5);
  tft.drawRect(1, 257, 94, 62, COLOR5);
  tft.fillTriangle(384, 62, 400, 62, 384, 46, COLOR6);
  tft.drawRect(383, 0, 96, 64, COLOR6);
  tft.drawRect(384, 1, 94, 62, COLOR6);
  tft.fillTriangle(384, 126, 400, 126, 384, 110, COLOR7);
  tft.drawRect(383, 64, 96, 64, COLOR7);
  tft.drawRect(384, 65, 94, 62, COLOR7);
  tft.fillTriangle(384, 190, 400, 190, 384, 174, COLOR8);
  tft.drawRect(383, 128, 96, 64, COLOR8);
  tft.drawRect(384, 129, 94, 62, COLOR8);
  tft.fillTriangle(384, 254, 400, 254, 384, 238, COLOR9);
  tft.drawRect(383, 192, 96, 64, COLOR9);
  tft.drawRect(384, 193, 94, 62, COLOR9);
  tft.fillTriangle(384, 318, 400, 318, 384, 302, COLOR0);
  tft.drawRect(383, 256, 96, 64, COLOR0);
  tft.drawRect(384, 257, 94, 62, COLOR0);

  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(186, 8);
  tft.print("Patch 1-5");
  tft.setCursor(180, 166);
  tft.print("Patch 6-10");
  
  tft.setTextSize(3);

  // draw MUTE indicators
  if(mute[0] == true) {
    tft.setTextColor(COLOR1);
    tft.setCursor(75, 41);
    tft.print("="); 
  }
  if(mute[1] == true) {
    tft.setTextColor(COLOR2);
    tft.setCursor(75, 105);
    tft.print("="); 
  }
  if(mute[2] == true) {
    tft.setTextColor(COLOR3);
    tft.setCursor(75, 169);
    tft.print("="); 
  }
  if(mute[3] == true) {
    tft.setTextColor(COLOR4);
    tft.setCursor(75, 233);
    tft.print("="); 
  }
  if(mute[4] == true) {
    tft.setTextColor(COLOR5);
    tft.setCursor(75, 297);
    tft.print("="); 
  }
  if(mute[5] == true) {
    tft.setTextColor(COLOR6);
    tft.setCursor(458, 41);
    tft.print("="); 
  }
  if(mute[6] == true) {
    tft.setTextColor(COLOR7);
    tft.setCursor(458, 105);
    tft.print("="); 
  }
  if(mute[7] == true) {
    tft.setTextColor(COLOR8);
    tft.setCursor(458, 169);
    tft.print("="); 
  }
  if(mute[8] == true) {
    tft.setTextColor(COLOR9);
    tft.setCursor(458, 233);
    tft.print("="); 
  }
  if(mute[9] == true) {
    tft.setTextColor(COLOR0);
    tft.setCursor(458, 297);
    tft.print("=");   
  }
  
  // print patch names
  tft.setTextColor(LTGRAY);
  tft.setTextSize(2);
  tft.setCursor(6, 6);
  tft.print(patchNames[0]);
  tft.setCursor(6, 70);
  tft.print(patchNames[1]);
  tft.setCursor(6, 134);
  tft.print(patchNames[2]);
  tft.setCursor(6, 198);
  tft.print(patchNames[3]);
  tft.setCursor(6, 262);
  tft.print(patchNames[4]);
  tft.setCursor(389, 6);
  tft.print(patchNames[5]);
  tft.setCursor(389, 70);
  tft.print(patchNames[6]);
  tft.setCursor(389, 134);
  tft.print(patchNames[7]);
  tft.setCursor(389, 198);
  tft.print(patchNames[8]);
  tft.setCursor(389, 262);
  tft.print(patchNames[9]);
  
  // print linked sensors
  tft.setTextSize(1);
  tft.setCursor(6, 24);
  tft.print(sensorTypes[sensorTypeIdentifier[0]]); 
  tft.setCursor(6, 88);
  tft.print(sensorTypes[sensorTypeIdentifier[1]]); 
  tft.setCursor(6, 152);
  tft.print(sensorTypes[sensorTypeIdentifier[2]]); 
  tft.setCursor(6, 216);
  tft.print(sensorTypes[sensorTypeIdentifier[3]]); 
  tft.setCursor(6, 280);
  tft.print(sensorTypes[sensorTypeIdentifier[4]]); 
  tft.setCursor(389, 24);
  tft.print(sensorTypes[sensorTypeIdentifier[5]]); 
  tft.setCursor(389, 88);
  tft.print(sensorTypes[sensorTypeIdentifier[6]]); 
  tft.setCursor(389, 152);
  tft.print(sensorTypes[sensorTypeIdentifier[7]]); 
  tft.setCursor(389, 216);
  tft.print(sensorTypes[sensorTypeIdentifier[8]]); 
  tft.setCursor(389, 280);
  tft.print(sensorTypes[sensorTypeIdentifier[9]]); 
  
  // print output configuration
  tft.setCursor(6, 34);
  tft.print("CH");
  tft.print(midiChan[0]); 
  if (commandIdentifier[0] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[0]);
  }
  else if (commandIdentifier[0] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[0]);
    tft.print(" L");
    tft.print(firstDataByteB[0]);
  }
  tft.setCursor(6, 98);
  tft.print("CH");
  tft.print(midiChan[1]);
  if (commandIdentifier[1] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[1]);
  }
  else if (commandIdentifier[1] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[1]);
    tft.print(" L");
    tft.print(firstDataByteB[1]);
  }
  tft.setCursor(6, 162);
  tft.print("CH");
  tft.print(midiChan[2]);
  if (commandIdentifier[2] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[2]);
  }
  else if (commandIdentifier[2] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[2]);
    tft.print(" L");
    tft.print(firstDataByteB[2]);
  }
  tft.setCursor(6, 226);
  tft.print("CH");
  tft.print(midiChan[3]);
  if (commandIdentifier[3] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[3]);
  }
  else if (commandIdentifier[3] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[3]);
    tft.print(" L");
    tft.print(firstDataByteB[3]);
  }
  tft.setCursor(6, 290);
  tft.print("CH");
  tft.print(midiChan[4]);
  if (commandIdentifier[4] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[4]);
  }
  else if (commandIdentifier[4] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[4]);
    tft.print(" L");
    tft.print(firstDataByteB[4]);
  }
  tft.setCursor(389, 34);
  tft.print("CH");
  tft.print(midiChan[5]);
  if (commandIdentifier[5] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[5]);
  }
  else if (commandIdentifier[5] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[5]);
    tft.print(" L");
    tft.print(firstDataByteB[5]);
  }
  tft.setCursor(389, 98);
  tft.print("CH");
  tft.print(midiChan[6]);
  if (commandIdentifier[6] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[6]);
  }
  else if (commandIdentifier[6] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[6]);
    tft.print(" L");
    tft.print(firstDataByteB[6]);
  }
  tft.setCursor(389, 162);
  tft.print("CH");
  tft.print(midiChan[7]);
  if (commandIdentifier[7] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[7]);
  }
  else if (commandIdentifier[7] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[7]);
    tft.print(" L");
    tft.print(firstDataByteB[7]);
  } 
  tft.setCursor(389, 226);
  tft.print("CH");
  tft.print(midiChan[8]);
  if (commandIdentifier[8] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[8]);
  }
  else if (commandIdentifier[8] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[8]);
    tft.print(" L");
    tft.print(firstDataByteB[8]);
  } 
  tft.setCursor(389, 290);
  tft.print("CH");
  tft.print(midiChan[9]);
  if (commandIdentifier[9] == 1) {
    tft.print(" CC");
    tft.print(firstDataByte[9]);
  }
  else if (commandIdentifier[9] > 1) {
    tft.print(" M");
    tft.print(firstDataByte[9]);
    tft.print(" L");
    tft.print(firstDataByteB[9]);
  }
}

//====================================================================================
//-                                                              VOID buildLiveModeC =
void buildLiveModeC(){
  char arrows = 18;

  tft.fillScreen(BLACK);

  // draw horizontal bars
  tft.fillRect(0, 10, 480, 20, LTGRAY);
  tft.fillTriangle(220, 10, 239, 29, 239, 10, BLACK);
  tft.fillTriangle(259, 10, 240, 29, 240, 10, BLACK);
  tft.drawFastVLine(0, 30, 128, LTGRAY);
  tft.drawFastVLine(239, 30, 128, LTGRAY);
  tft.drawFastVLine(240, 30, 128, LTGRAY);
  tft.drawFastVLine(479, 30, 128, LTGRAY);
  tft.fillRect(0, 158, 480, 2, LTGRAY);
  
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 13);
  tft.print("Patch 1-5");
  tft.setCursor(358, 13);
  tft.print("Patch 6-10");

  // draw button boxes
  tft.drawRect(0, 168, 48, 48, COLOR1);
  tft.drawRect(48, 168, 48, 48, COLOR2);
  tft.drawRect(96, 168, 48, 48, COLOR3);
  tft.drawRect(144, 168, 48, 48, COLOR4);
  tft.drawRect(192, 168, 48, 48, COLOR5);
  tft.drawRect(240, 168, 48, 48, COLOR6);
  tft.drawRect(288, 168, 48, 48, COLOR7);
  tft.drawRect(336, 168, 48, 48, COLOR8);
  tft.drawRect(384, 168, 48, 48, COLOR9);
  tft.drawRect(432, 168, 48, 48, COLOR0);
  tft.drawRect(1, 169, 46, 46, COLOR1);
  tft.drawRect(49, 169, 46, 46, COLOR2);
  tft.drawRect(97, 169, 46, 46, COLOR3);
  tft.drawRect(145, 169, 46, 46, COLOR4);
  tft.drawRect(193, 169, 46, 46, COLOR5);
  tft.drawRect(241, 169, 46, 46, COLOR6);
  tft.drawRect(289, 169, 46, 46, COLOR7);
  tft.drawRect(337, 169, 46, 46, COLOR8);
  tft.drawRect(385, 169, 46, 46, COLOR9);
  tft.drawRect(433, 169, 46, 46, COLOR0);
  
  tft.fillRect(0, 214, 48, 48, COLOR1);
  tft.fillRect(48, 214, 48, 48, COLOR2);
  tft.fillRect(96, 214, 48, 48, COLOR3);
  tft.fillRect(144, 214, 48, 48, COLOR4);
  tft.fillRect(192, 214, 48, 48, COLOR5);
  tft.fillRect(240, 214, 48, 48, COLOR6);
  tft.fillRect(288, 214, 48, 48, COLOR7);
  tft.fillRect(336, 214, 48, 48, COLOR8);
  tft.fillRect(384, 214, 48, 48, COLOR9);
  tft.fillRect(432, 214, 48, 48, COLOR0);
  
  tft.drawRect(0, 260, 48, 48, COLOR1);
  tft.drawRect(48, 260, 48, 48, COLOR2);
  tft.drawRect(96, 260, 48, 48, COLOR3);
  tft.drawRect(144, 260, 48, 48, COLOR4);
  tft.drawRect(192, 260, 48, 48, COLOR5);
  tft.drawRect(240, 260, 48, 48, COLOR6);
  tft.drawRect(288, 260, 48, 48, COLOR7);
  tft.drawRect(336, 260, 48, 48, COLOR8);
  tft.drawRect(384, 260, 48, 48, COLOR9);
  tft.drawRect(432, 260, 48, 48, COLOR0);
  tft.drawRect(1, 261, 46, 46, COLOR1);
  tft.drawRect(49, 261, 46, 46, COLOR2);
  tft.drawRect(97, 261, 46, 46, COLOR3);
  tft.drawRect(145, 261, 46, 46, COLOR4);
  tft.drawRect(193, 261, 46, 46, COLOR5);
  tft.drawRect(241, 261, 46, 46, COLOR6);
  tft.drawRect(289, 261, 46, 46, COLOR7);
  tft.drawRect(337, 261, 46, 46, COLOR8);
  tft.drawRect(385, 261, 46, 46, COLOR9);
  tft.drawRect(433, 261, 46, 46, COLOR0);
  
  tft.setTextSize(3);

  // draw MUTE indicators
  tft.setCursor(16, 180);
  if(mute[0] == true) {
    tft.setTextColor(COLOR1);
    tft.print("=");
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(64, 180);
  if(mute[1] == true) {
    tft.setTextColor(COLOR2);
    tft.print("=");
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(112, 180);
  if(mute[2] == true) {
    tft.setTextColor(COLOR3);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(160, 180);
  if(mute[3] == true) {
    tft.setTextColor(COLOR4);
    tft.print("=");
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(208, 180);
  if(mute[4] == true) {
    tft.setTextColor(COLOR5);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(256, 180);
  if(mute[5] == true) {
    tft.setTextColor(COLOR6);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(304, 180);
  if(mute[6] == true) {
    tft.setTextColor(COLOR7);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(352, 180);
  if(mute[7] == true) {
    tft.setTextColor(COLOR8);
    tft.print("=");
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(400, 180);
  if(mute[8] == true) {
    tft.setTextColor(COLOR9);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }
  tft.setCursor(448, 180);
  if(mute[9] == true) {
    tft.setTextColor(COLOR0);
    tft.print("="); 
  }
  else {
    tft.setTextColor(GRAY);
    tft.print("=");
  }

  // draw box labels
  tft.setTextColor(BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, 228);
  tft.print(patchNames[0]);
  tft.setCursor(4, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[0]]); 
  tft.setCursor(52, 228);
  tft.print(patchNames[1]);
  tft.setCursor(52, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[1]]); 
  tft.setCursor(100, 228);
  tft.print(patchNames[2]);
  tft.setCursor(100, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[2]]); 
  tft.setCursor(148, 228); 
  tft.print(patchNames[3]);
  tft.setCursor(148, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[3]]); 
  tft.setCursor(196, 228); 
  tft.print(patchNames[4]);
  tft.setCursor(196, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[4]]); 
  tft.setCursor(244, 228); 
  tft.print(patchNames[5]);
  tft.setCursor(244, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[5]]); 
  tft.setCursor(292, 228); 
  tft.print(patchNames[6]);
  tft.setCursor(292, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[6]]); 
  tft.setCursor(340, 228); 
  tft.print(patchNames[7]);
  tft.setCursor(340, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[7]]); 
  tft.setCursor(388, 228); 
  tft.print(patchNames[8]); 
  tft.setCursor(388, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[8]]);
  tft.setCursor(436, 228); 
  tft.print(patchNames[9]);
  tft.setCursor(436, 239);
  tft.print(sensorTypes[sensorTypeIdentifier[9]]); 
  
  tft.setTextColor(GRAY);
  tft.setTextSize(2);
  tft.setCursor(19, 276);
  tft.print(arrows);
  tft.setCursor(67, 276);
  tft.print(arrows);
  tft.setCursor(115, 276);
  tft.print(arrows);
  tft.setCursor(163, 276);
  tft.print(arrows);
  tft.setCursor(211, 276);
  tft.print(arrows);
  tft.setCursor(259, 276);
  tft.print(arrows);
  tft.setCursor(307, 276);
  tft.print(arrows);
  tft.setCursor(355, 276);
  tft.print(arrows);
  tft.setCursor(403, 276);
  tft.print(arrows);
  tft.setCursor(451, 276);
  tft.print(arrows);
}

//====================================================================================
//-                                                              MATH STUFF AND MORE =
// DIGIT COUNTER
byte countDigits(int num) {
  byte count = 0;
  while(num) {
    num = num / 10;
    count++;
  }
  if(count == 0) {
    return 1;
  } 
  else {
    return count;
  }
}

// MAP FLOAT VALUES TO INT
int mapfloat(float x, float inMin, float inMax, float outMin, float outMax)
{
 return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// INVERT INT
void invertInt() {
  int intBuffer;
  intBuffer = minInValue[currentPatch];
  minInValue[currentPatch] = maxInValue[currentPatch];
  maxInValue[currentPatch] = intBuffer;
}

//INVERT FLOAT
void invertFloat() {
  float floatBuffer;
  floatBuffer = dmpMinInValue[currentPatch];
  dmpMinInValue[currentPatch] = dmpMaxInValue[currentPatch];
  dmpMaxInValue[currentPatch] = floatBuffer;
}

//====================================================================================
//-                                                                         BMP DRAW =
#define BUFFPIXEL 20

void bmpDraw(char *filename, int x, int y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;
  boolean  first = true;

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    //Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { 
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.pushColors(lcdbuffer, lcdidx, first);
                lcdidx = 0;
                first  = false;
              }
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = tft.color565(r,g,b);
          } // end pixel
        } // end scanline
        // Write any remaining data to LCD
        if(lcdidx > 0) {
          tft.pushColors(lcdbuffer, lcdidx, first);
        } 
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
//====================================================================================
//-                                                              void deleteSDFile =
void deleteSDFile() {
  switch (fileId) {
       case 0:
         SD.remove("00.txt");
         break;
        case 1:
          SD.remove("01.txt");
          break;
        case 2:
          SD.remove("02.txt");
          break;
        case 3:
          SD.remove("03.txt");
          break;
        case 4:
          SD.remove("04.txt");
          break;
        case 5:
          SD.remove("05.txt");
          break;
        case 6:
          SD.remove("06.txt");
          break;
        case 7:
          SD.remove("07.txt");
          break;
        case 8:
          SD.remove("08.txt");
          break;
        case 9:
          SD.remove("09.txt");
          break;
        case 10:
          SD.remove("10.txt");
          break;
        case 11:
          SD.remove("11.txt");
          break;
        case 12:
          SD.remove("12.txt");
          break;
        case 13:
          SD.remove("13.txt");
          break;
        case 14:
          SD.remove("14.txt");
          break;
        case 15:
          SD.remove("15.txt");
          break;
        case 16:
          SD.remove("16.txt");
          break;
        case 17:
          SD.remove("17.txt");
          break;
        case 18:
          SD.remove("18.txt");
          break;
        case 19:
          SD.remove("19.txt");
          break;
        case 20:
          SD.remove("20.txt");
          break;
        case 21:
          SD.remove("21.txt");
          break;
        case 22:
          SD.remove("22.txt");
          break;
        case 23:
          SD.remove("23.txt");
          break;
        case 24:
          SD.remove("24.txt");
          break;
        case 25:
          SD.remove("25.txt");
          break;
        case 26:
          SD.remove("26.txt");
          break;
        case 27:
          SD.remove("27.txt");
          break;
        case 28:
          SD.remove("28.txt");
          break;
        case 29:
          SD.remove("29.txt");
          break;
        case 30:
          SD.remove("30.txt");
          break;
        case 31:
          SD.remove("31.txt");
          break;
        case 32:
          SD.remove("32.txt");
          break;
        case 33:
          SD.remove("33.txt");
          break;
        case 34:
          SD.remove("34.txt");
          break;
        case 35:
          SD.remove("35.txt");
          break;
        case 36:
          SD.remove("36.txt");
          break;
        case 37:
          SD.remove("37.txt");
          break;
        case 38:
          SD.remove("38.txt");
          break;
        case 39:
          SD.remove("39.txt");
          break;
        case 40:
          SD.remove("40.txt");
          break;
        case 41:
          SD.remove("41.txt");
          break;
        case 42:
          SD.remove("42.txt");
          break;
        case 43:
          SD.remove("43.txt");
          break;
        case 44:
          SD.remove("44.txt");
          break;
        case 45:
          SD.remove("45.txt");
          break;
        case 46:
          SD.remove("46.txt");
          break;
        case 47:
          SD.remove("47.txt");
          break;
        case 48:
          SD.remove("48.txt");
          break;
        case 49:
          SD.remove("49.txt");
          break;
        case 50:
          SD.remove("50.txt");
          break;
        case 101:
          SD.remove("S.txt");
          break;
  }
}

//====================================================================================
//-                                                               void writeSDPreset =
void writeSDPreset() {
  switch (fileId) {
       case 0:
         myFile = SD.open("00.txt", FILE_WRITE);
         break;
       case 1:
         myFile = SD.open("01.txt", FILE_WRITE);
         break;
       case 2:
         myFile = SD.open("02.txt", FILE_WRITE);
         break;
       case 3:
         myFile = SD.open("03.txt", FILE_WRITE);
         break;
       case 4:
         myFile = SD.open("04.txt", FILE_WRITE);
         break;
       case 5:
         myFile = SD.open("05.txt", FILE_WRITE);
         break;
       case 6:
         myFile = SD.open("06.txt", FILE_WRITE);
         break;
       case 7:
         myFile = SD.open("07.txt", FILE_WRITE);
         break;
       case 8:
         myFile = SD.open("08.txt", FILE_WRITE);
         break;
       case 9:
         myFile = SD.open("09.txt", FILE_WRITE);
         break;
       case 10:
         myFile = SD.open("10.txt", FILE_WRITE);
         break;
       case 11:
         myFile = SD.open("11.txt", FILE_WRITE);
         break;
       case 12:
         myFile = SD.open("12.txt", FILE_WRITE);
         break;
       case 13:
         myFile = SD.open("13.txt", FILE_WRITE);
         break;
       case 14:
         myFile = SD.open("14.txt", FILE_WRITE);
         break;
       case 15:
         myFile = SD.open("15.txt", FILE_WRITE);
         break;
       case 16:
         myFile = SD.open("16.txt", FILE_WRITE);
         break;
       case 17:
         myFile = SD.open("17.txt", FILE_WRITE);
         break;
       case 18:
         myFile = SD.open("18.txt", FILE_WRITE);
         break;
       case 19:
         myFile = SD.open("19.txt", FILE_WRITE);
         break;
       case 20:
         myFile = SD.open("20.txt", FILE_WRITE);
         break;
       case 21:
         myFile = SD.open("21.txt", FILE_WRITE);
         break;
       case 22:
         myFile = SD.open("22.txt", FILE_WRITE);
         break;
       case 23:
         myFile = SD.open("23.txt", FILE_WRITE);
         break;
       case 24:
         myFile = SD.open("24.txt", FILE_WRITE);
         break;
       case 25:
         myFile = SD.open("25.txt", FILE_WRITE);
         break;
       case 26:
         myFile = SD.open("26.txt", FILE_WRITE);
         break;
       case 27:
         myFile = SD.open("27.txt", FILE_WRITE);
         break;
       case 28:
         myFile = SD.open("28.txt", FILE_WRITE);
         break;
       case 29:
         myFile = SD.open("29.txt", FILE_WRITE);
         break;
       case 30:
         myFile = SD.open("30.txt", FILE_WRITE);
         break;
       case 31:
         myFile = SD.open("31.txt", FILE_WRITE);
         break;
       case 32:
         myFile = SD.open("32.txt", FILE_WRITE);
         break;
       case 33:
         myFile = SD.open("33.txt", FILE_WRITE);
         break;
       case 34:
         myFile = SD.open("34.txt", FILE_WRITE);
         break;
       case 35:
         myFile = SD.open("35.txt", FILE_WRITE);
         break;
       case 36:
         myFile = SD.open("36.txt", FILE_WRITE);
         break;
       case 37:
         myFile = SD.open("37.txt", FILE_WRITE);
         break;
       case 38:
         myFile = SD.open("38.txt", FILE_WRITE);
         break;
       case 39:
         myFile = SD.open("39.txt", FILE_WRITE);
         break;
       case 40:
         myFile = SD.open("40.txt", FILE_WRITE);
         break;
       case 41:
         myFile = SD.open("41.txt", FILE_WRITE);
         break;
       case 42:
         myFile = SD.open("42.txt", FILE_WRITE);
         break;
       case 43:
         myFile = SD.open("43.txt", FILE_WRITE);
         break;
       case 44:
         myFile = SD.open("44.txt", FILE_WRITE);
         break;
       case 45:
         myFile = SD.open("45.txt", FILE_WRITE);
         break;
       case 46:
         myFile = SD.open("46.txt", FILE_WRITE);
         break;
       case 47:
         myFile = SD.open("47.txt", FILE_WRITE);
         break;
       case 48:
         myFile = SD.open("48.txt", FILE_WRITE);
         break;
       case 49:
         myFile = SD.open("49.txt", FILE_WRITE);
         break;
       case 50:
         myFile = SD.open("50.txt", FILE_WRITE);
         break;
  }
  
  if (myFile) {
    myFile.print("Y;");
    myFile.println(pNameB);
    myFile.print("A0;");
    myFile.println(patchNames[0]);
    myFile.print("A1;");
    myFile.println(patchNames[1]);
    myFile.print("A2;");
    myFile.println(patchNames[2]);
    myFile.print("A3;");
    myFile.println(patchNames[3]);
    myFile.print("A4;");
    myFile.println(patchNames[4]);
    myFile.print("A5;");
    myFile.println(patchNames[5]);
    myFile.print("A6;");
    myFile.println(patchNames[6]);
    myFile.print("A7;");
    myFile.println(patchNames[7]);
    myFile.print("A8;");
    myFile.println(patchNames[8]);
    myFile.print("A9;");
    myFile.println(patchNames[9]);
    myFile.print("B0;");
    myFile.println(sensorTypeIdentifier[0]);
    myFile.print("B1;");
    myFile.println(sensorTypeIdentifier[1]);
    myFile.print("B2;");
    myFile.println(sensorTypeIdentifier[2]);
    myFile.print("B3;");
    myFile.println(sensorTypeIdentifier[3]);
    myFile.print("B4;");
    myFile.println(sensorTypeIdentifier[4]);
    myFile.print("B5;");
    myFile.println(sensorTypeIdentifier[5]);
    myFile.print("B6;");
    myFile.println(sensorTypeIdentifier[6]);
    myFile.print("B7;");
    myFile.println(sensorTypeIdentifier[7]);
    myFile.print("B8;");
    myFile.println(sensorTypeIdentifier[8]);
    myFile.print("B9;");
    myFile.println(sensorTypeIdentifier[9]);
    myFile.print("C0;");
    myFile.println(isDmp[0]);
    myFile.print("C1;");
    myFile.println(isDmp[1]);
    myFile.print("C2;");
    myFile.println(isDmp[2]);
    myFile.print("C3;");
    myFile.println(isDmp[3]);
    myFile.print("C4;");
    myFile.println(isDmp[4]);
    myFile.print("C5;");
    myFile.println(isDmp[5]);
    myFile.print("C6;");
    myFile.println(isDmp[6]);
    myFile.print("C7;");
    myFile.println(isDmp[7]);
    myFile.print("C8;");
    myFile.println(isDmp[8]);
    myFile.print("C9;");
    myFile.println(isDmp[9]);
    myFile.print("D0;");
    myFile.println(isMidiOut1[0]);
    myFile.print("D1;");
    myFile.println(isMidiOut1[1]);
    myFile.print("D2;");
    myFile.println(isMidiOut1[2]);
    myFile.print("D3;");
    myFile.println(isMidiOut1[3]);
    myFile.print("D4;");
    myFile.println(isMidiOut1[4]);
    myFile.print("D5;");
    myFile.println(isMidiOut1[5]);
    myFile.print("D6;");
    myFile.println(isMidiOut1[6]);
    myFile.print("D7;");
    myFile.println(isMidiOut1[7]);
    myFile.print("D8;");
    myFile.println(isMidiOut1[8]);
    myFile.print("D9;");
    myFile.println(isMidiOut1[9]);
    myFile.print("E0;");
    myFile.println(isMidiOut2[0]);
    myFile.print("E1;");
    myFile.println(isMidiOut2[1]);
    myFile.print("E2;");
    myFile.println(isMidiOut2[2]);
    myFile.print("E3;");
    myFile.println(isMidiOut2[3]);
    myFile.print("E4;");
    myFile.println(isMidiOut2[4]);
    myFile.print("E5;");
    myFile.println(isMidiOut2[5]);
    myFile.print("E6;");
    myFile.println(isMidiOut2[6]);
    myFile.print("E7;");
    myFile.println(isMidiOut2[7]);
    myFile.print("E8;");
    myFile.println(isMidiOut2[8]);
    myFile.print("E9;");
    myFile.println(isMidiOut2[9]);
    myFile.print("F0;");
    myFile.println(isMidiOut3[0]);
    myFile.print("F1;");
    myFile.println(isMidiOut3[1]);
    myFile.print("F2;");
    myFile.println(isMidiOut3[2]);
    myFile.print("F3;");
    myFile.println(isMidiOut3[3]);
    myFile.print("F4;");
    myFile.println(isMidiOut3[4]);
    myFile.print("F5;");
    myFile.println(isMidiOut3[5]);
    myFile.print("F6;");
    myFile.println(isMidiOut3[6]);
    myFile.print("F7;");
    myFile.println(isMidiOut3[7]);
    myFile.print("F8;");
    myFile.println(isMidiOut3[8]);
    myFile.print("F9;");
    myFile.println(isMidiOut3[9]);
    myFile.print("G0;");
    myFile.println(commandIdentifier[0]);
    myFile.print("G1;");
    myFile.println(commandIdentifier[1]);
    myFile.print("G2;");
    myFile.println(commandIdentifier[2]);
    myFile.print("G3;");
    myFile.println(commandIdentifier[3]);
    myFile.print("G4;");
    myFile.println(commandIdentifier[4]);
    myFile.print("G5;");
    myFile.println(commandIdentifier[5]);
    myFile.print("G6;");
    myFile.println(commandIdentifier[6]);
    myFile.print("G7;");
    myFile.println(commandIdentifier[7]);
    myFile.print("G8;");
    myFile.println(commandIdentifier[8]);
    myFile.print("G9;");
    myFile.println(commandIdentifier[9]);
    myFile.print("H0;");
    myFile.println(midiChan[0]);
    myFile.print("H1;");
    myFile.println(midiChan[1]);
    myFile.print("H2;");
    myFile.println(midiChan[2]);
    myFile.print("H3;");
    myFile.println(midiChan[3]);
    myFile.print("H4;");
    myFile.println(midiChan[4]);
    myFile.print("H5;");
    myFile.println(midiChan[5]);
    myFile.print("H6;");
    myFile.println(midiChan[6]);
    myFile.print("H7;");
    myFile.println(midiChan[7]);
    myFile.print("H8;");
    myFile.println(midiChan[8]);
    myFile.print("H9;");
    myFile.println(midiChan[9]);
    myFile.print("I0;");
    myFile.println(firstDataByte[0]);
    myFile.print("I1;");
    myFile.println(firstDataByte[1]);
    myFile.print("I2;");
    myFile.println(firstDataByte[2]);
    myFile.print("I3;");
    myFile.println(firstDataByte[3]);
    myFile.print("I4;");
    myFile.println(firstDataByte[4]);
    myFile.print("I5;");
    myFile.println(firstDataByte[5]);
    myFile.print("I6;");
    myFile.println(firstDataByte[6]);
    myFile.print("I7;");
    myFile.println(firstDataByte[7]);
    myFile.print("I8;");
    myFile.println(firstDataByte[8]);
    myFile.print("I9;");
    myFile.println(firstDataByte[9]);
    myFile.print("J0;");
    myFile.println(firstDataByteB[0]);
    myFile.print("J1;");
    myFile.println(firstDataByteB[1]);
    myFile.print("J2;");
    myFile.println(firstDataByteB[2]);
    myFile.print("J3;");
    myFile.println(firstDataByteB[3]);
    myFile.print("J4;");
    myFile.println(firstDataByteB[4]);
    myFile.print("J5;");
    myFile.println(firstDataByteB[5]);
    myFile.print("J6;");
    myFile.println(firstDataByteB[6]);
    myFile.print("J7;");
    myFile.println(firstDataByteB[7]);
    myFile.print("J8;");
    myFile.println(firstDataByteB[8]);
    myFile.print("J9;");
    myFile.println(firstDataByteB[9]);
    myFile.print("K0;");
    myFile.println(dmpMinInValue[0]);
    myFile.print("K1;");
    myFile.println(dmpMinInValue[1]);
    myFile.print("K2;");
    myFile.println(dmpMinInValue[2]);
    myFile.print("K3;");
    myFile.println(dmpMinInValue[3]);
    myFile.print("K4;");
    myFile.println(dmpMinInValue[4]);
    myFile.print("K5;");
    myFile.println(dmpMinInValue[5]);
    myFile.print("K6;");
    myFile.println(dmpMinInValue[6]);
    myFile.print("K7;");
    myFile.println(dmpMinInValue[7]);
    myFile.print("K8;");
    myFile.println(dmpMinInValue[8]);
    myFile.print("K9;");
    myFile.println(dmpMinInValue[9]);
    myFile.print("L0;");
    myFile.println(dmpMaxInValue[0]);
    myFile.print("L1;");
    myFile.println(dmpMaxInValue[1]);
    myFile.print("L2;");
    myFile.println(dmpMaxInValue[2]);
    myFile.print("L3;");
    myFile.println(dmpMaxInValue[3]);
    myFile.print("L4;");
    myFile.println(dmpMaxInValue[4]);
    myFile.print("L5;");
    myFile.println(dmpMaxInValue[5]);
    myFile.print("L6;");
    myFile.println(dmpMaxInValue[6]);
    myFile.print("L7;");
    myFile.println(dmpMaxInValue[7]);
    myFile.print("L8;");
    myFile.println(dmpMaxInValue[8]);
    myFile.print("L9;");
    myFile.println(dmpMaxInValue[9]);
    myFile.print("M0;");
    myFile.println(minInValue[0]);
    myFile.print("M1;");
    myFile.println(minInValue[1]);
    myFile.print("M2;");
    myFile.println(minInValue[2]);
    myFile.print("M3;");
    myFile.println(minInValue[3]);
    myFile.print("M4;");
    myFile.println(minInValue[4]);
    myFile.print("M5;");
    myFile.println(minInValue[5]);
    myFile.print("M6;");
    myFile.println(minInValue[6]);
    myFile.print("M7");
    myFile.println(minInValue[7]);
    myFile.print("M8;");
    myFile.println(minInValue[8]);
    myFile.print("M9;");
    myFile.println(minInValue[9]);
    myFile.print("N0;");
    myFile.println(maxInValue[0]);
    myFile.print("N1;");
    myFile.println(maxInValue[1]);
    myFile.print("N2;");
    myFile.println(maxInValue[2]);
    myFile.print("N3;");
    myFile.println(maxInValue[3]);
    myFile.print("N4;");
    myFile.println(maxInValue[4]);
    myFile.print("N5;");
    myFile.println(maxInValue[5]);
    myFile.print("N6;");
    myFile.println(maxInValue[6]);
    myFile.print("N7;");
    myFile.println(maxInValue[7]);
    myFile.print("N8;");
    myFile.println(maxInValue[8]);
    myFile.print("N9;");
    myFile.println(maxInValue[9]);
    myFile.print("O0;");
    myFile.println(minOutValue[0]);
    myFile.print("O1;");
    myFile.println(minOutValue[1]);
    myFile.print("O2;");
    myFile.println(minOutValue[2]);
    myFile.print("O3;");
    myFile.println(minOutValue[3]);
    myFile.print("O4;");
    myFile.println(minOutValue[4]);
    myFile.print("O5;");
    myFile.println(minOutValue[5]);
    myFile.print("O6;");
    myFile.println(minOutValue[6]);
    myFile.print("O7;");
    myFile.println(minOutValue[7]);
    myFile.print("O8;");
    myFile.println(minOutValue[8]);
    myFile.print("O9;");
    myFile.println(minOutValue[9]);
    myFile.print("P0;");
    myFile.println(maxOutValue[0]);
    myFile.print("P1;");
    myFile.println(maxOutValue[1]);
    myFile.print("P2;");
    myFile.println(maxOutValue[2]);
    myFile.print("P3;");
    myFile.println(maxOutValue[3]);
    myFile.print("P4;");
    myFile.println(maxOutValue[4]);
    myFile.print("P5;");
    myFile.println(maxOutValue[5]);
    myFile.print("P6;");
    myFile.println(maxOutValue[6]);
    myFile.print("P7;");
    myFile.println(maxOutValue[7]);
    myFile.print("P8;");
    myFile.println(maxOutValue[8]);
    myFile.print("P9;");
    myFile.println(maxOutValue[9]);
    myFile.print("Q0;");
    myFile.println(maxOutLimit[0]);
    myFile.print("Q1;");
    myFile.println(maxOutLimit[1]);
    myFile.print("Q2;");
    myFile.println(maxOutLimit[2]);
    myFile.print("Q3;");
    myFile.println(maxOutLimit[3]);
    myFile.print("Q4;");
    myFile.println(maxOutLimit[4]);
    myFile.print("Q5;");
    myFile.println(maxOutLimit[5]);
    myFile.print("Q6;");
    myFile.println(maxOutLimit[6]);
    myFile.print("Q7;");
    myFile.println(maxOutLimit[7]);
    myFile.print("Q8;");
    myFile.println(maxOutLimit[8]);
    myFile.print("Q9;");
    myFile.println(maxOutLimit[9]);
    myFile.print("R0;");
    myFile.println(mute[0]);
    myFile.print("R1;");
    myFile.println(mute[1]);
    myFile.print("R2;");
    myFile.println(mute[2]);
    myFile.print("R3;");
    myFile.println(mute[3]);
    myFile.print("R4;");
    myFile.println(mute[4]);
    myFile.print("R5;");
    myFile.println(mute[5]);
    myFile.print("R6;");
    myFile.println(mute[6]);
    myFile.print("R7;");
    myFile.println(mute[7]);
    myFile.print("R8;");
    myFile.println(mute[8]);
    myFile.print("R9;");
    myFile.println(mute[9]);
    myFile.print("Z;");
    myFile.println(liveMode);
    myFile.close();
  }
}    

//====================================================================================
//-                                                             void writeSDSettings =
void writeSDSettings() {
  myFile = SD.open("S.txt", FILE_WRITE);
  
  if (myFile) {
    myFile.print("A;");
    myFile.println(ax_offset);
    myFile.print("B;");
    myFile.println(ay_offset);
    myFile.print("C;");
    myFile.println(az_offset);
    myFile.print("X;");
    myFile.println(gx_offset);
    myFile.print("Y;");
    myFile.println(gy_offset);
    myFile.print("Z;");
    myFile.println(gz_offset);
    myFile.close();
  }
}    

//====================================================================================
//-                                                                void readSDPreset =
void readSDPreset(){
  char character;
  String settingName;
  String settingValue;
  switch (fileId) {
       case 0:
         myFile = SD.open("00.txt");
         break;
        case 1:
          myFile = SD.open("01.txt");
          break;
        case 2:
          myFile = SD.open("02.txt");
          break;
        case 3:
          myFile = SD.open("03.txt");
          break;
        case 4:
          myFile = SD.open("04.txt");
          break;
        case 5:
          myFile = SD.open("05.txt");
          break;
        case 6:
          myFile = SD.open("06.txt");
          break;
        case 7:
          myFile = SD.open("07.txt");
          break;
        case 8:
          myFile = SD.open("08.txt");
          break;
        case 9:
          myFile = SD.open("09.txt");
          break;
        case 10:
          myFile = SD.open("10.txt");
          break;
        case 11:
          myFile = SD.open("11.txt");
          break;
        case 12:
          myFile = SD.open("12.txt");
          break;
        case 13:
          myFile = SD.open("13.txt");
          break;
        case 14:
          myFile = SD.open("14.txt");
          break;
        case 15:
          myFile = SD.open("15.txt");
          break;
        case 16:
          myFile = SD.open("16.txt");
          break;
        case 17:
          myFile = SD.open("17.txt");
          break;
        case 18:
          myFile = SD.open("18.txt");
          break;
        case 19:
          myFile = SD.open("19.txt");
          break;
        case 20:
          myFile = SD.open("20.txt");
          break;
        case 21:
          myFile = SD.open("21.txt");
          break;
        case 22:
          myFile = SD.open("22.txt");
          break;
        case 23:
          myFile = SD.open("23.txt");
          break;
        case 24:
          myFile = SD.open("24.txt");
          break;
        case 25:
          myFile = SD.open("25.txt");
          break;
        case 26:
          myFile = SD.open("26.txt");
          break;
        case 27:
          myFile = SD.open("27.txt");
          break;
        case 28:
          myFile = SD.open("28.txt");
          break;
        case 29:
          myFile = SD.open("29.txt");
          break;
        case 30:
          myFile = SD.open("30.txt");
          break;
        case 31:
          myFile = SD.open("31.txt");
          break;
        case 32:
          myFile = SD.open("32.txt");
          break;
        case 33:
          myFile = SD.open("33.txt");
          break;
        case 34:
          myFile = SD.open("34.txt");
          break;
        case 35:
          myFile = SD.open("35.txt");
          break;
        case 36:
          myFile = SD.open("36.txt");
          break;
        case 37:
          myFile = SD.open("37.txt");
          break;
        case 38:
          myFile = SD.open("38.txt");
          break;
        case 39:
          myFile = SD.open("39.txt");
          break;
        case 40:
          myFile = SD.open("40.txt");
          break;
        case 41:
          myFile = SD.open("41.txt");
          break;
        case 42:
          myFile = SD.open("42.txt");
          break;
        case 43:
          myFile = SD.open("43.txt");
          break;
        case 44:
          myFile = SD.open("44.txt");
          break;
        case 45:
          myFile = SD.open("45.txt");
          break;
        case 46:
          myFile = SD.open("46.txt");
          break;
        case 47:
          myFile = SD.open("47.txt");
          break;
        case 48:
          myFile = SD.open("48.txt");
          break;
        case 49:
          myFile = SD.open("49.txt");
          break;
        case 50:
          myFile = SD.open("50.txt");
          break;
  }
  if (myFile) {
    while (myFile.available()) {
      character = myFile.read();
      while(character != ';'){
        settingName = settingName + character;
        character = myFile.read();
      }
      character = myFile.read();
      while(character != '\n'){
        settingValue = settingValue + character;
        character = myFile.read();
        if(character == '\n'){
          // Apply the value to the parameter
          applyPreset(settingName,settingValue);
          // Reset Strings
          settingName = "";
          settingValue = "";
        }
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn’t open, print an error:
    //Serial.println("error opening settings.txt");
  }
}  

//------------------------------------------------------------------------------------
//                                                                  void applyPreset -
void applyPreset(String settingName, String settingValue) {
  
  if(settingName == "Y") {
    settingValue.toCharArray(pNameB, 8);
  }
  else if(settingName == "A0") {
    settingValue.toCharArray(patchNames[0], 8);
  }
  else if(settingName == "A1") {
    settingValue.toCharArray(patchNames[1], 8);
  }
  else if(settingName == "A2") {
    settingValue.toCharArray(patchNames[2], 8);
  }
  else if(settingName == "A3") {
    settingValue.toCharArray(patchNames[3], 8);
  }
  else if(settingName == "A4") {
    settingValue.toCharArray(patchNames[4], 8);
  }
  else if(settingName == "A5") {
    settingValue.toCharArray(patchNames[5], 8);
  }
  else if(settingName == "A6") {
    settingValue.toCharArray(patchNames[6], 8);
  }
  else if(settingName == "A7") {
    settingValue.toCharArray(patchNames[7], 8);
  }
  else if(settingName == "A8") {
    settingValue.toCharArray(patchNames[8], 8);
  }
  else if(settingName == "A9") {
    settingValue.toCharArray(patchNames[9], 8);
  }
  else if(settingName == "B0") {
    sensorTypeIdentifier[0]=settingValue.toInt();
  }
  else if(settingName == "B1") {
    sensorTypeIdentifier[1]=settingValue.toInt();
  }
  else if(settingName == "B2") {
    sensorTypeIdentifier[2]=settingValue.toInt();
  }
  else if(settingName == "B3") {
    sensorTypeIdentifier[3]=settingValue.toInt();
  }
  else if(settingName == "B4") {
    sensorTypeIdentifier[4]=settingValue.toInt();
  }
  else if(settingName == "B5") {
    sensorTypeIdentifier[5]=settingValue.toInt();
  }
  else if(settingName == "B6") {
    sensorTypeIdentifier[6]=settingValue.toInt();
  }
  else if(settingName == "B7") {
    sensorTypeIdentifier[7]=settingValue.toInt();
  }
  else if(settingName == "B8") {
    sensorTypeIdentifier[8]=settingValue.toInt();
  }
  else if(settingName == "B9") {
    sensorTypeIdentifier[9]=settingValue.toInt();
  }
  else if(settingName == "C0") {
    isDmp[0]=toBoolean(settingValue);
  }
  else if(settingName == "C1") {
    isDmp[1]=toBoolean(settingValue);
  }
  else if(settingName == "C2") {
    isDmp[2]=toBoolean(settingValue);
  }
  else if(settingName == "C3") {
    isDmp[3]=toBoolean(settingValue);
  }
  else if(settingName == "C4") {
    isDmp[4]=toBoolean(settingValue);
  }
  else if(settingName == "C5") {
    isDmp[5]=toBoolean(settingValue);
  }
  else if(settingName == "C6") {
    isDmp[6]=toBoolean(settingValue);
  }
  else if(settingName == "C7") {
    isDmp[7]=toBoolean(settingValue);
  }
  else if(settingName == "C8") {
    isDmp[8]=toBoolean(settingValue);
  }
  else if(settingName == "C9") {
    isDmp[9]=toBoolean(settingValue);
  }
  else if(settingName == "D0") {
    isMidiOut1[0]=toBoolean(settingValue);
  }
  else if(settingName == "D1") {
    isMidiOut1[1]=toBoolean(settingValue);
  }
  else if(settingName == "D2") {
    isMidiOut1[2]=toBoolean(settingValue);
  }
  else if(settingName == "D3") {
    isMidiOut1[3]=toBoolean(settingValue);
  }
  else if(settingName == "D4") {
    isMidiOut1[4]=toBoolean(settingValue);
  }
  else if(settingName == "D5") {
    isMidiOut1[5]=toBoolean(settingValue);
  }
  else if(settingName == "D6") {
    isMidiOut1[6]=toBoolean(settingValue);
  }
  else if(settingName == "D7") {
    isMidiOut1[7]=toBoolean(settingValue);
  }
  else if(settingName == "D8") {
    isMidiOut1[8]=toBoolean(settingValue);
  }
  else if(settingName == "D9") {
    isMidiOut1[9]=toBoolean(settingValue);
  }
  else if(settingName == "E0") {
    isMidiOut2[0]=toBoolean(settingValue);
  }
  else if(settingName == "E1") {
    isMidiOut2[1]=toBoolean(settingValue);
  }
  else if(settingName == "E2") {
    isMidiOut2[2]=toBoolean(settingValue);
  }
  else if(settingName == "E3") {
    isMidiOut2[3]=toBoolean(settingValue);
  }
  else if(settingName == "E4") {
    isMidiOut2[4]=toBoolean(settingValue);
  }
  else if(settingName == "E5") {
    isMidiOut2[5]=toBoolean(settingValue);
  }
  else if(settingName == "E6") {
    isMidiOut2[6]=toBoolean(settingValue);
  }
  else if(settingName == "E7") {
    isMidiOut2[7]=toBoolean(settingValue);
  }
  else if(settingName == "E8") {
    isMidiOut2[8]=toBoolean(settingValue);
  }
  else if(settingName == "E9") {
    isMidiOut2[9]=toBoolean(settingValue);
  }
  else if(settingName == "F0") {
    isMidiOut3[0]=toBoolean(settingValue);
  }
  else if(settingName == "F1") {
    isMidiOut3[1]=toBoolean(settingValue);
  }
  else if(settingName == "F2") {
    isMidiOut3[2]=toBoolean(settingValue);
  }
  else if(settingName == "F3") {
    isMidiOut3[3]=toBoolean(settingValue);
  }
  else if(settingName == "F4") {
    isMidiOut3[4]=toBoolean(settingValue);
  }
  else if(settingName == "F5") {
    isMidiOut3[5]=toBoolean(settingValue);
  }
  else if(settingName == "F6") {
    isMidiOut3[6]=toBoolean(settingValue);
  }
  else if(settingName == "F7") {
    isMidiOut3[7]=toBoolean(settingValue);
  }
  else if(settingName == "F8") {
    isMidiOut3[8]=toBoolean(settingValue);
  }
  else if(settingName == "F9") {
    isMidiOut3[9]=toBoolean(settingValue);
  }
  else if(settingName == "G0") {
    commandIdentifier[0]=settingValue.toInt();
  }
  else if(settingName == "G1") {
    commandIdentifier[1]=settingValue.toInt();
  }
  else if(settingName == "G2") {
    commandIdentifier[2]=settingValue.toInt();
  }
  else if(settingName == "G3") {
    commandIdentifier[3]=settingValue.toInt();
  }
  else if(settingName == "G4") {
    commandIdentifier[4]=settingValue.toInt();
  }
  else if(settingName == "G5") {
    commandIdentifier[5]=settingValue.toInt();
  }
  else if(settingName == "G6") {
    commandIdentifier[6]=settingValue.toInt();
  }
  else if(settingName == "G7") {
    commandIdentifier[7]=settingValue.toInt();
  }
  else if(settingName == "G8") {
    commandIdentifier[8]=settingValue.toInt();
  }
  else if(settingName == "G9") {
    commandIdentifier[9]=settingValue.toInt();
  }
  else if(settingName == "H0") {
    midiChan[0]=settingValue.toInt();
  }
  else if(settingName == "H1") {
    midiChan[1]=settingValue.toInt();
  }
  else if(settingName == "H2") {
    midiChan[2]=settingValue.toInt();
  }
  else if(settingName == "H3") {
    midiChan[3]=settingValue.toInt();
  }
  else if(settingName == "H4") {
    midiChan[4]=settingValue.toInt();
  }
  else if(settingName == "H5") {
    midiChan[5]=settingValue.toInt();
  }
  else if(settingName == "H6") {
    midiChan[6]=settingValue.toInt();
  }
  else if(settingName == "H7") {
    midiChan[7]=settingValue.toInt();
  }
  else if(settingName == "H8") {
    midiChan[8]=settingValue.toInt();
  }
  else if(settingName == "H9") {
    midiChan[9]=settingValue.toInt();
  }
  else if(settingName == "I0") {
    firstDataByte[0]=settingValue.toInt();
  }
  else if(settingName == "I1") {
    firstDataByte[1]=settingValue.toInt();
  }
  else if(settingName == "I2") {
    firstDataByte[2]=settingValue.toInt();
  }
  else if(settingName == "I3") {
    firstDataByte[3]=settingValue.toInt();
  }
  else if(settingName == "I4") {
    firstDataByte[4]=settingValue.toInt();
  }
  else if(settingName == "I5") {
    firstDataByte[5]=settingValue.toInt();
  }
  else if(settingName == "I6") {
    firstDataByte[6]=settingValue.toInt();
  }
  else if(settingName == "I7") {
    firstDataByte[7]=settingValue.toInt();
  }
  else if(settingName == "I8") {
    firstDataByte[8]=settingValue.toInt();
  }
  else if(settingName == "I9") {
    firstDataByte[9]=settingValue.toInt();
  }
  else if(settingName == "J0") {
    firstDataByteB[0]=settingValue.toInt();
  }
  else if(settingName == "J1") {
    firstDataByteB[1]=settingValue.toInt();
  }
  else if(settingName == "J2") {
    firstDataByteB[2]=settingValue.toInt();
  }
  else if(settingName == "J3") {
    firstDataByteB[3]=settingValue.toInt();
  }
  else if(settingName == "J4") {
    firstDataByteB[4]=settingValue.toInt();
  }
  else if(settingName == "J5") {
    firstDataByteB[5]=settingValue.toInt();
  }
  else if(settingName == "J6") {
    firstDataByteB[6]=settingValue.toInt();
  }
  else if(settingName == "J7") {
    firstDataByteB[7]=settingValue.toInt();
  }
  else if(settingName == "J8") {
    firstDataByteB[8]=settingValue.toInt();
  }
  else if(settingName == "J9") {
    firstDataByteB[9]=settingValue.toInt();
  }
  else if(settingName == "K0") {
    dmpMinInValue[0]=toFloat(settingValue);
  }
  else if(settingName == "K1") {
    dmpMinInValue[1]=toFloat(settingValue);
  }
  else if(settingName == "K2") {
    dmpMinInValue[2]=toFloat(settingValue);
  }
  else if(settingName == "K3") {
    dmpMinInValue[3]=toFloat(settingValue);
  }
  else if(settingName == "K4") {
    dmpMinInValue[4]=toFloat(settingValue);
  }
  else if(settingName == "K5") {
    dmpMinInValue[5]=toFloat(settingValue);
  }
  else if(settingName == "K6") {
    dmpMinInValue[6]=toFloat(settingValue);
  }
  else if(settingName == "K7") {
    dmpMinInValue[7]=toFloat(settingValue);
  }
  else if(settingName == "K8") {
    dmpMinInValue[8]=toFloat(settingValue);
  }
  else if(settingName == "K9") {
    dmpMinInValue[9]=toFloat(settingValue);
  }
  else if(settingName == "L0") {
    dmpMaxInValue[0]=toFloat(settingValue);
  }
  else if(settingName == "L1") {
    dmpMaxInValue[1]=toFloat(settingValue);
  }
  else if(settingName == "L2") {
    dmpMaxInValue[2]=toFloat(settingValue);
  }
  else if(settingName == "L3") {
    dmpMaxInValue[3]=toFloat(settingValue);
  }
  else if(settingName == "L4") {
    dmpMaxInValue[4]=toFloat(settingValue);
  }
  else if(settingName == "L5") {
    dmpMaxInValue[5]=toFloat(settingValue);
  }
  else if(settingName == "L6") {
    dmpMaxInValue[6]=toFloat(settingValue);
  }
  else if(settingName == "L7") {
    dmpMaxInValue[7]=toFloat(settingValue);
  }
  else if(settingName == "L8") {
    dmpMaxInValue[8]=toFloat(settingValue);
  }
  else if(settingName == "L9") {
    dmpMaxInValue[9]=toFloat(settingValue);
  }
  else if(settingName == "M0") {
    minInValue[0]=settingValue.toInt();
  }
  else if(settingName == "M1") {
    minInValue[1]=settingValue.toInt();
  }
  else if(settingName == "M2") {
    minInValue[2]=settingValue.toInt();
  }
  else if(settingName == "M3") {
    minInValue[3]=settingValue.toInt();
  }
  else if(settingName == "M4") {
    minInValue[4]=settingValue.toInt();
  }
  else if(settingName == "M5") {
    minInValue[5]=settingValue.toInt();
  }
  else if(settingName == "M6") {
    minInValue[6]=settingValue.toInt();
  }
  else if(settingName == "M7") {
    minInValue[7]=settingValue.toInt();
  }
  else if(settingName == "M8") {
    minInValue[8]=settingValue.toInt();
  }
  else if(settingName == "M9") {
    minInValue[9]=settingValue.toInt();
  }
  else if(settingName == "N0") {
    maxInValue[0]=settingValue.toInt();
  }
  else if(settingName == "N1") {
    maxInValue[1]=settingValue.toInt();
  }
  else if(settingName == "N2") {
    maxInValue[2]=settingValue.toInt();
  }
  else if(settingName == "N3") {
    maxInValue[3]=settingValue.toInt();
  }
  else if(settingName == "N4") {
    maxInValue[4]=settingValue.toInt();
  }
  else if(settingName == "N5") {
    maxInValue[5]=settingValue.toInt();
  }
  else if(settingName == "N6") {
    maxInValue[6]=settingValue.toInt();
  }
  else if(settingName == "N7") {
    maxInValue[7]=settingValue.toInt();
  }
  else if(settingName == "N8") {
    maxInValue[8]=settingValue.toInt();
  }
  else if(settingName == "N9") {
    maxInValue[9]=settingValue.toInt();
  }
  else if(settingName == "O0") {
    minOutValue[0]=settingValue.toInt();
  }
  else if(settingName == "O1") {
    minOutValue[1]=settingValue.toInt();
  }
  else if(settingName == "O2") {
    minOutValue[2]=settingValue.toInt();
  }
  else if(settingName == "O3") {
    minOutValue[3]=settingValue.toInt();
  }
  else if(settingName == "O4") {
    minOutValue[4]=settingValue.toInt();
  }
  else if(settingName == "O5") {
    minOutValue[5]=settingValue.toInt();
  }
  else if(settingName == "O6") {
    minOutValue[6]=settingValue.toInt();
  }
  else if(settingName == "O7") {
    minOutValue[7]=settingValue.toInt();
  }
  else if(settingName == "O8") {
    minOutValue[8]=settingValue.toInt();
  }
  else if(settingName == "O9") {
    minOutValue[9]=settingValue.toInt();
  }
  else if(settingName == "P0") {
    maxOutValue[0]=settingValue.toInt();
  }
  else if(settingName == "P1") {
    maxOutValue[1]=settingValue.toInt();
  }
  else if(settingName == "P2") {
    maxOutValue[2]=settingValue.toInt();
  }
  else if(settingName == "P3") {
    maxOutValue[3]=settingValue.toInt();
  }
  else if(settingName == "P4") {
    maxOutValue[4]=settingValue.toInt();
  }
  else if(settingName == "P5") {
    maxOutValue[5]=settingValue.toInt();
  }
  else if(settingName == "P6") {
    maxOutValue[6]=settingValue.toInt();
  }
  else if(settingName == "P7") {
    maxOutValue[7]=settingValue.toInt();
  }
  else if(settingName == "P8") {
    maxOutValue[8]=settingValue.toInt();
  }
  else if(settingName == "P9") {
    maxOutValue[9]=settingValue.toInt();
  }
  else if(settingName == "Q0") {
    maxOutLimit[0]=settingValue.toInt();
  }
  else if(settingName == "Q1") {
    maxOutLimit[1]=settingValue.toInt();
  }
  else if(settingName == "Q2") {
    maxOutLimit[2]=settingValue.toInt();
  }
  else if(settingName == "Q3") {
    maxOutLimit[3]=settingValue.toInt();
  }
  else if(settingName == "Q4") {
    maxOutLimit[4]=settingValue.toInt();
  }
  else if(settingName == "Q5") {
    maxOutLimit[5]=settingValue.toInt();
  }
  else if(settingName == "Q6") {
    maxOutLimit[6]=settingValue.toInt();
  }
  else if(settingName == "Q7") {
    maxOutLimit[7]=settingValue.toInt();
  }
  else if(settingName == "Q8") {
    maxOutLimit[8]=settingValue.toInt();
  }
  else if(settingName == "Q9") {
    maxOutLimit[9]=settingValue.toInt();
  }
  else if(settingName == "R0") {
    mute[0]=toBoolean(settingValue);
  }
  else if(settingName == "R1") {
    mute[1]=toBoolean(settingValue);
  }
  else if(settingName == "R2") {
    mute[2]=toBoolean(settingValue);
  }
  else if(settingName == "R3") {
    mute[3]=toBoolean(settingValue);
  }
  else if(settingName == "R4") {
    mute[4]=toBoolean(settingValue);
  }
  else if(settingName == "R5") {
    mute[5]=toBoolean(settingValue);
  }
  else if(settingName == "R6") {
    mute[6]=toBoolean(settingValue);
  }
  else if(settingName == "R7") {
    mute[7]=toBoolean(settingValue);
  }
  else if(settingName == "R8") {
    mute[8]=toBoolean(settingValue);
  }
  else if(settingName == "R9") {
    mute[9]=toBoolean(settingValue);
  }
  else if(settingName == "Z") {
    liveMode=settingValue.toInt();
  }
}

//====================================================================================
//                                                                   void readSDName =
void readSDpName(){
  char character;
  String settingName;
  String settingValue;
  switch (fileId) {
       case 0:
         myFile = SD.open("00.txt");
         break;
        case 1:
          myFile = SD.open("01.txt");
          break;
        case 2:
          myFile = SD.open("02.txt");
          break;
        case 3:
          myFile = SD.open("03.txt");
          break;
        case 4:
          myFile = SD.open("04.txt");
          break;
        case 5:
          myFile = SD.open("05.txt");
          break;
        case 6:
          myFile = SD.open("06.txt");
          break;
        case 7:
          myFile = SD.open("07.txt");
          break;
        case 8:
          myFile = SD.open("08.txt");
          break;
        case 9:
          myFile = SD.open("09.txt");
          break;
        case 10:
          myFile = SD.open("10.txt");
          break;
        case 11:
          myFile = SD.open("11.txt");
          break;
        case 12:
          myFile = SD.open("12.txt");
          break;
        case 13:
          myFile = SD.open("13.txt");
          break;
        case 14:
          myFile = SD.open("14.txt");
          break;
        case 15:
          myFile = SD.open("15.txt");
          break;
        case 16:
          myFile = SD.open("16.txt");
          break;
        case 17:
          myFile = SD.open("17.txt");
          break;
        case 18:
          myFile = SD.open("18.txt");
          break;
        case 19:
          myFile = SD.open("19.txt");
          break;
        case 20:
          myFile = SD.open("20.txt");
          break;
        case 21:
          myFile = SD.open("21.txt");
          break;
        case 22:
          myFile = SD.open("22.txt");
          break;
        case 23:
          myFile = SD.open("23.txt");
          break;
        case 24:
          myFile = SD.open("24.txt");
          break;
        case 25:
          myFile = SD.open("25.txt");
          break;
        case 26:
          myFile = SD.open("26.txt");
          break;
        case 27:
          myFile = SD.open("27.txt");
          break;
        case 28:
          myFile = SD.open("28.txt");
          break;
        case 29:
          myFile = SD.open("29.txt");
          break;
        case 30:
          myFile = SD.open("30.txt");
          break;
        case 31:
          myFile = SD.open("31.txt");
          break;
        case 32:
          myFile = SD.open("32.txt");
          break;
        case 33:
          myFile = SD.open("33.txt");
          break;
        case 34:
          myFile = SD.open("34.txt");
          break;
        case 35:
          myFile = SD.open("35.txt");
          break;
        case 36:
          myFile = SD.open("36.txt");
          break;
        case 37:
          myFile = SD.open("37.txt");
          break;
        case 38:
          myFile = SD.open("38.txt");
          break;
        case 39:
          myFile = SD.open("39.txt");
          break;
        case 40:
          myFile = SD.open("40.txt");
          break;
        case 41:
          myFile = SD.open("41.txt");
          break;
        case 42:
          myFile = SD.open("42.txt");
          break;
        case 43:
          myFile = SD.open("43.txt");
          break;
        case 44:
          myFile = SD.open("44.txt");
          break;
        case 45:
          myFile = SD.open("45.txt");
          break;
        case 46:
          myFile = SD.open("46.txt");
          break;
        case 47:
          myFile = SD.open("47.txt");
          break;
        case 48:
          myFile = SD.open("48.txt");
          break;
        case 49:
          myFile = SD.open("49.txt");
          break;
        case 50:
          myFile = SD.open("50.txt");
          break;
  }
  if (myFile) {
    while (myFile.available()) {
      character = myFile.read();
      while(character != ';'){
        settingName = settingName + character;
        character = myFile.read();
      }
      character = myFile.read();
      while(character != '\n'){
        settingValue = settingValue + character;
        character = myFile.read();
        if(character == '\n'){
          // Apply the value to the parameter
          applyPresetName(settingName,settingValue);
          // Reset Strings
          settingName = "";
          settingValue = "";
        }
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn’t open, print an error:
    //Serial.println("error opening settings.txt");
  }
}  

//------------------------------------------------------------------------------------
//                                                              void applyPresetName -
void applyPresetName(String settingName, String settingValue) {
  
  if(settingName == "Y") {
    settingValue.toCharArray(pName, 8);
  }
}

//====================================================================================
//-                                                              void readSDSettings =
void readSDSettings(){
  char character;
  String settingName;
  String settingValue;
  myFile = SD.open("S.txt");
  if (myFile) {
    while (myFile.available()) {
      character = myFile.read();
      while(character != ';'){
        settingName = settingName + character;
        character = myFile.read();
      }
      character = myFile.read();
      while(character != '\n'){
        settingValue = settingValue + character;
        character = myFile.read();
        if(character == '\n'){
          // Apply the value to the parameter
          applySettings(settingName,settingValue);
          // Reset Strings
          settingName = "";
          settingValue = "";
        }
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn’t open, print an error:
    //Serial.println("error opening settings.txt");
  }
}  

//------------------------------------------------------------------------------------
//                                                                void applySettings -
void applySettings(String settingName, String settingValue) {
  
  if(settingName == "C") {
    az_offset=settingValue.toInt();
  }
  else if(settingName == "X") {
    gx_offset=settingValue.toInt();
  }
  else if(settingName == "Y") {
    gy_offset=settingValue.toInt();
  }
  else if(settingName == "Z") {
    gz_offset=settingValue.toInt();
  }
}

//------------------------------------------------------------------------------------
//                                                 variable type conversion routines -

float toFloat(String settingValue){
  char floatbuf[settingValue.length()];
  settingValue.toCharArray(floatbuf, sizeof(floatbuf));
  float f = atof(floatbuf);
  return f;
}

boolean toBoolean(String settingValue) {
  if(settingValue.toInt()==1){
    return true;
  } else {
    return false;
  }
}

//====================================================================================
//=                                                                VOID calibrateMSU =
// This calibration routine is based on code published by Luis Ródenas 
// <luisrodenaslorda@gmail.com>
void calibrateMSU() {
  boolean inSetupMode = true;
  boolean placeSensor = false;
  
  tft.fillScreen(BLACK);
  
  // initialize device
  accelgyro.initialize();
  
  // print instructions
  tft.setTextSize(2);
  tft.setTextColor(LIME);
  tft.setCursor(4, 8);
  tft.print("Place MSU in horizontal position.");
  tft.setCursor(4, 30);
  tft.print("Don't touch it from now on!");
  tft.setCursor(4, 52);
  tft.print("Touch the screen when you are ready.");
  
  // touchscreen loop 1
  while(placeSensor == false) { 
    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      placeSensor = true;
    }
  }
  
  // check DSP connection and print result
  tft.setTextColor(GRAY);
  tft.setCursor(4, 96);
  tft.print(accelgyro.testConnection() ? "MSU connection successful." : "MSU connection failed.");
  delay(500);
  
  //reset offset values
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  
  // read initial sensor data
  if (state==0){
    tft.setCursor(4, 118);
    tft.print("Reading sensors for first time.");
    meansensors();
    state++;
    delay(500);
  }
  
  tft.setCursor(4, 140);
  tft.print("Calculating offsets...");
  
  // execute calibration
  if (state==1) {
    calibration();
    state++;
    delay(1000);
  }
  
  // print results
  if (state==2) {
    meansensors();
    tft.setCursor(4, 162);
    tft.print("FINISHED!");
    tft.setCursor(4, 206);
    tft.print("SRWO:");
    tft.print(mean_az);
    tft.print(", ");
    tft.print(mean_gx);
    tft.print(", ");
    tft.print(mean_gy);
    tft.print(", ");
    tft.print(mean_gz);
    tft.setCursor(4, 228);
    tft.print("AZ:");
    tft.print(az_offset);
    tft.print(", GX:");
    tft.print(gx_offset);
    tft.print(", GY:");
    tft.print(gy_offset);
    tft.print(", GZ:");
    tft.print(gz_offset);
  }
  
  //print instructions
  tft.setTextColor(LIME);
  tft.setCursor(4, 272);
  tft.print("Please touch the screen to save");
  tft.setCursor(4, 294);
  tft.print("calibration data now!");
  
  // touchscreen loop 2
  while(placeSensor == true) { 
    TSPoint p = ts.getPoint();
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      placeSensor = false;
      fileId = 101;
      deleteSDFile(); // delete old settings file 
      writeSDSettings(); // and write calibration data to a new one
    }
  }
  tft.fillScreen(BLACK);
  tft.setCursor(4, 8);
  tft.print("SAVED! MIDI Garden needs to reboot now.");
  tft.setCursor(4, 52);
  tft.print("Please switch it off, wait a few ");
  tft.setCursor(4, 74);
  tft.print("seconds, then switch it on again.");
  while(1);
}

// FUNCTIONS
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ // first 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); // needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    tft.print(".");

    if (abs(mean_ax)<=8) ready++;
    else ax_offset=ax_offset-mean_ax/8;

    if (abs(mean_ay)<=8) ready++;
    else ay_offset=ay_offset-mean_ay/8;

    if (abs(16384-mean_az)<=8) ready++;
    else az_offset=az_offset+(16384-mean_az)/8;

    if (abs(mean_gx)<=1) ready++;
    else gx_offset=gx_offset-mean_gx/(1+1);

    if (abs(mean_gy)<=1) ready++;
    else gy_offset=gy_offset-mean_gy/(1+1);

    if (abs(mean_gz)<=1) ready++;
    else gz_offset=gz_offset-mean_gz/(1+1);

    if (ready==6) break;
  }
}

//====================================================================================
//-                                                                       VOID about =
void about() {
  boolean inSetupMode = true;
  
  tft.fillScreen(BLACK);
  
  tft.setTextSize(2);
  tft.setTextColor(LIME);
  tft.setCursor(4, 8);
  tft.print("MIDI Garden has been developed by");
  tft.setCursor(4, 30);
  tft.setTextColor(GRAY);
  tft.print("Philipp Stute ");
  tft.setTextColor(LIME);
  tft.print("aka. Tom Trialanderror");
  tft.setCursor(4, 52);
  tft.print("of Lime Labs.");
  /*
  tft.setCursor(4, 96);
  tft.print("Thanks dear members of the Facebook");
  tft.setCursor(4, 118);
  tft.print("groups ");
  tft.setTextColor(GRAY);
  tft.print("Synth DIY ");
  tft.setTextColor(LIME);
  tft.print("& ");
  tft.setTextColor(GRAY);
  tft.print("The Hard, the Soft,");
  tft.setCursor(4, 140);
  tft.print("and the Modular ");
  tft.setTextColor(LIME);
  tft.print("for making this project");
  tft.setCursor(4, 162);
  tft.print("possible.");
  */
  tft.setCursor(4, 206);
  tft.print("Thanks ");
  tft.setTextColor(GRAY);
  tft.print("Pete Hartman");
  tft.setTextColor(LIME);
  tft.print(", ");
  tft.setTextColor(GRAY);
  tft.print("Quincas Moreira");
  tft.setTextColor(LIME);
  tft.print(", ");
  tft.setCursor(4, 228);
  tft.setTextColor(GRAY);
  tft.print("HazardsMind ");
  tft.setTextColor(LIME);
  tft.print("& ");
  tft.setTextColor(GRAY);
  tft.print("Steve Smith ");
  tft.setTextColor(LIME);
  tft.print("for the help.");
  
  tft.setCursor(4, 272);
  tft.print("Thanks ");
  tft.setTextColor(GRAY);
  tft.print("Arduino Team");
  tft.setTextColor(LIME);
  tft.print(", ");
  tft.setTextColor(GRAY);
  tft.print("Jeff Rowberg ");
  tft.setTextColor(LIME);
  tft.print("& ");
  tft.setCursor(4, 294);
  tft.setTextColor(GRAY);
  tft.print("Adafruit Industries ");
  tft.setTextColor(LIME);
  tft.print("for the libraries.");
  
  // touchscreen loop
  while(inSetupMode == true) { 
    TSPoint p = ts.getPoint();

    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      inSetupMode = false;
      tft.fillScreen(BLACK);
      loop();
    }
  }
}
