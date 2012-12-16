/*
Copyright 2012 Paul Thomas                

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce.h>

#define P_SD      9
#define LED       8

//-------------------------
// MCP2515 Pin Defines
#define P_CS       10
#define P_SCK      13
#define P_MOSI     11
#define P_MISO     12
#define P_INT      9
#define rxPin      7
#define txPin      6

//-------------------------
//MCP2515
#define CNF3        0x28
#define CNF2        0x29
#define CNF1        0x2A
#define CANINTE     0x2B
#define CANINTF     0x2C
#define RXB0CTRL    0x60
#define RXB1CTRL    0x70
#define RXM0SIDH    0x20
#define RXM0SIDL    0x21
#define RXM0EID8    0x22
#define RXM0EID0    0x23
#define RXM1SIDH    0x24
#define RXM1SIDL    0x25
#define RXM1EID8    0x26
#define RXM1EID0    0x27
#define CANCTRL     0x0F
#define TXB0SIDH    0x31
#define TXB0SIDL    0x32
#define TXB0DLC     0x35
#define TXB0D0      0x36
#define TXB0CTRL    0x30

//-------------------------
//RX Filter Bytes... 
#define RXF0SIDH    0x00
#define RXF1SIDH    0x04
#define RXF2SIDH    0x08
#define RXF3SIDH    0x10
#define RXF4SIDH    0x14
#define RXF5SIDH    0x18

//-------------------------
// MCP2515 defs
#define EXIDE      (1<<3)
#define EXMASK     0x1FFFFFFF
#define MCP_DLC_MASK        0x0F    /* 4 LSBits */
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3 

/* Define Joystick connection */
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   A5

int LED2 = 8;
int LED3 = 7;

SoftwareSerial sLCD =  SoftwareSerial(3, 6); /* Serial LCD is connected on pin 14 (Analog input 0) */
#define COMMAND 0xFE
#define CLEAR   0x01
#define LINE0   0x80
#define LINE1   0xC0

// Bouncers
Bounce clickBounce = Bounce(CLICK, 5);
Bounce leftBounce = Bounce(LEFT, 5);
Bounce rightBounce = Bounce(RIGHT, 5);
Bounce upBounce = Bounce(UP, 5);
Bounce downBounce = Bounce(DOWN, 5);

byte extID;
unsigned long heady;
byte datalength;
byte message[8];
int recording;
File myFile;

void setup()
{
  Serial.begin(115200);
  // set up pins for MCP2515        
  pinMode(P_CS,OUTPUT);      
  pinMode(P_SCK,OUTPUT);
  pinMode(P_MOSI,OUTPUT);
  pinMode(P_MISO,INPUT);
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(LEFT,INPUT);
  pinMode(RIGHT,INPUT);
  pinMode(CLICK,INPUT);
  pinMode(LED2, OUTPUT); 
  pinMode(LED3, OUTPUT);
  pinMode(LED, OUTPUT);
  
//  sLCD.begin(9600);
//  clear_lcd();
  
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);

  digitalWrite(UP, HIGH);       /* Enable internal pull-ups */
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  digitalWrite(CLICK, HIGH);
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
//  Serial.println("Init MCP2515");
  mcp2515_init(33, 0x00);  //Iinitialize MCP2515: 1Mbit/s, 500, 125 Kbits/s, 33kps with filters enabled. 
  mcp2515_write_register(CANCTRL, 0x00);
  delay(250);
  digitalWrite(LED, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
//  Serial.println("Init done");
//  sLCD.write(COMMAND);
//  sLCD.write(LINE0);
//  sLCD.print("Init Done");  
  
  if (!SD.begin(P_SD)) {
    Serial.println("SD init failed");
  }
  else {
    Serial.println("SD init suceeded");
  }
  
  recording = 0;
}

void loop() {
  byte flags;
  
  clickBounce.update();
  upBounce.update();
  downBounce.update();
  leftBounce.update();
  
  if (clickBounce.fallingEdge() == 1) {
    if (recording == 0) {
      char buf[50];
      unsigned long ms = millis();
      sprintf(buf, "%1u.txt", ms);
      myFile = SD.open(buf, FILE_WRITE);
      digitalWrite(LED, HIGH);
      recording = 1;
      myFile.print(millis());
      myFile.println(" - Opened");
    } else {
      myFile.print(millis());
      myFile.println(" - Closing");
      recording = 0;
      myFile.close();
      digitalWrite(LED, LOW);
    }
  }
  else if (upBounce.fallingEdge() == 1) {
    byte packet[] = {0x00, 0x00, 0x00, 0x01};
    Serial.print("Trying to send up jog... ");
    if (can_send_29bit_message(0x100D0060, sizeof(packet), packet)) {
      File writeFile = SD.open("writes.txt", FILE_WRITE);
      writeFile.print(millis());
      writeFile.print(" : ");
      writeFile.println("Jog Up");
      writeFile.close();
      Serial.println("suceeded");
    } else {
      Serial.println("failed");
    }
  }
  else if (downBounce.fallingEdge() == 1) {
    byte packet[] = {0x00, 0x00, 0x00, 0x1F};
    Serial.print("Trying to send down jog... ");
    if (can_send_29bit_message(0x100D0060, sizeof(packet), packet)) {
      File writeFile = SD.open("writes.txt", FILE_WRITE);
      writeFile.print(millis());
      writeFile.print(" : ");
      writeFile.println("Jog Down");
      writeFile.close();
      Serial.println("suceeded");
    } else {
      Serial.println("failed");
    }
  }
  else if (leftBounce.fallingEdge() == 1) {
    byte packet[] = {0x86, 0x78, 0x05, 0xFF, 0x05};
    Serial.print("Trying to send chime... ");
    if (can_send_29bit_message(0x1001E058, sizeof(packet), packet)) {
      File writeFile = SD.open("writes.txt", FILE_WRITE);
      writeFile.print(millis());
      writeFile.print(" : ");
      writeFile.println("Jog Down");
      writeFile.close();
      Serial.println("suceeded");
    } else {
      Serial.println("failed");
    }
  }
  flags = mcp2515_read_register(CANINTF);
  if ((flags & 0x01) == 0x01) {
    renderMsg(0x61);
    // Clear the message flag..
    mcp2515_modifyRegister(CANINTF, 0x01, 0x00);  
  }

  if ((flags & 0x02) == 0x02) {
    renderMsg(0x71);
    // Clear the message flag..
    mcp2515_modifyRegister(CANINTF, 0x02, 0x00);  
  }
  
  digitalWrite(LED2, LOW);
}

void renderMsg(uint8_t address) {
  byte i;

  mcp2515_read_can_id( address , &extID, &heady );
  mcp2515_read_canMsg( address,
  &datalength,
  &message[0]);

  processMessage();
}

void processMessage() {
  
  // File myFile = SD.open("output.txt", FILE_WRITE);
  if (recording == 1 && myFile) {
    myFile.print(millis());
    myFile.print(" : ");
    myFile.print(heady, HEX);
    myFile.print(" | ");
    myFile.print(datalength, DEC);
    myFile.print(" | ");
    for (int i=0; i<datalength; i++) {
      myFile.print(message[i], HEX);
      myFile.print(" ");
    }
    myFile.println("");
  } else {
    File tmpFile = SD.open("overflow.txt", FILE_WRITE);
    tmpFile.print(millis());
    tmpFile.print(" : ");
    tmpFile.print(heady, HEX);
    tmpFile.print(" | ");
    tmpFile.print(datalength, DEC);
    tmpFile.print(" | ");
    for (int i=0; i<datalength; i++) {
      tmpFile.print(message[i], HEX);
      tmpFile.print(" ");
    }
    tmpFile.println("");
    tmpFile.close();
  }

//  Serial.print(millis());
//  Serial.print(" : ");
//  Serial.print(heady, HEX);
//  Serial.print(" | ");
//  Serial.print(datalength, DEC);
//  Serial.print(" | ");
//  for (int i=0; i<datalength; i++) {
//    Serial.print(message[i], HEX);
//    Serial.print(" ");
//  }
//  Serial.println("");
//  digitalWrite(LED3, LOW);
}

void clear_lcd(void)
{
  sLCD.write(COMMAND);
  sLCD.write(CLEAR);
}  
