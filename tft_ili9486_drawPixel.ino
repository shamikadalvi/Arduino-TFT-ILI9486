/*
This is a modified and simplified code of Adafriut GFX and MCUFriend
This works for Arduino TFT shield 3.5 " 480*320 with driver ILI9486
As of now it has functions of drawing pixel,fillScreen and fillRect 
It works for sure as I have coded and tested it
No libraries or any other file is required
It uses only 10% of program storage
Feel free to modify and use only include this and given license 
below in the commented form 
@shamika dalvi



*/



/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.



vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvStartvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
Software License Agreement (FreeBSD License)

Copyright (c) 2018 David Prentice (https://github.com/prenticedavid/MCUFRIEND_kbv/)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

 */



//LCD PINS D7-D0


#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

//#include <SPI.h>          // f.k. for Arduino-1.5.2

#include "Arduino.h"


#define WIDTH 480
#define HEIGHT 320

#include "Arduino.h"

#define D7 7
#define D6 6
#define D5 5
#define D4 4
#define D3 3
#define D2 2
#define D1 9
#define D0 8



#define RD_edit A0
#define WR_edit A1
#define CD_edit A2
#define CS_edit A3
#define RESET_edit A4



#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4




#define RD_ACTIVE   digitalWrite(RD_edit,LOW)



#define RD_IDLE    digitalWrite(RD_edit, HIGH)




#define RD_OUTPUT  pinMode(RD_edit, OUTPUT)




#define WR_ACTIVE  digitalWrite(WR_edit,LOW)


#define WR_IDLE  digitalWrite(WR_edit,HIGH)


#define WR_OUTPUT   pinMode(WR_edit, OUTPUT)



#define CD_COMMAND digitalWrite(CD_edit, LOW)



#define CD_DATA digitalWrite(CD_edit, HIGH)



#define CD_OUTPUT  pinMode(CD_edit, OUTPUT)


#define CS_ACTIVE  digitalWrite(CS_edit, LOW)



#define CS_IDLE    digitalWrite(CS_edit, HIGH)



#define CS_OUTPUT    pinMode(CS_edit, OUTPUT)



#define RESET_ACTIVE  digitalWrite(RESET_edit, LOW)



#define RESET_IDLE    digitalWrite(RESET_edit, HIGH)



#define RESET_OUTPUT  digitalWrite(RESET_edit, OUTPUT)

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }     
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE     



uint16_t  _lcd_xor, _lcd_capable;
 uint16_t  _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;
 int16_t _width;     
 int16_t _height;  
 uint8_t rotation;
/*
#define  BLACK   0x0000
#define RED    0x001F
#define BLUE     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFDA0  
*/
#define  BLACK   0x0000
#define  BLUE    0x001F
#define  RED     0xF800
#define  GREEN   0x07E0
#define MAGENTA  0xF81F
#define  YELLOW  0xFFE0 
#define  WHITE    0xFFFF

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

void setup(void);
void loop(void);
unsigned long testFillScreen();


uint16_t g_identifier;


void setup(void) {
    Serial.begin(9600);
    uint32_t when = millis();
    
    if (!Serial) delay(5000);         

  
begin_new(0x9486);

 fillScreen_2();
   
}
       


void loop(void) {
  
  testFillRect();
}




void fillScreen_2(void)
  {
  
      for(int i=0;i<320;i++)
       {
        for(int j=0;j<480;j++)
           {
           drawPixel_2(0+i, 0+j, BLACK);  

      
           }

    
         }
  
  }

void testFillRect(void)
  {



//rect1

for(int i=100;i<105;i++)
 {
  for(int j=100;j<105;j++)
    {

        drawPixel_2(i, j, BLUE); 

    }
  
  }
//rect 2

  for(int i=110;i<120;i++)
 {
  for(int j=110;j<120;j++)
    {

        drawPixel_2(i, j, RED);  

        
    
    }
  
  }
 
   
  }




void drawPixel_2(int16_t x, int16_t y, uint16_t color)
{
   
  

   

    setAddrWindow_2(x, y, x, y);

    WriteCmdData(_MW, color);
}





void begin_new(uint16_t ID)



  {
  
    
      reset_new();

    
    
     _lcd_xor = 0;
     _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS; 




            CS_ACTIVE;
            CD_COMMAND;
            write8(0x01);
            CS_IDLE;
           
             delay(150); 
      
       
            CS_ACTIVE;
            CD_COMMAND;
            write8(0x28);
            CS_IDLE;
         
     
     
            CS_ACTIVE;
            CD_COMMAND;
            write8(0x3A);
      
      
            CD_DATA;
            write8(0x55);
      
      
      
            CS_IDLE;
     
     //power control 1
     
     
     
             CS_ACTIVE;
            CD_COMMAND;
            write8(0xC0);
      
      
            CD_DATA;
            write8(0x0d);
      
      
             CD_DATA;
            write8(0x0d);
      
      
      
            CS_IDLE;
      
      
      
         //power control 2
     
     
     
        CS_ACTIVE;
            CD_COMMAND;
            write8(0xC1);
      
      
      CD_DATA;
            write8(0x43);
      
      
      CD_DATA;
            write8(0x00);
      
      
      
            CS_IDLE;      
      
      
      
       //power control 3
     
     
     
        CS_ACTIVE;
            CD_COMMAND;
            write8(0xC2);
      
      
      CD_DATA;
            write8(0x00);
      
      
            CS_IDLE;    
      
      
      
      
      //VCOM  Control 1
     
     
     
        CS_ACTIVE;
            CD_COMMAND;
            write8(0xC5);
      
      
      CD_DATA;
            write8(0x00);
      
      
        CD_DATA;
            write8(0x48);
      
      
        CD_DATA;
            write8(0x00);
      
      
        CD_DATA;
            write8(0x48);
      
      
            CS_IDLE;    
     
     
     
      //Inversion Control
      
      
        CS_ACTIVE;
            CD_COMMAND;
            write8(0xB4);
      
      
      CD_DATA;
            write8(0x00);
      
      
        CS_IDLE;
     
     
     
      // Display Function Control  0xB6, 3, 0x02, 0x02, 0x3B,
      
      
      
            CS_ACTIVE;
            CD_COMMAND;
            write8(0xB6);
      
      
            CD_DATA;
            write8(0x02);
      
      
      
            CD_DATA;
            write8(0x02);
      
      
            CD_DATA;
            write8(0x3B);
      
      
      
      
        CS_IDLE;
      
      
     ///gamma control
    //0xE0, 15, 0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00, 
   


    
     
          CS_ACTIVE;
          CD_COMMAND;
          write8(224); //0xE0
    
    
          CD_DATA;
          write8(15);  //0x0F
    
    
          CD_DATA;
          write8(33); //0x21
    
    
          CD_DATA;
          write8(28); //0x1C
    
    
          CD_DATA;
          write8(11); //0x0B

          CD_DATA;
          write8(14); //0x0E

          
          CD_DATA;
          write8(8); //0x08

          
          CD_DATA;
          write8(73); //0x49

          
          CD_DATA;
          write8(152); //0x98



          CD_DATA;
          write8(56); //0x38

          
          CD_DATA;
          write8(9); //0x09


          CD_DATA;
          write8(17); //0x11

          
          CD_DATA;
          write8(3); //0x03

        
          CD_DATA;
          write8(20);  //0x14

          
          CD_DATA;
          write8(16); //0x10    

          CD_DATA;
          write8(0);  // 0x00   


    
    
          CS_IDLE;   

        //0xE1, 15, 0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00,      
    

            
          CS_ACTIVE;
          CD_COMMAND;
          write8(225);   //0xE1
    
    
         
    
    
          CD_DATA;
          write8(15); //0x0F
    
    
          CD_DATA;
          write8(47);  //0x2F 
    
    
          CD_DATA;
          write8(43); //0x2B

          CD_DATA;
          write8(12); //0x0C

          
          CD_DATA;
          write8(14);  //0x0E

          
          CD_DATA;
          write8(6);   //0x06

          
          CD_DATA;
          write8(71);   //0x47



          CD_DATA;
          write8(118);   //0x76

          
          CD_DATA;
          write8(55);   //0x37


          CD_DATA;
          write8(7);  //0x07

          
          CD_DATA;
          write8(17);  //0x11

        
          CD_DATA;
          write8(4);    //0x04

          
          CD_DATA;
          write8(35);   //0x23    

          CD_DATA;
          write8(30);    //0x1E


          CD_DATA;
          write8(0);  //0x00   


    
    
          CS_IDLE;   
 
      
  
  
  
  
      

      
       //Sleep Out
      
           CS_ACTIVE;
            CD_COMMAND;
            write8(0x11);
            CS_IDLE;
      
      
      delay(150);
      
       //Display On
        CS_ACTIVE;
        CD_COMMAND;
        write8(0x29);
        CS_IDLE;
      
      
      

     setRotation_2();
  
  
  }


static void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
  {

       CS_ACTIVE;
    WriteCmd(cmd);
  
  
      write8(d1);
      write8(d2);
      write8(d3);
      write8(d4);
  
       CS_IDLE;
  
  
  }



void setAddrWindow_2(int16_t x, int16_t y, int16_t x1, int16_t y1)
   {

 

              if (_lcd_capable & MIPI_DCS_REV1) {
     
   
         CS_ACTIVE;
         WriteCmd(_SC);
  
  
        write8(x >> 8);
        write8(x);
        write8(x1 >> 8);
        write8(x1);
  
         CS_IDLE;
  
  
  
         CS_ACTIVE;
         WriteCmd(_SP);
  
  
        write8(y >> 8);
        write8(y);
        write8(y1 >> 8);
        write8(y1);
  
         CS_IDLE;
    
    
       
    }

                 

  
  }





void setRotation_2(void)
  {


       uint16_t GS, SS_v, ORG, REV = _lcd_rev;
       uint8_t val, d[3];
       rotation = 0;           
       _width =  HEIGHT ;
       _height =  WIDTH ;
       val = 0x48; 

          if (_lcd_capable & INVERT_GS)
        val ^= 0x80;
    if (_lcd_capable & INVERT_SS)
        val ^= 0x40;
    if (_lcd_capable & INVERT_RGB)
        val ^= 0x08;
    if (_lcd_capable & MIPI_DCS_REV1) {
       
      //common_MC:
    
        _MC = 0x2A;
        _MP = 0x2B;
        _MW = 0x2C;
        _SC = 0x2A;
        _EC = 0x2A;
        _SP = 0x2B;
        _EP = 0x2B;


    }

      
        setAddrWindow_2(0, 0, _width - 1, _height - 1);
  
  }



 void reset_new(void)
  {


    RD_OUTPUT; 
  WR_OUTPUT; 
  CD_OUTPUT;
  CS_OUTPUT; 
  RESET_OUTPUT;
  
  
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
  
  
  }



void WriteCmd(uint16_t x)
{ CD_COMMAND;
  write16(x);
  CD_DATA; 
}


void WriteData(uint16_t x)
{ 
  write16(x);
 
}






 void WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}





void write_8(uint16_t x)

{
  
   pinMode(D7, OUTPUT);
   pinMode(D6, OUTPUT);
   pinMode(D5, OUTPUT);
   pinMode(D4, OUTPUT);
   pinMode(D3, OUTPUT);
   pinMode(D2, OUTPUT);
   pinMode(D1, OUTPUT);
   pinMode(D0, OUTPUT);

   
   
   digitalWrite(D7,x&0x80);
   
 
   
   
    digitalWrite(D6,x&0x40);

   
   
     digitalWrite(D5,x&0x20);
 
   
   
      digitalWrite(D4,x&0x10);
   
  
   
    digitalWrite(D3,x&0x08);
  
   digitalWrite(D2,x&0x04);
  
   
     digitalWrite(D1,x&0x02);
  
     digitalWrite(D0,x&0x01);
   
 
  
}



void write8(uint16_t x)     {
  
  write_8(x);
  WR_STROBE; 
  }
  
void write16(uint16_t x)  

{ 
  uint8_t h = (x)>>8;
  uint8_t l = x;
  write8(h);
  write8(l);
}





