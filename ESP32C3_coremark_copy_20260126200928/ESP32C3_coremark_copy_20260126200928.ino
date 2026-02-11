//ESP32 test code for coremark and IIC
//V2.0 by Zell
//2.Feb.2026

#include <Arduino.h>
#include <core_arduino.h>
#include <FastLED.h>
#include <coremark.h>
#include <Wire.h>
#include "IIC_scan.h"

//

21:53:07.495 -> I2C device found at address 0x38
21:53:07.538 -> I2C device found at address 0x77

//WS2812 LED
//Pixel LED defines
#define NUM_LEDS 1
#define DATA_PIN 8  //GPIO8 on C3 devkit wroom n4r8
#define BRIGHTNESS 32
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
uint8_t LED_brightness = 20;
float CPU_Speed=160; //Mhz
float CM_score = 0;
float score_per_Mhz = 0;
extern core_results results[1];
extern CORE_TICKS   total_time;

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32C3 Arduino coremark demo by Zell, 26.Jan.2026");
    Serial.println("demo based on core_arduino, modified by Zell");
    //__DATE__ and __TIME__,
    Show_ESP32_sys_info();
    Serial.printf("-FW Compile time: %s, date: %s\r\n", __TIME__, __DATE__);

    Serial.println("Turn on Onboard WS2812 LED");
      FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Green;
    FastLED.setBrightness(LED_brightness);
  FastLED.show();
  show_IIC_pins();
  show_SPI_pins();
   Serial.println(">:IIC init now...");
  Wire.begin(SDA, SCL);
}

void loop()
{   
    IIC_scan();
    Serial.println(">>:startCoremark now..!\r\n");
    leds[0] = CRGB::Red;
    FastLED.show();
    startCoremark();
    
    //CM_score = results[0].iterations/time_in_secs(total_time);
    score_per_Mhz = CM_score/CPU_Speed;
    Serial.printf(">>:Coremark Finshed! Score: %4.2f/Mhz \r\n",score_per_Mhz);
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(2000);
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(2000);
    leds[0] = CRGB::Orange;
    FastLED.show();
    delay(1000);
}


void show_IIC_pins(void) {
  Serial.println("ESP32 IIC interface:");
  Serial.print("SDA: ");
  Serial.println(SDA);
  Serial.print("SCL: ");
  Serial.println(SCL);
  /*
static const uint8_t SDA = 8;
static const uint8_t SCL = 9;
*/
}

void show_SPI_pins(void) {
  Serial.println("ESP32 SPI interface:");
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);
  /*
20:00:47.005 -> ESP32s3 SPI interface:
20:00:47.005 -> MOSI: 11
20:00:47.005 -> MISO: 13
20:00:47.005 -> SCK: 12
20:00:47.009 -> SS: 10
*/
}

void Show_ESP32_sys_info (void){

  // Print the current CPU frequency
Serial.print("CPU Frequency: ");
Serial.print(getCpuFrequencyMhz());
Serial.println(" MHz");

// Print the XTAL crystal frequency
Serial.print("XTAL Frequency: ");
Serial.print(getXtalFrequencyMhz());
Serial.println(" MHz");

// Print the APB bus frequency
Serial.print("APB Bus Frequency: ");
Serial.print(getApbFrequency()/1000000);
Serial.println(" MHz");
}
/*
.894 -> CPU Frequency: 160 MHz
21:31:39.894 -> XTAL Frequency: 40 MHz
21:31:39.897 -> APB Bus Frequency: 80000000 Hz
*/
/*
C3
20:59:19.033 -> >>:startCoremark now..!

20:59:36.230 -> 2K performance run parameters for coremark.
20:59:36.232 -> CoreMark Size    : 666
20:59:36.235 -> Total ticks      : 13461
20:59:36.235 -> Total time (secs): 13.46
20:59:36.238 -> Iterations/Sec   : 297.15
20:59:36.241 -> Iterations       : 4000
20:59:36.243 -> Compiler version : GCC14.2.0
20:59:36.246 -> Compiler flags   : (flags unknown)
20:59:36.256 -> Memory location  : STACK
20:59:36.256 -> seedcrc          : 0xE9F5
20:59:36.256 -> [0]crclist       : 0xE714
20:59:36.256 -> [0]crcmatrix     : 0x1FD7
20:59:36.257 -> [0]crcstate      : 0x8E3A
20:59:36.260 -> [0]crcfinal      : 0x65C5
20:59:36.263 -> Correct operation validated. See README.md for run and reporting rul
es.
20:59:36.278 -> CoreMark 1.0 : 297.15 / GCC14.2.0 (flags unknown) / STACK
*/

/* S3 469 core 1
23:02:24.362 -> Turn on Onboard WS2812 LED
23:02:24.362 -> >>:startCoremark now..!

23:02:39.486 -> CoreMark Size    : 666ameters for coremark.
23:02:39.492 -> Total ticks      : 12772
23:02:39.492 -> Total time (secs): 12.77
23:02:39.492 -> Iterations/Sec   : 469.78
23:02:39.498 -> Iterations       : 6000
23:02:39.498 -> Compiler version : GCC14.2.0
23:02:39.503 -> Compiler flags   : (flags unknown)
23:02:39.503 -> Memory location  : STACK
23:02:39.536 -> seedcrc          : 0xE9F5
23:02:39.536 -> [0]crclist       : 0xE714
23:02:39.536 -> [0]crcmatrix     : 0x1FD7
23:02:39.536 -> [0]crcstate      : 0x8E3A
23:02:39.536 -> [0]crcfinal      : 0xA14C
23:02:39.536 -> Correct operation validated. See README.md for run and reporting rules.
23:02:39.536 -> CoreMark 1.0 : 469.78 / GCC14.2.0 (flags unknown) / STACK
*/