/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
 //V1.12 added WiFi get Quote test function, test works!
 //V1.11 added some fonts, opt for display
//V1.10 fixed  EPD seconds show bug, 17.Jan.2026
//V1.09 trying to to add debug info for EPD seconds show bug,
//V1.08 fixed EPD seconds show bug, failed
//V1.07 opt of sleep, turn off LED during sleep, Active run 45mA, sleep <4mA
//V1.06 added light sleep to save power, test works!
//V1.05 added CO2 history data and average data show
//V1.04 added EPD CO2 display, added Min Max display
//V1.03 SCD40 detected, working, every 1s updates
//V1.02 modified pin to fit IIC pins for SCD40,
//V1.01. basic EPD working
//depend on Adafruit_EPD and FastLED and SparkFun_SCD4x_Arduino_Library

//use debug port for uploading FW!
#include "Adafruit_EPD.h"
#include <Fonts/FreeMonoBold9pt7b.h> //has a height of about 18 pixels,
#include <Fonts/FreeMono9pt7b.h> 
//#include <Fonts/FreeMono9pt7b.h> //
//#include <Fonts/FreeSans12pt7b.h> //
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

//#include <Fonts/Picopixel.h>
#include <Fonts/FreeSerifItalic9pt7b.h>

#include <FastLED.h>

#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"

#include <Wire.h>

#include <Benchmark.h>
#include "SparkFun_SCD4x_Arduino_Library.h"  //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x

#include "Net_config.h"

SCD4x SCD40Sensor;                           //The default I2C address for the SCD4x is 0x62.
#define ESP32_sleep_interval 5000            //in ms 5s
//versions
#define Version_Nrd '1'
#define Version_Nrf1 '1'  //2._x
#define Version_Nrf2 '2'  //2.x_

#define Init_test_EPD_EN 0
#define SCD40Sensor_EN 1
#define Loop_perodic_update_EN 1
#define Loop_WiFi_perodic_update_EN 1
#define  WiFi_refresh_interval 5
uint16_t  WiFi_refresh_interval_tmp = WiFi_refresh_interval ;

#define EPD_refresh_interval 15 //30
uint64_t sleepTime = ESP32_sleep_interval * 1000;  // Sleep duration in microseconds (1 seconds)
uint64_t sys_on_time_second = 0;
uint32_t sys_on_time_hour = 0;
uint64_t sys_on_time_second_tmp = 0;
uint64_t sleepTime_seconds = ESP32_sleep_interval / 1000;

#undef ARDUINO_ADAFRUIT_FEATHER_RP2040_THINKINK
#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040_THINKINK  // detects if compiling for \
                                                 // Feather RP2040 ThinkInk
#define EPD_DC PIN_EPD_DC                        // ThinkInk 24-pin connector DC
#define EPD_CS PIN_EPD_CS                        // ThinkInk 24-pin connector CS
#define EPD_BUSY PIN_EPD_BUSY                    // ThinkInk 24-pin connector Busy
#define SRAM_CS -1                               // use onboard RAM
#define EPD_RESET PIN_EPD_RESET                  // ThinkInk 24-pin connector Reset
#define EPD_SPI &SPI1                            // secondary SPI for ThinkInk
#else
#define EPD_DC 5      //seems OK original 10,
#define EPD_CS 10     //Low wirh SCK, OK!  ori 9, confilict with SCL
#define EPD_BUSY 7    //7  INPUT pin, can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS -1    //6, show waveform signal! not used for ESP32?, it is for Adafruit Metro M4
#define EPD_RESET 21  // ori 8, can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI  // primary SPI, HSPI? #include <SPI.h>?
#endif
#define SPI_CS 10  //Low wirh SCK, OK! Use this pin for EPD CS1
//SPI MOSI 11  seems has missing waveforms!
//MISO 13
//SCK 12
//SS: 10
//IIC wire
// -> SDA: 8
// -> SCL: 9

//Pixel LED defines
#define NUM_LEDS 1
#define DATA_PIN 48  //GPIO48 on S3 devkit wroom n8r8
#define BRIGHTNESS 64
#define COLOR_ORDER GRB
uint8_t color_change_idx = 0;



#define EPD_width 250
#define EPD_height 122
//#define EPD_height 104

//https://docs.arduino.cc/libraries/adafruit-epd/
// Uncomment the following line if you are using 1.54" EPD with IL0373
// Adafruit_IL0373 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1680
// Adafruit_SSD1680 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,


//Uncomment the following line if you are using 2.13" EPD with SSD1680
Adafruit_SSD1680 display(EPD_width, EPD_height, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
//Adafruit_SSD1680 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SPI_CS, EPD_BUSY, EPD_SPI); //tested partialy working

// Uncomment the following line if you are using 2.13" EPD with SSD1675
//Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,EPD_BUSY, EPD_SPI); //not working!
//Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SPI_CS,EPD_BUSY, EPD_SPI); //not working!


// Uncomment the following line if you are using 2.13" EPD with SSD1675B
//Adafruit_SSD1675B display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);//not working!

// Uncomment the following line if you are using 2.13" EPD with UC8151D
//Adafruit_UC8151D display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);//not working!

// Uncomment the following line if you are using 2.13" EPD with IL0373
//Adafruit_IL0373 display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
//#define FLEXIBLE_213

// Uncomment the following line if you are using 2.9" EPD with IL0373
// Adafruit_IL0373 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI); #define FLEXIBLE_290

// Uncomment the following line if you are using 2.9" EPD with SSD1680
// Adafruit_SSD1680 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.9" EPD with UC8151D
// Adafruit_UC8151D display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED



uint8_t SCD40Sensor_exist = 0;
uint16_t CO2_cur = 0;
uint16_t CO2_Min = 0;
uint16_t CO2_Max = 0;
uint16_t CO2_ave = 0;
#define CO2_history_ave_size 60
uint16_t CO2_history[CO2_history_ave_size];
uint16_t CO2_data_idx = 0;
uint32_t CO2_data_cnt = 0;
float Temperature_cur = 0;
float Humidity_cur = 0;
uint16_t altitude = 100;

uint16_t pixel_pos_x, pixel_pos_y;
char text_buf[128];



//WS2812 LED
CRGB leds[NUM_LEDS];
uint8_t LED_brightness = 20;

void show_SPI_pins(void);

uint32_t sys_cnt = 0;
char version_buf[4];
uint16_t wifi_sucess_cnt = 0;
char quote_text[100]; 

void setup() {
  Serial.begin(115200);

  version_buf[0] = Version_Nrd;
  version_buf[1] = '.';
  version_buf[2] = Version_Nrf1;
  version_buf[3] = Version_Nrf2;

  // while (!Serial) { delay(10); }
  //Serial.println("\r\n<Adafruit 2.13 BW+R EPD SCD40 sensor test V1.08B by Zell Jan.2026>");
  Serial.println("\r\n<Adafruit 2.13 BW+R EPD SCD40 sensor test by Zell 2026>");
  Serial.printf("-Version %sB, 17.Jan.2025 by Zell-\r\n", version_buf);
  Serial.println("---Code Configuration---");
  Serial.printf("#Init_test_EPD_EN %u;SCD40Sensor_EN %u;Loop_perodic_update_EN %u\r\n", Init_test_EPD_EN, SCD40Sensor_EN, Loop_perodic_update_EN);
  Serial.printf(">:EPD_width %u;EPD_height %u\r\n", EPD_width, EPD_height);
  show_SPI_pins();
  show_IIC_pins();
  Serial.println("Turn on Onboard WS2812 LED");
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Green;
  FastLED.setBrightness(LED_brightness);
  FastLED.show();
  //display.begin(true);
  Serial.println(">:init SPI...");

  //SPI.begin();
  Serial.println(">:init EPD...");
  delay(50);
  display.begin(true);
  //display.powerUp();

#if defined(FLEXIBLE_213) || defined(FLEXIBLE_290)
  // The flexible displays have different buffers and invert settings!
  display.setBlackBuffer(1, false);
  display.setColorBuffer(1, false);
  Serial.println("#>:display.setColorBuffer for flexible screen!");
#endif
  /*
  // large block of text
  display.clearBuffer();
  Serial.println(">:Draw texts...");
  testdrawtext(
      "ZELL EPD test,1.Jan.2026.Curabitur ",
      COLOR1);
  display.display();
  Serial.println(">:waiting for refresh...");
  delay(8000);
  */
  if (SCD40Sensor_EN) {
    Serial.println(">:init IIC...");
    Wire.begin();
    if (SCD40Sensor.begin() == false) {
      Serial.println(F("SCD40 Sensor not detected. Please check wiring..."));

    } else {
      Serial.println(F("SCD40 Sensor detected !"));
      SCD40Sensor_exist = 1;
      if (SCD40Sensor.stopPeriodicMeasurement() == true) {
        Serial.println(F("#>:SCD40 Periodic measurement is disabled!"));
      }
      if (SCD40Sensor.startLowPowerPeriodicMeasurement() == true) {
        Serial.println(F("#>:SCD40 Low power mode enabled!"));
      }
      Serial.printf("#>:SCD40 setSensorAltitude:%u!\r\n", altitude);
      SCD40Sensor.setSensorAltitude(altitude, 10);
    }
  }

  display.clearBuffer();
  if (Init_test_EPD_EN) {
    Serial.println(">:Draw test lines...");
    for (int16_t i = 0; i < EPD_width; i += 4) {
      display.drawLine(0, 0, i, EPD_height - 1, COLOR2);
    }
  }
  /*
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i,
                     COLOR1); // on grayscale this will be mid-gray
  }
  */

  Serial.println(">:waiting for refresh...");
  display.display();
  delay(3000);
  //display.display();

  // large block of text
  display.clearBuffer();
  Serial.println(">:Draw texts...");
  //test only
  display.setTextSize(1);
  testdrawtext(
    "ZELL EPD test,8.Jan.2026. Version 1.02 ! "
    "2.13 BW+Red HINK E-INK Display,250*122,  Driver Unknown!",
    COLOR1);
  //
  //display.setFont(&Picopixel);//tiny
  //display.setFont(&FreeSerifItalic9pt7b); 
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("A", 0, 20, &x1, &y1, &w, &h);
  Serial.print("Default Font Width: "); Serial.println(w);
  Serial.print("Font Height: "); Serial.println(h);
  /*
  Create a GFXcanvas1 object (an offscreen bitmap) for a fixed-size area, draw custom text in there and copy to the screen using drawBitmap().*/
  display.setFont(&FreeSerifItalic9pt7b);
  display.setTextSize(1);
  testdrawtext2(
    "Get as much education as you can. Nobody can take that away from you!",
    COLOR2);
  display.display();
  display.setFont();
  display.setTextSize(1);
  Serial.println(">:waiting for refresh...");
  delay(1000);

  memset(CO2_history, 0, sizeof(CO2_history));

  //Serial.printf("Connecting to %s ", ssid);
  //WiFi.begin(ssid, password);
  // Enable wake-up by timer
  esp_err_t result = esp_sleep_enable_timer_wakeup(sleepTime);

  if (result == ESP_OK) {
    Serial.printf("Timer Wake-Up set successfully as wake-up source. sleep interval is %u ms\r\n", ESP32_sleep_interval);
  } else {
    Serial.println("Failed to set Timer Wake-Up as wake-up source.");
  }
}

void loop() {
  // don't do anything!
  sys_cnt++;
  color_change_idx++;
  Serial.printf(">>:loop running! %u\r\n", sys_cnt);

  sys_on_time_hour = sys_on_time_second / 3600;
  if (sys_on_time_hour > 0) {
    sys_on_time_second_tmp = sys_on_time_second - sys_on_time_hour * 3600;
    Serial.printf(">>:sys on time since last boot %u hour,%lu s!\r\n", sys_on_time_hour, sys_on_time_second_tmp);
  } else {
    Serial.printf(">>:sys on time since last boot %u s!\r\n", sys_on_time_second);
    sys_on_time_second_tmp = sys_on_time_second;
  }

  if (1 == color_change_idx)
    leds[0] = CRGB::Coral;
  else if (2 == color_change_idx)
    leds[0] = CRGB::Azure;
  else {
    leds[0] = CRGB::Green;
    color_change_idx = 0;
  }
  FastLED.show();

  if (SCD40Sensor_exist) {
    if (SCD40Sensor.readMeasurement())  // readMeasurement will return true when fresh data is available
    {
      Serial.println();
      CO2_cur = SCD40Sensor.getCO2();
      Temperature_cur = SCD40Sensor.getTemperature();
      Humidity_cur = SCD40Sensor.getHumidity();
      CO2_data_cnt++;
      Serial.printf("CO2(ppm): %u ", CO2_cur);
      Serial.printf("\tTemperature(C): %3.1f ", Temperature_cur);
      Serial.printf("\tHumidity(%RH): %3.1f%%\r\n", Humidity_cur);
      if (CO2_data_idx < CO2_history_ave_size) {
        CO2_history[CO2_data_idx] = CO2_cur;
        CO2_data_idx++;
      }
      uint32_t CO2_SUM_tmp = 0;
      for (uint16_t idx = 0; idx < CO2_history_ave_size; idx++) {

        CO2_SUM_tmp += CO2_history[idx];
      }
      if (CO2_data_cnt < CO2_history_ave_size) {
        CO2_ave = CO2_SUM_tmp / CO2_data_cnt;
      } else {
        CO2_ave = CO2_SUM_tmp / CO2_history_ave_size;
      }
      Serial.printf("#>:Average CO2(ppm): %u (total data counts:%u)", CO2_ave, CO2_data_cnt);
      //Serial.print(CO2_cur);

      //Serial.print(F("\tTemperature(C):" ));
      //Serial.print(SCD40Sensor.getTemperature(), 1);

      //Serial.print(F("\tHumidity(%RH):"));
      //Serial.print(SCD40Sensor.getTemperature(), 1);
      //19:29:20.082 -> CO2(ppm):894Temperature(C):10.8Humidity(%RH):55.5
      Serial.println();
    }
    if (1 == sys_cnt) {
      Serial.println(">:Draw EPD for first loop!");

      //sprintf(text_buf,"SCD40 sensor recorder APP V1.05 by ZELL Jan.2026",CO2_cur,Temperature_cur,Humidity_cur);
      //EPD_draw_title_info(text_buf,COLOR1);
      if (sys_on_time_hour > 0) {
        sys_on_time_second_tmp = sys_on_time_second - sys_on_time_hour * 3600;
        sprintf(text_buf, ">:sys on time %u hour,%lu s!", sys_on_time_hour, sys_on_time_second_tmp);
        //Serial.printf(">>:sys on time since last boot %u hour,%lu s!\r\n",sys_on_time_hour,sys_on_time_second_tmp);
      } else {
        //Serial.printf(">>:sys on time since last boot %u s!\r\n",sys_on_time_second);
        sys_on_time_second_tmp = sys_on_time_second;
        sprintf(text_buf, ">:sys on time %lu s!", sys_on_time_second_tmp);
      }



      //sprintf(text_buf,">:sys on time %u hour,%lu s!",sys_on_time_hour,sys_on_time_second_tmp);
      //Serial.printf(">Debug>:%s \r\n", text_buf);
      EPD_draw_sys_on_time(text_buf, COLOR1);
      memset(text_buf, 0, sizeof(text_buf));
      sprintf(text_buf, "CO2:%u,T:%3.1f,H:%2.0f", CO2_cur, Temperature_cur, Humidity_cur);
      display.setTextSize(2);
      Sensor_info_draw(text_buf, COLOR2);
      display.display();
      Serial.println("refresh EPD for the first sensor readings finished now!");
      delay(5000);
      CO2_Min = CO2_cur;
      Serial.println(">>:System benchmark now!");
      benchBegin();
      //delay(1);
      benchEnd(Serial);  //about 970us? bench: 972.5126us (233403 ticks)
    }
    if (CO2_cur > CO2_Max) {
      CO2_Max = CO2_cur;
    }
    if (CO2_cur < CO2_Min) {
      CO2_Min = CO2_cur;
    }
  }
  if (Loop_WiFi_perodic_update_EN) {
    if (0 == sys_cnt % WiFi_refresh_interval_tmp) {
      WiFi_connect_quote_test(15);
      if (wifi_sucess_cnt>0){
         WiFi_refresh_interval_tmp = wifi_sucess_cnt*EPD_refresh_interval ; 

      }
    }
  }
  if (Loop_perodic_update_EN) {
    if (0 == sys_cnt % EPD_refresh_interval) {
      leds[0] = CRGB::Red;
      FastLED.show();
      Serial.printf(">>:CO2 Max:%u ;CO2 Min:%u\r\n", CO2_Max, CO2_Min);
      Serial.println(">>:updating EPD buffer now!");

      sprintf(text_buf, "sys_cnt:%u", sys_cnt);
      testdrawsys_info(text_buf, COLOR1); //will clear buffer

      if (sys_on_time_hour > 0) {
        sys_on_time_second_tmp = sys_on_time_second - sys_on_time_hour * 3600;
        sprintf(text_buf, ">:sys on time %u hour,%lu s!", sys_on_time_hour, sys_on_time_second_tmp);
        //Serial.printf(">>:sys on time since last boot %u hour,%lu s!\r\n",sys_on_time_hour,sys_on_time_second_tmp);
      } else {
        //Serial.printf(">>:sys on time since last boot %u s!\r\n",sys_on_time_second);
        sys_on_time_second_tmp = sys_on_time_second;
        sprintf(text_buf, ">:sys on time %lu s!", sys_on_time_second_tmp);
      }
      //sprintf(text_buf,">>:sys on time %u hour,%lu s!",sys_on_time_hour,sys_on_time_second_tmp);
      EPD_draw_sys_on_time(text_buf, COLOR1);
      
      
      // display.setFont(&FreeSerif9pt7b);
      display.setFont(&FreeSans9pt7b); //too big
      display.setTextSize(2);
      sprintf(text_buf, "CO2:%u", CO2_cur );
      Sensor_info_draw(text_buf, COLOR2);
      display.setFont(&FreeSerif9pt7b); //too big
      display.setTextSize(1);
      sprintf(text_buf, "T:%3.1f C", Temperature_cur);
      SensorT_info_draw(text_buf, COLOR2);
      //sprintf(text_buf, "T:%3.1fc H:%3.1f%%", Temperature_cur,Humidity_cur);
      sprintf(text_buf, "H:%3.1f %%", Humidity_cur);
      Sensor_info2_draw(text_buf, COLOR2);
      //sprintf(text_buf, "CO2:%u,T:%3.1fC,          H:%3.1f%%", CO2_cur, Temperature_cur, Humidity_cur); //ori with def font

      

      display.setFont();

      sprintf(text_buf, "CO2Max:%u,Min:%u", CO2_Max, CO2_Min);
      Sensor_info_MinMax_draw(text_buf, COLOR1);
      sprintf(text_buf, "Ave:%u, cnt:%u", CO2_ave, CO2_data_cnt);
      Sensor_info_average_draw(text_buf, COLOR1);

      //display.setFont(&FreeSans9pt7b);
      display.setTextSize(1);
      sprintf(text_buf, "SCD40 sensor APP V%s by ZELL Jan.2026", version_buf);
      EPD_draw_title_info(text_buf, COLOR1);
      if(wifi_sucess_cnt>0){

        Quote_text_draw(quote_text,EPD_RED);
      }

      Serial.println(">>:Push EPD display refresh now!");
      display.display();
      display.setFont();

      delay(3000);
      Serial.println(">>:EPD display refresh finished now!");
    }
  }

  /*
    if (SCD40Sensor_EN){
        if (SCD40Sensor.begin() == false)
    {
      Serial.println(F("SCD40 Sensor not detected. Please check wiring..."));

    }
    else {
      Serial.println(F("SCD40 Sensor detected !"));
    }
  }
  */
  Serial.println(">>:Going into light sleep mode,turn off WS2812 LED now!\r\n");
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(50);                //wait for UART
  esp_light_sleep_start();  // Enter light sleep
  Serial.println("!>:Returning from light sleep");
  sys_on_time_second = sys_on_time_second + sleepTime_seconds;
  //delay(1000);
}

void testdrawtext(const char *text, uint16_t color) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void testdrawtext2(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = 0;
  //pixel_pos_y = EPD_height / 2 - text_height;
  pixel_pos_y = EPD_height / 2 ;
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void testdrawsys_info(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  pixel_pos_x = EPD_width / 2 + EPD_width / 5 - text_height;  //Botton right
  pixel_pos_y = EPD_height - 2 * text_height;
  display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

// Serial.printf(">>:sys on time since last boot %u hour,%u s! %u\r\n",sys_on_time_hour,sys_on_time_second_tmp);
void EPD_draw_sys_on_time(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  pixel_pos_x = text_height;  //Botton right
  pixel_pos_y = EPD_height - 2 * text_height;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void EPD_draw_title_info(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  pixel_pos_x = text_height;  //Botton right
  pixel_pos_y = 0;
  // display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Sensor_info_draw(const char *text, uint16_t color) {
  //first line
  uint8_t text_height = 18;
  pixel_pos_x = 4;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 - 2 * text_height +16;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void SensorT_info_draw(const char *text, uint16_t color) {
  //first line
  uint8_t text_height = 18;
  uint8_t text_width = 12;
  pixel_pos_x = EPD_width/2+4*text_width;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 - 2 * text_height+6;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Sensor_info2_draw(const char *text, uint16_t color) {
  //second line
  uint8_t text_height = 18;
  uint8_t text_width = 12;
  //pixel_pos_x = EPD_width/2-4;  // EPD_weight/2;
  pixel_pos_x = EPD_width/2+4*text_width;
  pixel_pos_y = EPD_height / 2 -  text_height +8;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void Sensor_info_MinMax_draw(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = 4;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 -4 ;
  //display.clearBuffer();
  display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void EPD_test_draw_rect(void) {
  //Serial.println("Color rectangle demo");
  display.clearBuffer();
  display.fillRect(display.width() / 3, 0, display.width() / 3,
                   display.height(), EPD_BLACK);
  display.fillRect((display.width() * 2) / 3, 0, display.width() / 3,
                   display.height(), EPD_RED);
}
//need test
//display.drawBitmap
void Quote_text_draw(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = 2;  // EPD_weight/2;
  pixel_pos_y = EPD_height - 3 * text_height;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Sensor_info_average_draw(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = 4;  // EPD_weight/2;
  pixel_pos_y = EPD_height - 2 * text_height + 4;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}
void show_IIC_pins(void) {
  Serial.println("ESP32s3 IIC interface:");
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
  Serial.println("ESP32s3 SPI interface:");
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

void WiFi_connect_quote_test (uint8_t retry_cnt){
  uint8_t try_cnt = 0;
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
    try_cnt++;
    if (retry_cnt<try_cnt){
      Serial.print(">>: WiFi Connection failed!\r\n");
      
      return;
    }
  }

  Serial.print("Connected to ");
  Serial.println(ssid);

  Serial.printf("\nStarting connection to server: %s...\r\n",server);
  client.setInsecure();
  if (!client.connect(server, 443)) {
    Serial.println("server Connection failed!");
    return;
  }
  Serial.println("Connected to server!");
  // Make a HTTP request:
  client.print("GET "); client.print(path); client.println(" HTTP/1.1");
  client.print("Host: "); client.println(server);
  client.println("Connection: close");
  client.println();

  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    Serial.print(F("Unexpected response: "));
    Serial.println(status);
   return;
  }
    while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  //[{"text":"You don't make progress by standing on the sidelines, whimpering and complaining. You make progress by implementing ideas","author":"Shirley Chisholm"}]
  while (client.peek() != '[') {
    client.read();
  }
    // Allocate the JSON document
  // Use arduinojson.org/v6/assistant to compute the capacity.
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(8) + 200;
  DynamicJsonDocument doc(capacity);

  // Parse JSON object
  DeserializationError error = deserializeJson(doc, client);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
   return;
  }
   // Extract values
  JsonObject root_0 = doc[0];
  Serial.println(F("Response:"));
  const char* root_0_text = root_0["text"];
  const char* root_0_author = root_0["author"];
  
  Serial.print("Quote: "); Serial.println(root_0_text);
  Serial.print("Author: "); Serial.println(root_0_author);
  //quote_text=  root_0_text;
  strcpy(quote_text, root_0_text);
  //Draw something here!

  Quote_text_draw(root_0_text,EPD_RED);
  display.display();
  wifi_sucess_cnt++;
  while (client.available() > 0)
  {
    //read back one line from the server
    String line = client.readStringUntil('\r');
    Serial.println(line);
  }
  
  // disconnect
  client.stop();
  // Turn off WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.println("WiFi turned off");
}
