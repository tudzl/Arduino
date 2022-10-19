//将PC画面实时传送至M5Stack


#pragma GCC optimize ("O3")

 

#if defined(ARDUINO_M5Stack_Core_ESP32) || defined(ARDUINO_M5STACK_FIRE)

 

  #include <M5Stack.h>

  #include <M5StackUpdater.h>     // https://github.com/tobozo/M5Stack-SD-Updater/

  TFT_eSPI &Tft = M5.Lcd;

 

#elif defined(ARDUINO_M5Stick_C)

 

  #include <M5StickC.h>

  TFT_eSPI &Tft = M5.Lcd;

 

#else

 

  #include <TFT_eSPI.h>  // https://github.com/Bodmer/TFT_eSPI

 

  TFT_eSPI Tft;

 

#endif

 

#include <WiFi.h>

#include <esp_wifi.h>

 

#include "TCPReceiver.h"

#include "DMADrawer.h"

 

TCPReceiver recv;

 

const char* ssid     = "SSID";

const char* password = "PASSWORD";

 

void setup(void)

{

#if defined(ARDUINO_M5Stack_Core_ESP32) || defined(ARDUINO_M5STACK_FIRE) || defined(ARDUINO_M5Stick_C)

  M5.begin();

  #ifdef __M5STACKUPDATER_H

    if(digitalRead(BUTTON_A_PIN) == 0) {

       Serial.println("Will Load menu binary");

       updateFromFS(SD);

       ESP.restart();

    }

  #endif

#else

  Serial.begin(115200);

  Serial.flush();

  Tft.begin();

 

  #ifdef TFT_BL

    if (TFT_BL >= 0) {

      ledcAttachPin(TFT_BL, 1);

      ledcSetup(1, 12000, 8);

      ledcWrite(1, 128);

    }

  #endif

#endif

 

  Tft.setRotation(0);

  if (Tft.width() < Tft.height())

    Tft.setRotation(1);

 

  int width  = Tft.width();

  int height = Tft.height();

  if (width  > 320) width  = 320;

  if (height > 240) height = 240;

 

  Tft.setTextFont(2);

 

  Serial.println("WiFi begin.");

  Tft.println("WiFi begin.");

 

  WiFi.mode(WIFI_MODE_STA);

  WiFi.begin(ssid, password);

 

 

  for (int i = 0; WiFi.status() != WL_CONNECTED && i < 100; i++) { delay(100); }

 

 

  // https://itunes.apple.com/app/id1071176700

  // https://play.google.com/store/ap ... erclub.iot.esptouch

  if (WiFi.status() != WL_CONNECTED) {

    Serial.print("SmartConfig start.");

    Tft.println("SmartConfig start.");

    WiFi.mode(WIFI_MODE_APSTA);

    WiFi.beginSmartConfig();

 

    while (WiFi.status() != WL_CONNECTED) {

      delay(100);

    }

    WiFi.stopSmartConfig();

    WiFi.mode(WIFI_MODE_STA);

  }

 

  Serial.println(String("IP:") + WiFi.localIP().toString());

  Tft.println(WiFi.localIP().toString());

 

  setup_t s;

  Tft.getSetup(s);

 

  int spi_freq = SPI_FREQUENCY;

 

//  if (spi_freq > 40000000)  spi_freq = 40000000;

 

  recv.setup( s.r0_x_offset

            , s.r0_y_offset

            , width

            , height

            , spi_freq

            , TFT_MOSI

            , TFT_MISO

            , TFT_SCLK

            , TFT_CS

            , TFT_DC

            );

}

 

void loop(void)

{

  recv.loop();

}