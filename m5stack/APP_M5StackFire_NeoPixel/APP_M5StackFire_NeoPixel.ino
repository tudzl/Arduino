/************************************************************************

  M5StackFire BlackOut Help

  Here in this region we have a BlackOut.

  Fortunatelly I have the M5StackFire with a little spare energie in its
  batttery. With this I can illumnate my room ...

  4.October 2018, ChrisMicro, Swizerland, close to Zürich

  note: need add library Adafruit_NeoPixel from library manage


************************************************************************/


//for use as bin app lovyan03
#include <M5StackUpdater.h> // https://github.com/tobozo/M5Stack-SD-Updater/
#include <M5Stack.h>

#define USENEOLED

#ifdef USENEOLED
#include <Adafruit_NeoPixel.h>

#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);
#endif

#define DELAYVAL 200

POWER m5_power;
unsigned long run_cnt = 0;
//Brightness
uint8_t Brightness = 255;

void setup()
{
  M5.Lcd.begin();
  M5.Lcd.setBrightness(20);  //define BLK_PWM_CHANNEL 7  PWM
  M5.Lcd.fillScreen( WHITE );
  M5.Lcd.setTextSize(3);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.println("M5stack NeoPixel");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(BLUE,WHITE);
  M5.Lcd.println("V1.1 DEZ.2019");
  Serial.println(F("<<<M5stack NeoPixel test>>>"));
  Serial.println(F("<<<Firmware Version 1.1, ling zhou, 26.12.2019>>>"));
  //APP flash back
  if (digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS(SD);
    ESP.restart();
  }

#ifdef USENEOLED
  pixels.begin();
  int r = 240;
  int g = 240;
  int b = 200;
  for (int n = 0; n < 10; n++)pixels.setPixelColor(n, pixels.Color(r, g, b));
  pixels.show();
#endif
  m5_power.begin();
}

void loop(void)
{

  int r = 255;
  int g = 240;
  int b = 200;
  for (int n = 0; n < 10; n++)
    pixels.setPixelColor(n, pixels.Color(r, g, b));

  //change  r,g,b,setBrightness
  if (Brightness < 10) {
    Brightness = 255;
  }
  pixels.setBrightness(BR);
  pixels.show();
  
  M5.Lcd.setCursor(120, 120);
  M5.Lcd.printf("BR: %3d% ", Brightness);
  Brightness -= 5;



  //-------sys status GUI-------------------------------------
  int Akku_level = m5_power.getBatteryLevel();
  M5.Lcd.setCursor(0, 220);
  M5.Lcd.setTextSize(2); //size 2 to 8
  M5.Lcd.setTextColor(ORANGE, WHITE);
  M5.Lcd.printf("Akku: %3d%%   Run:%d ", Akku_level, run_cnt);

  M5.update(); // This function reads The State of Button A and B and C.
  delay(DELAYVAL);
  run_cnt++;
}
