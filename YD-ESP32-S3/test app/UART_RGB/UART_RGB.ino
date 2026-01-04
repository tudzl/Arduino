/*
  BlinkRGB

  Demonstrates usage of onboard RGB LED on some ESP dev boards.

  Calling digitalWrite(RGB_BUILTIN, HIGH) will use hidden RGB driver.
    
  RGBLedWrite demonstrates controll of each channel:
  void neopixelWrite(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val)

  WARNING: After using digitalWrite to drive RGB LED it will be impossible to drive the same pin
    with normal HIGH/LOW level
*/
//#define RGB_BRIGHTNESS 64 // Change white brightness (max 255)

// the setup function runs once when you press reset or power the board
#include <Adafruit_NeoPixel.h>

#define LED_PIN    48

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 1

// Declare our NeoPixel Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel YD_WS2812LED(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


uint32_t chipId = 0;
uint32_t sys_cnt = 0;

void setup() {
  	Serial.begin(115200);
    delay(500);
    Serial.println("ESP32 S3 test code by Zell");

    YD_WS2812LED.begin();
    YD_WS2812LED.show(); // Initialize all pixels to 'off'

  // No need to initialize the RGB LED
  	for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}

	Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
	Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: "); Serial.println(chipId);
}

// the loop function runs over and over again forever
void loop() {
  sys_cnt++;
#ifdef RGB_BUILTIN
  // digitalWrite(RGB_BUILTIN, HIGH);   // Turn the RGB LED white
  // delay(1000);
  // digitalWrite(RGB_BUILTIN, LOW);    // Turn the RGB LED off
  // delay(1000);
  //strip.setPixelColor(n, red, green, blue);
  YD_WS2812LED.setPixelColor(0, 0, 0, 255);
  YD_WS2812LED.show();
  Serial.printf("WS2812 LED color Blue\n");
  //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
  delay(1000);
  YD_WS2812LED.setPixelColor(0, 0, 255, 0);
  YD_WS2812LED.show();
  Serial.printf("WS2812 LED color Green loop...%u\n", sys_cnt);
  delay(1000);
  // neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
  // delay(1000);
  // neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue
  // delay(1000);
  // neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
  // delay(1000);
#endif
}
