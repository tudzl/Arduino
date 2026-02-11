//ESP32 test code for coremark and IIC plus BMP280 AHT20 sensors
//Sketch uses 368232 bytes (11%) of program storage space. Maximum is 3145728 bytes.
//Global variables use 14760 bytes (4%) of dynamic memory, leaving 312920 bytes for local variables. Maximum is 327680 bytes.

//V2.12 created MCP9804 lib base on 9808 lib, added 9804 sensor routines, need test! seems working
//V2.11 added Alt calc functions, need test
//V2.10 added AHT20 sensor,
//V2.02 found issue SDA pin8 conflict with ws2812 pin!!! need to find a solution
//V2.01 added basic BMP280 code, fixed Coremark compiling issue: NULL define
//#define NULL nullptr
//但请注意，nullptr是C++11的关键字，确保你的编译器支持。
//步骤2：如果修改后仍然有错误，或者你希望使用标准的NULL，那么可以考虑完全删除这一行定义，因为Arduino环境通常已经在stddef.h或cstddef中定义了NULL。
//V2.0 by Zell
//2.Feb.2026

#include <Arduino.h>
#include <core_arduino.h>
#include <FastLED.h>
#include <coremark.h>
#include <Wire.h>
#include "IIC_scan.h"
#include "Alt_calc.h"

#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_MCP9804.h>
//#include <Adafruit_MCP9804.h>

//

//21:53:07.495 -> I2C device found at address 0x38
//21:53:07.538 -> I2C device found at address 0x77

#define Core_mark_test_EN 0

#define ESP32_sleep_interval 5000  //in ms 5s
//versions
#define Version_Nrd '2'
#define Version_Nrf1 '1'  //2._x
#define Version_Nrf2 '2'  //2.x_


//WS2812 LED
//Pixel LED defines
#define NUM_LEDS 1
#define LED_DATA_PIN 8  //GPIO8 on C3 devkit wroom n4r8, conflict to IIC SDA
#define BRIGHTNESS 32
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
uint8_t LED_brightness = 10;
float CPU_Speed = 160;  //Mhz
float CM_score = 0;
float score_per_Mhz = 0;
extern core_results results[1];
extern CORE_TICKS total_time;

Adafruit_BMP280 bmp280;  // use I2C interface, address 0x77
uint8_t bmp280_sensor_exist = 0;
Adafruit_Sensor *bmp280_temp = bmp280.getTemperatureSensor();
Adafruit_Sensor *bmp280_pressure = bmp280.getPressureSensor();

Adafruit_AHTX0 AHT20;  //address 0x38
uint8_t AHT20_sensor_exist = 0;

//MCP9804. address 0x18
Adafruit_MCP9804  MCP9804_sensor;
uint8_t MCP9804_sensor_exist = 0;
//
#define AltitudeInfo_EN 1
// 实际应用中建议根据当地气象站数据校准
/*
#define SEA_LEVEL_PRESSURE_HZ 1012.5  // 杭州平均海平面气压 (hPa)
#define TEMPERATURE_LAPSE_RATE 0.0065  // 温度递减率 (K/m)
#define GRAVITY 9.80665  // 重力加速度 (m/s^2)
#define MOLAR_MASS 0.0289644  // 干空气摩尔质量 (kg/mol)
#define UNIVERSAL_GAS_CONSTANT 8.314462618  // 通用气体常数 (J/(mol·K))
#define GAS_CONSTANT_DRY_AIR 287.058  // 干空气气体常数 (J/(kg·K))



float calculateAltitude(float pressure, float temperature = 15.0, float seaLevelPressure = SEA_LEVEL_PRESSURE_HZ);
float calculateAltitudeBarometric(float pressure, float seaLevelPressure = SEA_LEVEL_PRESSURE_HZ);
float calculateAltitudeHypsometric(float pressure, float temperature, float seaLevelPressure = SEA_LEVEL_PRESSURE_HZ);
float calculateAltitudeInternational(float pressure, float seaLevelPressure = SEA_LEVEL_PRESSURE_HZ);
void printAltitudeInfo(float pressure, float temperature);
*/

sensors_event_t temp_event, pressure_event;
sensors_event_t AHT20_humidity, AHT20_temp;
sensors_event_t MCP9804_temp;
char version_buf[4];

void setup() {
  Serial.begin(115200);
  version_buf[0] = Version_Nrd;
  version_buf[1] = '.';
  version_buf[2] = Version_Nrf1;
  version_buf[3] = Version_Nrf2;
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Orange;
  //FastLED.show();
  FastLED.setBrightness(LED_brightness);
  FastLED.show();
  Serial.println("ESP32C3 Arduino coremark+BMP280 sensor demo by Zell, 26.Jan.2026");
  Serial.printf("-Version %s, 06.Feb.2025 by Zell-\r\n", version_buf);
  Serial.println("-Demo based on core_arduino+Adafruit example, modified by Zell");
  //__DATE__ and __TIME__,
  Show_ESP32_sys_info();
  Serial.printf("-FW Compile time: %s, date: %s\r\n", __TIME__, __DATE__);

  Serial.println("Turn on Onboard WS2812 LED->Orange->Green");

  show_IIC_pins();
  show_SPI_pins();
  Serial.println(">:IIC init now...");
  delay(50);
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(50);
  Wire.begin(SDA, SCL);
  Serial.println(">:Scan IIC now...");
  IIC_scan();

  unsigned BMP_status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  BMP_status = bmp280.begin();
  check_BMP_sensor_status(BMP_status);
  if (BMP_status) {
    bmp280_sensor_exist = 1;
    /* Default settings from datasheet. */
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                       Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                       Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    bmp280_temp->printSensorDetails();
  }
  if (AHT20.begin()) {
    Serial.println(">:Found AHT20 ENV sensor!");
    AHT20_sensor_exist = 1;
  } else {
    Serial.println(">:Didn't find AHT20 sensor");
    AHT20_sensor_exist = 0;
  }

  if (!MCP9804_sensor.begin()) {
    Serial.println("Failed to find Adafruit MCP9808 chip");
    
  }
  else{
    Serial.println(">:Found MCP9804 temp sensor!");
     MCP9804_sensor_exist = 1;
     MCP9804_sensor.setResolution(3);
     Serial.println(">>:set resolution to 0.0625 ^C!");
  }
  


  delay(1000);
}

void loop() {
  //IIC_scan();
  Wire.begin(SDA, SCL);

   if (MCP9804_sensor_exist) {
    MCP9804_sensor.getEvent(&MCP9804_temp);
    Serial.print("\r\n##MCP9804 Temperature:");
  Serial.print(MCP9804_temp.temperature);
  Serial.println(" *C\r\n");
   }


  if (AHT20_sensor_exist) {
    Serial.println(">>:--Reading AHT20 sensor now:--\r\n");
    AHT20.getEvent(&AHT20_humidity, &AHT20_temp);  // populate temp and humidity objects with fresh data
    Serial.print(F("Temperature = "));
    Serial.print(AHT20_temp.temperature);
    Serial.println(" *C");
    //temp.temperature
    Serial.print(F("RH = "));
    Serial.print(AHT20_humidity.relative_humidity);
    Serial.println(" %");
    //humidity.relative_humidity
  }
  if (bmp280_sensor_exist) {

    bmp280_temp->getEvent(&temp_event);
    bmp280_pressure->getEvent(&pressure_event);
    Serial.println("\r\n>>:**Reading BMP280 sensor now:**\r\n");
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");

    Serial.println();
  }

  if ((bmp280_sensor_exist) && (AltitudeInfo_EN)) {
    printAltitudeInfo(pressure_event.pressure, temp_event.temperature);
  }

  Wire.end();
  pinMode(SCL, INPUT);
  digitalWrite(SCL, LOW);
  pinMode(LED_DATA_PIN, OUTPUT);
  //pinMode(LED_DATA_PIN, INPUT_PULLUP);
  //pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  //digitalWrite(LED_DATA_PIN, HIGH);
  //delay(50);
  //digitalWrite(LED_DATA_PIN, LOW);
  delay(50);
  //FastLED.clear();
  //FastLED.show();
  //FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  //FastLED.setBrightness(LED_brightness);
  //FastLED.clear();
  //digitalWrite(LED_DATA_PIN, LOW);
  //delay(50);
  leds[0] = CRGB::Red;
  //FastLED.show();
  delay(50);
  Serial.println(">:set LED: Red failed");
  delay(500);
  if (Core_mark_test_EN) {
    Serial.println(">>:startCoremark now..!\r\n");
  }
  if (Core_mark_test_EN) {
    leds[0] = CRGB::Red;
    FastLED.show();
    startCoremark();
    CM_score = results[0].iterations / time_in_secs(total_time);
    score_per_Mhz = CM_score / CPU_Speed;
    Serial.printf(">>:Coremark Finshed! Score: %4.2f/Mhz \r\n", score_per_Mhz);
  }

  //leds[0] = CRGB::Green;
  //FastLED.show();
  delay(2000);
  //leds[0] = CRGB::Blue;
  //FastLED.show();
  //delay(2000);
  //leds[0] = CRGB::Orange;
  //FastLED.show();
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

void Show_ESP32_sys_info(void) {

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
  Serial.print(getApbFrequency() / 1000000);
  Serial.println(" MHz");

  Serial.println("--ESP32 Memory and Storage--");
  Serial.printf("  Program Size: %.1f KB\n", ESP.getSketchSize() / 1024.0);
  Serial.printf("  Free Program Space: %.1f KB\n", ESP.getFreeSketchSpace() / 1024.0);
  Serial.printf("  Flash Size: %.1f MB\n", ESP.getFlashChipSize() / (1024.0 * 1024.0));
  Serial.printf("  Free Heap: %.1f KB\n", ESP.getFreeHeap() / 1024.0);
  #ifdef BOARD_HAS_PSRAM
  if (psramFound()) {
    heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
    Serial.printf("  PSRAM (External):\n");
    Serial.printf("    Total: %u bytes (%.1f MB)\n", 
                  heap_info.total_free_bytes + heap_info.total_allocated_bytes,
                  (heap_info.total_free_bytes + heap_info.total_allocated_bytes) / (1024.0 * 1024.0));
    Serial.printf("    Allocated: %u bytes (%.1f KB)\n", 
                  heap_info.total_allocated_bytes,
                  heap_info.total_allocated_bytes / 1024.0);
    Serial.printf("    Free: %u bytes (%.1f KB)\n", 
                  heap_info.total_free_bytes,
                  heap_info.total_free_bytes / 1024.0);
  } else {
    Serial.println("  PSRAM: Not detected");
  }
#else
  Serial.println("  PSRAM: Not supported on this board");
#endif
}

void check_BMP_sensor_status(unsigned status) {

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp280.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");

  } else {
    Serial.print("!>>:BMP280 Found with SensorID : 0x");
    Serial.println(bmp280.sensorID(), 16);
    Serial.printf("-Hangzhou sea level pressure: %.1f hPa\r\n", SEA_LEVEL_PRESSURE_HZ);
  }
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