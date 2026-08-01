//ESP32 ENV sensor test demo by Zell, 2026

/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
//measured current consumption in light sleep: 6.3mA, LED off
//ESP on 25-32mA, AMS1117 and WCH340 may draw currents!
//peak current 170mA! with co2 sensor heating(70mA?) 
//need to add one external Temp. sensor for accurate T measure!
//https://javl.github.io/image2cpp/

//V1.39 claude opt of sensor priorities
//V1.38 add BLE braodcast for BMP580, Opt sensor priorities, need test
//V1.37 add BMP580, tested ok with SDO open, IIC address 0x47, improved GUI draw decline icon
//V1.36 修复：
  //- ble_rx_dis：改用 BLE_sensor_receive_stop()（只停止扫描）代替 _deinit()
  //- ble_tx_dis：改用 BLE_sensor_broadcast_stop()（只停止广播）代替 _deinit()
//V1.35 在 display.display() 之前添加了两个红色小圆点（半径 2px）：
  //- SHT40 存在 → 在 (228, 118) 画红点（右下角，WiFi 图标左侧）
  //- MCP9804 存在 → 在 (222, 118) 画红点（SHT40 点左侧再偏 6px）
//V1.34 12.June, OPT of 3 sensor defines, need real HW test!
//V1.33 11.June, added SHT40 sensor, need test! ref: https://github.com/MR01Right/SHT40/blob/main/examples/Basic/Basic.ino
//V1.32 30.May.26, BLE RX: active scan for scanResp, decode both MfrData+ScanResp(UUID 0xFC90), RSSI, stats
//V1.31 29.May.26，OPT BLE， To do：added scan mode decode
//V1.30 15.May.2025, added BLE Receive Feature, need test!

//V1.29 06.May.2026: (need test)
//  1. All Serial.print* replaced with esp_log: ESP_LOGI for boot/sensor/WiFi/EPD events, ESP_LOGD for debug/timing/stats
//     Added: #include "esp_log.h", static TAG="CO2_APP". Serial.begin kept for esp_log UART output
//  2. Added MCP9804 external temp sensor (I2C 0x18, Adafruit_MCP9804, resolution 0.0625C)
//     Init after Wire.begin(), read every loop cycle, stored in current_temp_MCP9804
//  3. Added JSON output every 60s (configurable: #define JSON_OUTPUT_INTERVAL_S 60)
//     Uses ArduinoJson, outputs: co2_ppm/temp_scd40/temp_mcp9804/humidity/co2_avg/co2_max/co2_min/uptime_s
//     JSON line sent via raw Serial.println (no log prefix) for machine parsing
//  4. EPD_temperature_define.h: Serial calls also converted to ESP_LOGD
//V1.28 added rise decline wifi icons, improved GUI and history calc functions
//V1.27B found boot pin always low! added debug code  digitalWrite(Boot_pin,0) in loop,need test.

//V1.27 added CO2 average icon,need test
//V1.26 updated icon images,need test
//V1.25 improved image draw
//V1.24 added EPD_darw image function, test ok with min max icons!
//V1.23 added EPD_darw_title_loop_EN（def 0) to improve GUI
//V1.22 added RTC_DATA_ATTR to sys_cnt to fix >>:544175780th loop running! bug
 //V1.21 optimization sleep related routines, turn off ws2812 during sleep
 //V1.20 fixed a sys_cnt error bug, added delay after wake up solved the issue. sllep time 10s
 //V1.19 improved co2 ave max min display to single line
//V1.18 try fixing Temp. min record bugs, done! also improved sys on time accuracy
 //V1.17 added EEPROM, additional temperature history vars
//V1.16 opt of GUI, added ppm show
 //V1.15 added and improved EPD detection function, OPT of GUI display of time and date
//V1.14 added NTP get time, need test, seems working!
//V1.13 added timeupdate function, fixed time show bug
//found bug: Serial.printf(">>: %u hour,%lu s! %u s\r\n" always show 0, seems related to uint64_t
//V1.12 added WiFi get Quote test function, test works! peak current 148mA
//V1.11 added some fonts, opt for display
//V1.10 fixed  EPD seconds show bug, 17.Jan.2026
//V1.09 trying to to add debug info for EPD seconds show bug,
//V1.08 fixed EPD seconds show bug, failed
//V1.07 opt of sleep, turn off LED during sleep, Active run 45mA, sleep <8mA
//V1.06 added light sleep to save power, test works!
//V1.05 added CO2 history data and average data show
//V1.04 added EPD CO2 display, added Min Max display
//V1.03 SCD40 detected, working, every 1s updates
//V1.02 modified pin to fit IIC pins for SCD40,
//V1.01. basic EPD working
//depend on Adafruit_EPD and FastLED and SparkFun_SCD4x_Arduino_Library

//use debug port for uploading FW!
#include <Arduino.h>
#include <esp_heap_caps.h>
#include "esp_log.h"
#include <ArduinoJson.h>
#include <Adafruit_MCP9804.h>
//#include "IIC_scan.h" //for debug
static const char *TAG = "CO2_APP";
static const char *TAG_BLE = "CO2_BLE";
static const char *TAG_SHT = "SHT_APP";
static const char *TAG_EPD = "EPD_GUI";

//#include <core_arduino.h> //compile error
#include "Adafruit_EPD.h"
#include <Adafruit_Sensor.h>
#include <Fonts/FreeMonoBold9pt7b.h>  //has a height of about 18 pixels,
#include <Fonts/FreeMono9pt7b.h>
//#include <Fonts/FreeMono9pt7b.h> //
//#include <Fonts/FreeSans12pt7b.h> //
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Fonts/FreeSerif12pt7b.h> 
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/Picopixel.h>
#include <Fonts/FreeSerifItalic9pt7b.h>

#include "image_icon.h"
#include "EPD_GUI_head.h"
#include <FastLED.h>

#include <WiFi.h>
#include "time.h"
//#include <TimeLib.h>
#include "esp_sntp.h"

#include <Wire.h>

#include <Benchmark.h>

//sensor lib
#include "SparkFun_SCD4x_Arduino_Library.h"  //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x
#define SHT40Sensor_EN 1  // 1=enable SHT40 support, 0=disable
#define BMP580Sensor_EN 1 //1=enable BMP580 Sensor support, 0=disable
#define SEALEVELPRESSURE_HPA (1013.25)
float sea_level_pressure_hpa = SEALEVELPRESSURE_HPA;  // runtime value, updated from network

//versions
#define Version_Nrd '1'
#define Version_Nrf1 '3'  //1._x
#define Version_Nrf2 '9'  //1.x_

SCD4x SCD40Sensor;        
#if SHT40Sensor_EN
#include <SHT40.h>. //need MRORight not 7Semi
SHT40 sht40;
#endif


#if BMP580Sensor_EN
#include "Adafruit_BMP5xx.h"

Adafruit_BMP5xx bmp580; // Create BMP5xx object
// Get separate sensor objects for temperature and pressure
Adafruit_Sensor *bmp_temp = NULL;
Adafruit_Sensor *bmp_pressure = NULL;
void BMP580_setting_readback_helper(void);
void BMP580_read_value_test(void);

float BMP580_Temperature_cur;
float BMP580_Pressure_cur;

#endif


#include "Net_config.h"
#include "EPD_temperature_define.h"
#include <EEPROM.h>
#define EEPROM_SIZE 512
#define BOARD_HAS_PSRAM
#include "esp_sleep.h"  // 提供 RTC_DATA_ATTR
#include "driver/gpio.h"
#include <Preferences.h>

// UART RX GPIO wakeup from light sleep
#define UART_RX_PIN GPIO_NUM_44       // ESP32-S3 UART0 RX pin
#define UART_CMD_WINDOW_MS 3000       // CMD window after UART wakeup (ms)

// ========== BLE Receive Feature ==========
#define BLE_SENSOR_RECEIVE_EN 1  // 1=enable BLE sensor receive, 0=disable
//BLE_sensor_data_t BLE_RX_latest
#if BLE_SENSOR_RECEIVE_EN
#include "BLE_sensor_receive.h"
bool BLE_receive_EN = false;
uint32_t BLE_receive_interval_ms = 60000;  // default 60s, range 500~600000ms
unsigned long last_BLE_receive_ms = 0;
Preferences BLE_prefs;
#endif

// ========== BLE Broadcast Feature ==========
#define BLE_SENSOR_BROADCAST_EN 1  // 1=enable BLE CO2 broadcast, 0=disable
#if BLE_SENSOR_BROADCAST_EN
#include "BLE_sensor_broadcast.h"
bool BLE_broadcast_EN = true;
uint32_t BLE_broadcast_interval_ms = 5000;  // default 30s
unsigned long last_BLE_broadcast_ms = 0;
Preferences BLE_TX_prefs;
#endif

         //The default I2C address for the SCD4x is 0x62.
#define ESP32_sleep_interval 10000  //in ms 5s



#define EPD_darw_title_loop_EN 1

#define EPD_darw_icon_test_EN 1
#define EPD_Boot_darw_icon_test_EN 0
#define EPD_Boot_darw_device_icon_EN 1
#define Init_test_EPD_EN 0
#define SCD40Sensor_EN 1   //Co2 sensor

#define Loop_perodic_update_EN 1
#define Loop_WiFi_perodic_update_EN 1
#define WiFi_refresh_interval 5
uint16_t WiFi_refresh_interval_tmp = WiFi_refresh_interval;
uint8_t WiFi_connect_status = 0;
#define Sensor_temperature_history_record_EN 1

#define EEPROM_save_EN 0
#define EPD_refresh_interval 30                    //30
#define EEPROM_save_interval 1500
uint32_t sleepTime = ESP32_sleep_interval * 1000;  // Sleep duration in microseconds (1 seconds)
RTC_DATA_ATTR uint32_t sys_on_time_second = 0;
RTC_DATA_ATTR uint32_t sys_active_time_second = 0;
uint32_t sys_sleep_time_second = 0;
RTC_DATA_ATTR uint32_t sys_on_time_milis = 0;
RTC_DATA_ATTR uint32_t sys_on_time_hour = 0;
RTC_DATA_ATTR uint32_t sys_on_time_second_tmp = 0;
uint32_t sleepTime_seconds = ESP32_sleep_interval / 1000;
#define EPD_display_refresh_time 22  //22s
float ESP32_active_ratio = 0; //sys_active_time_second/sys_on_time_second
RTC_DATA_ATTR uint32_t EPD_refresh_cnt = 0;


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
#define Boot_pin 0  //Low wirh SCK, OK! Use this pin for EPD CS1

#include "EPD_detect.h"
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
#define EPD_Driver_IC "SSD1680"
#define EPD_Driver_IC2 "SSD1675"
//#define EPD_height 104

//https://docs.arduino.cc/libraries/adafruit-epd/
// Uncomment the following line if you are using 1.54" EPD with IL0373

//Uncomment the following line if you are using 2.13" EPD with SSD1680, HINK 2.13 seems not very fit,Top bar crashed pixels
Adafruit_SSD1680 display(EPD_width, EPD_height, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
//Adafruit_SSD1680 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SPI_CS, EPD_BUSY, EPD_SPI); //tested partialy working
//SSD1680：较新的驱动芯片，支持更高的分辨率和更多的功能。例如，我们使用的2.13英寸三色屏（250x122）就是使用SSD1680驱动。它支持局部刷新和更灵活的波形控制。

// Uncomment the following line if you are using 2.13" EPD with SSD1675,HINK 2.13 seems resolution fit,Top bar no issue but no RED color
//Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,EPD_BUSY, EPD_SPI); //not working for TElink0213 A002!
//Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SPI_CS,EPD_BUSY, EPD_SPI); //not working!

// Uncomment the following line if you are using 2.13" EPD with SSD1675B, not fit HINK 2.13 , random pixels!
//Adafruit_SSD1675B display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);//not working!not working for TElink0213 A002!
//SSD1675B：SSD1675的改进版本，可能有一些优化，比如降低功耗、简化初始化序列等。同样支持三色（黑白红）和双色（黑白）显示。

// Uncomment the following line if you are using 2.13" EPD with UC8151D, not fit HINK 2.13 
//Adafruit_UC8151D display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);//not working!

// Uncomment the following line if you are using 2.13" EPD with IL0373， not fit  HINK 2.13 
//Adafruit_IL0373 display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
//#define FLEXIBLE_213

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DEBUG_FLUSH() Serial.flush()

#include "IIC_scan.h"
uint8_t SCD40Sensor_exist = 0;
uint8_t SHT40Sensor_exist = 0;
uint8_t BMP580Sensor_exist = 0;
uint16_t CO2_cur = 0;
uint16_t CO2_Min = 0;
uint16_t CO2_Max = 0;
uint16_t CO2_ave = 0;
uint16_t CO2_previous = 0;
#define CO2_history_ave_size 60
uint16_t CO2_history[CO2_history_ave_size]; //judge from this to show epd_bitmap_rising_icon
uint16_t CO2_data_idx = 0;
uint16_t CO2_array_write_idx = 0;
uint32_t CO2_data_cnt = 0;
float Temperature_cur = 0;
float Humidity_cur = 0;
uint16_t altitude = 100;

// MCP9804 external temperature sensor (I2C address 0x18)
Adafruit_MCP9804 MCP9804_sensor;
uint8_t MCP9804_sensor_exist = 0;
float current_temp_MCP9804 = NAN;

#if SHT40Sensor_EN
float SHT40_Temperature_cur = 0;
float SHT40_Humidity_cur = 0;
#endif

// Display values resolved by priority: SHT40 > MCP9804 > SCD40
float Display_Temperature = 0;
float Display_Humidity = 0;
uint8_t Display_Temp_Source = 0; // 0=SCD40, 1=MCP9804, 2=SHT40

// JSON output interval (seconds)
#define JSON_OUTPUT_INTERVAL_S 30
unsigned long lastJsonOutput = 0; //time stamp
bool json_output_EN = true; // controlled by UART CMD: json_en / json_dis

uint16_t pixel_pos_x, pixel_pos_y;
char text_buf[128];



//WS2812 LED
CRGB leds[NUM_LEDS];
uint8_t LED_brightness = 20;

void show_SPI_pins(void);
void show_IIC_pins(void);
void Show_ESP32_sys_info(void);
#if BLE_SENSOR_RECEIVE_EN
void handleSerialCommands(void);
#endif
RTC_DATA_ATTR uint32_t sys_cnt = 0;
char version_buf[4];
uint16_t wifi_sucess_cnt = 0;
char quote_text[100];
char timeDisplayStr[30];
extern struct tm  timeinfo;

void setup() {
  Serial.begin(115200);

  version_buf[0] = Version_Nrd;
  version_buf[1] = '.';
  version_buf[2] = Version_Nrf1;
  version_buf[3] = Version_Nrf2;
  delay(1000);
  //make core debug level INFO to see the following info !
  Serial.println(F("<Adafruit 2.13' ESP32S3 ENV sensor Node with BW+R EPD SCD40 by Zell 2026>"));
  ESP_LOGI(TAG, "Adafruit 2.13' BW+R EPD SCD40 sensor test by Zell 2026");
  ESP_LOGI(TAG, "Version %s, 19.June.2026 by Zell", version_buf);
  ESP_LOGI(TAG, "FW Compile time: %s, date: %s", __TIME__, __DATE__);
  Show_ESP32_sys_info();
  ESP_LOGI(TAG, "---Code Configuration---");
  ESP_LOGD(TAG, "Init_test_EPD_EN %u; SCD40Sensor_EN %u; Loop_perodic_update_EN %u", Init_test_EPD_EN, SCD40Sensor_EN, Loop_perodic_update_EN);
  ESP_LOGD(TAG, "EPD_width %u; EPD_height %u", EPD_width, EPD_height);
  ESP_LOGD(TAG, "EPD_Driver_IC: %s", EPD_Driver_IC);
  ESP_LOGI(TAG, "Sleep interval:%u s, EPD update interval:%u s", sleepTime_seconds, EPD_refresh_interval*sleepTime_seconds);
  show_SPI_pins();
  show_IIC_pins();
  ESP_LOGD(TAG, "Turn on Onboard WS2812 LED");
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Green;
  FastLED.setBrightness(LED_brightness);
  FastLED.show();
  ESP_LOGI(TAG, "Init IIC...");
  Serial.println(F("[IIC]>>:Init now..."));
  Wire.begin();
  delay(50);
  IIC_scan();


  ESP_LOGI(TAG, "Init SPI...");
  ESP_LOGI(TAG, "Init EPD...");
  delay(50);
  detectEPDChip();
  display.begin(true);

#if defined(FLEXIBLE_213) || defined(FLEXIBLE_290)
  display.setBlackBuffer(1, false);
  display.setColorBuffer(1, false);
  ESP_LOGD(TAG, "display.setColorBuffer for flexible screen!");
#endif
  /*
  // large block of text
  display.clearBuffer();
  ESP_LOGD(TAG, "Draw texts...");
  testdrawtext(
      "ZELL EPD test,1.Jan.2026.Curabitur ",
      COLOR1);
  display.display();
  ESP_LOGD(TAG, "Waiting for EPD refresh...");
  delay(8000);
  */

  if (SCD40Sensor_EN) {

    if (SCD40Sensor.begin() == false) {
      ESP_LOGI(TAG, "SCD40 Sensor not detected. Please check wiring...");
    } else {
      ESP_LOGI(TAG, "[IIC]>>:SCD40 Sensor detected!");
      SCD40Sensor_exist = 1;
      if (SCD40Sensor.stopPeriodicMeasurement() == true) {
        ESP_LOGI(TAG, "SCD40 Periodic measurement is disabled!");
      }
      if (SCD40Sensor.startLowPowerPeriodicMeasurement() == true) {
        ESP_LOGI(TAG, "SCD40 Low power mode enabled!");
      }
      ESP_LOGI(TAG, "SCD40 setSensorAltitude:%u!", altitude);
      SCD40Sensor.setSensorAltitude(altitude, 10);
    }

    // MCP9804 external temperature sensor init
    if (!MCP9804_sensor.begin()) {
      ESP_LOGI(TAG, "MCP9804 sensor not detected (addr 0x18)");
    } else {
      ESP_LOGI(TAG, "[IIC]>>:MCP9804 sensor detected! Setting resolution to 0.0625C");
      MCP9804_sensor_exist = 1;
      MCP9804_sensor.setResolution(3);
    }

#if SHT40Sensor_EN
    if (!sht40.begin(&Wire)) {
      ESP_LOGI(TAG, "SHT40 sensor not detected!!!");
    }
    else {
      ESP_LOGI(TAG, "[IIC]>>:SHT40 initialized successfully");
      ESP_LOGI(TAG, ">>:Will use SHT40 as source for Temperature & Humidity sensing!");
      SHT40Sensor_exist = 1;
    }
#endif

#if BMP580Sensor_EN
  //IF CS open, SDO =GND,  BMP5XX_DEFAULT_ADDRESS is (0x46); SDO open,BMP5XX_ALTERNATIVE_ADDRESS 0x47
    if (bmp580.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
      BMP580Sensor_exist =1;
     Serial.println(F("[IIC]>>:BMP580 sensor found @ BMP5XX_DEFAULT_ADDRESS 0x46!"));
    }
    else {
     if (bmp580.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
       BMP580Sensor_exist =1;
       Serial.println(F("[IIC]>>:BMP580 sensor found @ BMP5XX_ALTERNATIVE_ADDRESS 0x47!"));
      }
    }
    if (BMP580Sensor_exist){
  // For SPI mode (uncomment the line below and comment out the I2C line above):
  // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
      // Get the unified sensor objects
    bmp_temp = bmp580.getTemperatureSensor();
    bmp_pressure = bmp580.getPressureSensor();
    Serial.println(F("[IIC]>>:BMP5xx sensor found!"));
    ESP_LOGI(TAG, ">>:BMP580 sensor initialized successfully, setting now...");
      Serial.println(F("=== Setting Up Sensor Configuration ==="));
  
  /* Temperature Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
   */
  Serial.println(F("Setting temperature oversampling to 2X..."));
  bmp580.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  /* Pressure Oversampling Settings (same options as temperature):
   * Higher oversampling = better accuracy but slower readings
   * Recommended: 16X for good balance of speed/accuracy
   */
  Serial.println(F("Setting pressure oversampling to 64X..."));
  bmp580.setPressureOversampling(BMP5XX_OVERSAMPLING_64X);
    /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
   */
  Serial.println(F("Setting IIR filter to coefficient 3..."));
  bmp580.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
   Serial.println(F("Setting output data rate to 2 Hz..."));
  //bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  //BMP5XX_ODR_02_HZ
  bmp580.setOutputDataRate(BMP5XX_ODR_02_HZ);
  bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode
   bmp580.setPowerMode(desiredMode);
   bmp580.enablePressure(true);
     Serial.println(F("Configuring interrupt pin with data ready source..."));
  bmp580.configureInterrupt(BMP5XX_INTERRUPT_PULSED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);
  Serial.println();
  Serial.println(F("=== Current Sensor Configuration ==="));
  BMP580_setting_readback_helper();
  }
  else  {
     ESP_LOGI(TAG, "BMP5xx sensor sensor not detected!!!");
     Serial.println(F("\r\n[IIC]>>:BMP580 sensor sensor not detected!!!\r\n"));
    //Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
  }
#endif
  }

  if(Sensor_temperature_history_record_EN){
    ESP_LOGI(TAG, "Temperature_history_record enabled! Initialize now!");
        // Initialize temperature history variables
    resetTemperatureStats();
    resetDailyTemperatureStats();
    
    // Initialize circular buffer
    memset(temperature_rolling_history, 0, sizeof(temperature_rolling_history));
    
    // Set initial daily reset time
    last_daily_reset_time = millis();
  }

  display.clearBuffer();
  if (Init_test_EPD_EN) {
    ESP_LOGD(TAG, "Draw test lines...");
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

  ESP_LOGD(TAG, "Waiting for EPD refresh...");
  display.display();
  delay(3000);
  //display.display();

  // large block of text
  display.clearBuffer();
  ESP_LOGD(TAG, "Draw texts...");
  //test only
  display.setTextSize(1);
  /*
  testdrawtext(
    "ZELL EPD test,1.Mar.2026. Version 1.17 ! "
    "2.13 BW+Red HINK E-INK Display,250*122,  Driver Unknown! SSD1680?",
    COLOR1);
  */

  sprintf(text_buf, "SCD40 sensor EPD APP V%s by ZELL 2026.\r\n2.13 BW+Red HINK E-INK ,250*122, Driver Unknown! SSD1680?", version_buf);  
  testdrawtext(text_buf,COLOR1);

  //
  //display.setFont(&Picopixel);//tiny
  //display.setFont(&FreeSerifItalic9pt7b);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("A", 0, 20, &x1, &y1, &w, &h);
  ESP_LOGD(TAG, "Default Font Width:%u Height:%u", w, h);
  /*
  Create a GFXcanvas1 object (an offscreen bitmap) for a fixed-size area, draw custom text in there and copy to the screen using drawBitmap().*/
  display.setFont(&FreeSerifItalic9pt7b);
  display.setTextSize(1);
  if(1==BMP580Sensor_exist){

  testdrawtext2(
    "Get as much education as you can. Nobody can take that away from you! BMP580!",
    COLOR2);
  }
  else
    testdrawtext2(
    "Get as much education as you can. Nobody can take that away from you!",
    COLOR2);

  if(EPD_Boot_darw_icon_test_EN) {
     ESP_LOGD(TAG, "Draw image icons...");
    EPD_drawIcon(Icon_Max_X, Icon_Max_Y, 16, 16, maxIcon, EPD_BLACK); 
    EPD_drawIcon(Icon_Min_X, Icon_Min_Y, 16, 16, minIcon, EPD_BLACK); 
   // EPD_drawIcon_BG(Icon_Chart_X, Icon_Chart_Y, 64, 64, epd_bitmap_MAX_64, EPD_WHITE,EPD_RED); 
   EPD_drawIcon_BG(EPD_width-64, Icon_Temp_Chart_Y, 50, 60, epd_bitmap_TempMeter, EPD_WHITE,EPD_RED); //temp. ico
   // EPD_drawIcon_BG(Icon_Chart64_X, Icon_Chart64_Y, 64, 64, epd_bitmap_MAX_64, EPD_WHITE,EPD_BLACK); 

  }


  if(EPD_Boot_darw_device_icon_EN) {
    ESP_LOGD(TAG, "Draw CO2 device image icon...");

   EPD_drawIcon_BG(EPD_width-64, EPD_height-64, 64, 64, epd_bitmap_CO2_device_icon, EPD_WHITE,EPD_RED); //temp. ico


  }
  ESP_LOGD(TAG, "Waiting for EPD display refresh...");
  display.display();
  display.setFont();
  display.setTextSize(1);
  ESP_LOGI(TAG, "EPD boot refresh finished!");
  delay(500);
  ESP_LOGD(TAG, "Set Boot pin to high");
  pinMode(Boot_pin, OUTPUT);
  digitalWrite(Boot_pin,1);


  memset(CO2_history, 0, sizeof(CO2_history));

  //Serial.printf("Connecting to %s ", ssid);
  //WiFi.begin(ssid, password);
  // Sleep wakeup: timer + UART RX GPIO (configured each cycle in loop)
  ESP_LOGI(TAG, "Sleep interval: %u ms, UART wakeup on GPIO%d, CMD window: %u ms",
           ESP32_sleep_interval, UART_RX_PIN, UART_CMD_WINDOW_MS);

#if BLE_SENSOR_RECEIVE_EN
  BLE_prefs.begin("myapp", false);
  BLE_receive_EN = BLE_prefs.getBool("ble_rx_en", false);
  BLE_receive_interval_ms = BLE_prefs.getULong("ble_rx_intv", 60000);
  BLE_prefs.end();
  if (BLE_receive_interval_ms < 500) BLE_receive_interval_ms = 500;
  if (BLE_receive_interval_ms > 600000) BLE_receive_interval_ms = 600000;
  ESP_LOGI(TAG_BLE, "BLE receive: %s, interval: %lu ms", BLE_receive_EN ? "ON" : "OFF", BLE_receive_interval_ms);
  if (BLE_receive_EN) {
    ESP_LOGI(TAG_BLE, "BLE receiving is active!");
    BLE_sensor_receive_init();
  }
  ESP_LOGI(TAG_BLE, "UART CMD: ble_rx_en ble_rx_dis ble_rx_interval XXXX(ms) ble_rx_show ble_rx_history status");
#endif

#if BLE_SENSOR_BROADCAST_EN
  BLE_TX_prefs.begin("myapp", false);
  BLE_broadcast_EN = BLE_TX_prefs.getBool("ble_tx_en", true);
  BLE_broadcast_interval_ms = BLE_TX_prefs.getULong("ble_tx_intv", 5000);
  BLE_TX_prefs.end();
  if (BLE_broadcast_interval_ms < 1000) BLE_broadcast_interval_ms = 1000;
  if (BLE_broadcast_interval_ms > 600000) BLE_broadcast_interval_ms = 600000;
  ESP_LOGI(TAG_BLE, "BLE broadcast: %s, interval: %lu ms", BLE_broadcast_EN ? "ON" : "OFF", BLE_broadcast_interval_ms);
  if (BLE_broadcast_EN) {
    BLE_sensor_broadcast_init();
  }
  ESP_LOGI(TAG_BLE, "UART CMD: ble_tx_en ble_tx_dis ble_tx_interval XXXX(ms)");
#endif

  sys_cnt = 0;
}

void loop() {
  // don't do anything!
  sys_cnt++;
  color_change_idx++;
  ESP_LOGW(TAG, "%luth loop running!", sys_cnt);
  //Serial.printf(">>:sys_on_time_second %u s\r\n", sys_on_time_second);
  Temperature_stats.timestamp = millis();
  sys_on_time_second=Temperature_stats.timestamp/1000;
  
  sys_active_time_second=sys_on_time_second - sys_sleep_time_second;
  
  updateTimeDisplay();
  
  ESP_LOGW(TAG, "sys on time: %s", timeDisplayStr);
  ESP_LOGI(TAG, "CPU active ratio: %.1f%%", ESP32_active_ratio);
  sys_on_time_hour = sys_on_time_second / 3600;
  if (sys_on_time_hour > 0) {
    sys_on_time_second_tmp = sys_on_time_second % 3600;
    //  sys_on_time_second_tmp = sys_on_time_second - sys_on_time_hour*3600;
    ESP_LOGW(TAG, "sys on time: %u hour, %lu s", sys_on_time_hour, sys_on_time_second_tmp);
    //Serial.printf(">>: %u hour,%lu s! %lu s\r\n", sys_on_time_hour, sys_on_time_second_tmp,sys_on_time_second); //>>: 1 hour,0 s! 300 s @3900. debug only  1 hour,0 s! 424 s
    //llu:21:57:46.645 -> >>: 1 hour,124554055534 s! 4633050594506964992 s
  } 

  if (0== sys_cnt % 999){
    Show_ESP32_sys_info();
  }
  /*
  else {

    //sys_on_time_second_tmp = sys_on_time_second;
    //Serial.printf("#>:sys on time since last boot %lu s!\r\n", sys_on_time_second);
    //Temperature_stats.timestamp = millis();
    ESP_LOGD(TAG, "sys on time since last boot %lu s(from millis)", Temperature_stats.timestamp/1000);
    //sys_on_time_second=Temperature_stats.timestamp/1000;
  }
  */

  if (1 == color_change_idx)
    leds[0] = CRGB::Coral;
  else if (2 == color_change_idx)
    leds[0] = CRGB::Azure;
  else {
    leds[0] = CRGB::Green;
    color_change_idx = 0;
  }
  FastLED.show();

#if SHT40Sensor_EN
  if(SHT40Sensor_exist){
    SHT40_Temperature_cur = sht40.readTemperatureC();
    SHT40_Humidity_cur = sht40.readHumidityRH();
    if (isnan(SHT40_Temperature_cur) || isnan(SHT40_Humidity_cur)) {
      ESP_LOGE(TAG, "!>:Failed to read values from SHT40 sensor!");
    }
    ESP_LOGD(TAG, "SHT40 Temperature: %.2f C", SHT40_Temperature_cur);
    ESP_LOGD(TAG, "SHT40 Humidity: %.2f %%", SHT40_Humidity_cur);
  }
#endif

  // Read MCP9804 external temperature sensor
  if (MCP9804_sensor_exist) {
    sensors_event_t mcp_event;
    MCP9804_sensor.getEvent(&mcp_event);
    current_temp_MCP9804 = mcp_event.temperature;
    ESP_LOGD(TAG, "MCP9804 Temperature: %.2f C", current_temp_MCP9804);
  }

#if BMP580Sensor_EN
  // Check if new data is ready before reading; independent of SCD40 presence
  if (BMP580Sensor_exist) {
    if (true == bmp580.dataReady()) {
      BMP580_read_value_test(); //print sensor values
      BMP580_Temperature_cur = bmp580.temperature;
      BMP580_Pressure_cur = bmp580.pressure;
    }
  }
#endif

  if (SCD40Sensor_exist) {
    if (SCD40Sensor.readMeasurement())
    {
      CO2_cur = SCD40Sensor.getCO2();
      Temperature_cur = SCD40Sensor.getTemperature();
      Humidity_cur = SCD40Sensor.getHumidity();
      CO2_data_cnt++;
      ESP_LOGW(TAG, "CO2:%u ppm  T:%.1f C,  H:%.1f%%", CO2_cur, Temperature_cur, Humidity_cur);
      if (MCP9804_sensor_exist) {
        ESP_LOGW(TAG, "MCP9804 T:%.2f C", current_temp_MCP9804);
      }
       if (SHT40Sensor_exist) {
        ESP_LOGW(TAG, "SHT40 T:%.2f C, H:%.1f%%", SHT40_Temperature_cur,SHT40_Humidity_cur);
      }

      //need to improve the average， /* old codes */
      /*
      if (CO2_data_idx < CO2_history_ave_size) {
        CO2_history[CO2_data_idx] = CO2_cur;
        CO2_data_idx++;
      }
        */
      add_CO2_data(CO2_cur);
      //
      CO2_ave = get_CO2_average();
      uint32_t CO2_SUM_tmp = 0;
      /*
      for (uint16_t idx = 0; idx < CO2_history_ave_size; idx++) {

        CO2_SUM_tmp += CO2_history[idx];
      }
      if (CO2_data_cnt < CO2_history_ave_size) {
        CO2_ave = CO2_SUM_tmp / CO2_data_cnt;
      } else {
        CO2_ave = CO2_SUM_tmp / CO2_history_ave_size;
      }
      */
      ESP_LOGD(TAG, "Average CO2:%u ppm (total counts:%u)", CO2_ave, CO2_data_cnt);

      if (CO2_cur > CO2_Max) {
        CO2_Max = CO2_cur;
      }
      if (CO2_cur < CO2_Min) {
        CO2_Min = CO2_cur;
      }
    }
  }

  // Resolve display T/H by priority: SHT40 > MCP9804 > SCD40 (independent of SCD40 presence)
#if SHT40Sensor_EN
  if (SHT40Sensor_exist && !isnan(SHT40_Temperature_cur) && !isnan(SHT40_Humidity_cur)) {
    Display_Temperature = SHT40_Temperature_cur;
    Display_Humidity = SHT40_Humidity_cur;
    Display_Temp_Source = 2;
  } else
#endif
  if (MCP9804_sensor_exist && !isnan(current_temp_MCP9804)) {
    Display_Temperature = current_temp_MCP9804;
    Display_Humidity = Humidity_cur; // MCP9804 has no humidity, fall back to SCD40 humidity if present
    Display_Temp_Source = 1;
  } else if (SCD40Sensor_exist) {
    Display_Temperature = Temperature_cur;
    Display_Humidity = Humidity_cur;
    Display_Temp_Source = 0;
  }
  static const char* temp_source_names[] = {"SCD40", "MCP9804", "SHT40"};
  ESP_LOGI(TAG, "Display T:%.2f H:%.1f%% (src:%s)", Display_Temperature, Display_Humidity,
           temp_source_names[Display_Temp_Source]);

  ESP_LOGD(TAG, "UpdateTemperatureStats, count:%u", temperature_reading_count);
  updateTemperatureStats(Display_Temperature);
  temperature_reading_count++;
  if (0 == temperature_reading_count % 4) {
    ESP_LOGD(TAG, "Printing temperature stats...");
    printTemperatureStats();
  }

  if (1 == sys_cnt) {
    ESP_LOGI(TAG, "Draw EPD for first loop!");

    //EPD_draw_title_info(text_buf,COLOR1);

    //sys_on_time_second_tmp = sys_on_time_second;
    //-----------------
    sprintf(text_buf, ">:sys on time %u s!", sys_on_time_second);
    //sprintf(text_buf,">:sys on time %u hour,%lu s!",sys_on_time_hour,sys_on_time_second_tmp);
    //Serial.printf(">Debug>:%s \r\n", text_buf);
    EPD_draw_sys_on_time(text_buf, COLOR1);
    memset(text_buf, 0, sizeof(text_buf));
    sprintf(text_buf, "CO2:%u,T:%3.1f,H:%2.0f", CO2_cur, Display_Temperature, Display_Humidity);
    display.setTextSize(2);
    Sensor_info_draw_with_offset(text_buf, COLOR2,-8);

    display.display();
    EPD_refresh_cnt++;
    //--------------------
    //EPD_display_refresh_cnt
    ESP_LOGW(TAG, "First EPD sensor refresh finished!");
    delay(500);
    if (SCD40Sensor_exist) {
      CO2_Min = CO2_cur;
    }
    //Serial.println(">>:System benchmark now!");
    //benchBegin();
    //delay(1);
    //benchEnd(Serial);  //about 970us? bench: 972.5126us (233403 ticks)
    ESP_LOGI(TAG, "=== Beijing Timezone NTP Sync ===");
    if (!connectWiFi()) {
      ESP_LOGI(TAG, "WiFi failed, time may be incorrect. Check config and restart");
    }
    initNTP();
    Init_time_sync();

    // Fetch real-time sea-level pressure from Open-Meteo API
    if (WiFi.status() == WL_CONNECTED) {
      float online_pressure = fetch_sea_level_pressure_online();
      if (online_pressure > 0) {
        sea_level_pressure_hpa = online_pressure;
        ESP_LOGI(TAG, "Sea-level pressure updated from network: %.2f hPa", sea_level_pressure_hpa);
      } else {
        ESP_LOGI(TAG, "Using default sea-level pressure: %.2f hPa", sea_level_pressure_hpa);
      }
    }
  }
  if (Loop_WiFi_perodic_update_EN) {
    if (0 == sys_cnt % WiFi_refresh_interval_tmp) {

    if (millis() - lastWiFiCheck > wifiCheckInterval) {
    if (WiFi.status() != WL_CONNECTED) {
      ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
      connectWiFi();
    }
    lastWiFiCheck = millis();
  }
      checkTimeSync();

      WiFi_connect_quote_test(15);
      if (wifi_sucess_cnt > 0) {
        WiFi_refresh_interval_tmp = wifi_sucess_cnt * EPD_refresh_interval;
      }
    }
  }
  if (Loop_perodic_update_EN) {
    if (0 == sys_cnt % EPD_refresh_interval) {
      leds[0] = CRGB::Red;
      FastLED.show();
      ESP_LOGI(TAG, "--- periodic_update --- CO2 Max:%u Min:%u", CO2_Max, CO2_Min);
      checkTimeSync();
      if (timeSynced) {
        ESP_LOGD(TAG, "Current system time:");
        printLocalTime();
      } else {
        ESP_LOGD(TAG, "Time not synced yet");
      }

      ESP_LOGD(TAG, "Updating EPD buffer...");

      sprintf(text_buf, "sys_cnt:%lu", sys_cnt);
      testdrawsys_info(text_buf, COLOR1);  //will clear buffer

      /*
      if (sys_on_time_hour > 0) {
        sys_on_time_second_tmp = sys_on_time_second - sys_on_time_hour * 3600;
        sprintf(text_buf, ">:sys on time %u hour,%u s!", sys_on_time_hour, sys_on_time_second_tmp);


        //Serial.printf(">>:sys on time since last boot %u hour,%lu s!\r\n",sys_on_time_hour,sys_on_time_second_tmp);
      } else {
        //Serial.printf(">>:sys on time since last boot %u s!\r\n",sys_on_time_second);
        sys_on_time_second_tmp = sys_on_time_second;
        sprintf(text_buf, ">:sys on time %u s!", sys_on_time_second_tmp);
      }
      */
      sprintf(text_buf, ">:sys on time:%s!", timeDisplayStr);

      //sprintf(text_buf,">>:sys on time %u hour,%lu s!",sys_on_time_hour,sys_on_time_second_tmp);
      EPD_draw_sys_on_time(text_buf, COLOR1);


      display.setFont(&FreeSerifBold18pt7b);  
      //display.setTextSize(2);
      if(EPD_darw_icon_test_EN){
          sprintf(text_buf, " O2 %u", CO2_cur);
      }
      else sprintf(text_buf, "CO2:%u", CO2_cur);
      CO2_Sensor_info_draw(text_buf, COLOR2);
      display.setFont(&FreeSerif9pt7b);  //too big
      display.setTextSize(1);
      sprintf(text_buf, "T:%3.1f C", Display_Temperature);
      SensorT_info_draw(text_buf, COLOR2);
      sprintf(text_buf, "H:%3.1f %%", Display_Humidity);
      Sensor_info2_draw(text_buf, COLOR2);
      Sensor_info2_draw(text_buf, COLOR1);
      //sprintf(text_buf, "CO2:%u,T:%3.1fC,          H:%3.1f%%", CO2_cur, Temperature_cur, Humidity_cur); //ori with def font
      display.setFont();

      sprintf(time_buf,"Tm:%02dh:%02dm:%02ds", 
               timeinfo.tm_hour, 
               timeinfo.tm_min, 
               timeinfo.tm_sec);
      sprintf(date_buf,"%04d-%02d-%02d", 
               timeinfo.tm_year + 1900, 
               timeinfo.tm_mon + 1, 
               timeinfo.tm_mday);

      //sprintf(text_buf, "CO2Max:%u,Min:%u", CO2_Max, CO2_Min);
      //Sensor_info_MinMax_draw(text_buf, COLOR1);

      if(EPD_darw_icon_test_EN) {
        //EPD_drawIcon_BG(Icon_Chart64_X, Icon_Chart64_Y, 64, 64, epd_bitmap_MAX_64, EPD_WHITE,EPD_RED); 
        //epd_bitmap_Max_icon_zell
        EPD_drawIcon_BG(Icon_CO2Chart6436_X, Icon_CO2Chart6436_Y, 64, 36, epd_bitmap_Max_icon_zell, EPD_WHITE,EPD_RED); 
        //epd_bitmap_CO2Max_icon_zell
        //EPD_drawIcon_BG(Icon_Chart80_X, Icon_Chart80_Y, 80, 44, epd_bitmap_Max_icon_zell, EPD_WHITE,EPD_RED); 

        sprintf(text_buf, "CO2");
        CO2_Sensor_text_draw(text_buf, COLOR1);

        Sensor_ppm_draw_with_icon("PPM", COLOR2);

        sprintf(text_buf, "#%u,   %u,   %u",CO2_ave,CO2_Max, CO2_Min);
        Sensor_info_MinMax_draw(text_buf, COLOR1);

        //epd_bitmap_average
        EPD_drawIcon_BG(Icon_CO2ChartAvg_X, Icon_CO2ChartAvg_Y, 16, 16, epd_bitmap_average, EPD_WHITE,EPD_RED); 

        //draw rise icon
        if (CO2_cur>CO2_previous){
        EPD_drawIcon_BG(Icon_CO2rise_X, Icon_CO2Chart6436_Y, 32, 32, epd_bitmap_rising_icon, EPD_WHITE,EPD_BLACK); 
        }
        //draw decline icon, need improve bugs? fixed y pos bug!
        else if (CO2_cur<CO2_previous)
        {
           EPD_drawIcon_BG(Icon_CO2rise_X, Icon_CO2Chart6436_Y, 32, 32, epd_bitmap_decline_icon, EPD_WHITE,EPD_BLACK); 
        }
        CO2_previous = CO2_cur;


        if (999<CO2_ave){
          EPD_drawIcon(Icon_Max_X+8, Icon_Max_Y, 16, 16, maxIcon, EPD_BLACK); 
          EPD_drawIcon(Icon_Min_X+16, Icon_Min_Y, 16, 16, minIcon, EPD_BLACK); 
        }
        else {
          EPD_drawIcon(Icon_Max_X, Icon_Max_Y, 16, 16, maxIcon, EPD_BLACK); 
          EPD_drawIcon(Icon_Min_X, Icon_Min_Y, 16, 16, minIcon, EPD_BLACK); 
        }

        if(WiFi_connect_status){
          EPD_drawIcon_BG(Icon_wifi_X, Icon_wifi_Y, 16, 16, epd_bitmap_Wifi_small, EPD_WHITE,EPD_BLACK); 

        }

       }
       else{
        //without icons
        sprintf(text_buf, "#%u,Max%u,Min%u",CO2_ave,CO2_Max, CO2_Min);
        Sensor_info_MinMax_draw(text_buf, COLOR1);
        Sensor_ppm_draw("PPM", COLOR1);
       }

      //sprintf(text_buf, "Ave:%u,cnt:%u; %s %s", CO2_ave, CO2_data_cnt,time_buf,date_buf);
      //sprintf(text_buf, "Co2Ave: %u, cnt: %u", CO2_ave, CO2_data_cnt);
      //Sensor_info_average_draw(text_buf, COLOR2);
      if(EPD_darw_title_loop_EN){
        Time_info_draw(time_buf,COLOR2);
      }
      else {

        Time_info_title_draw(time_buf,COLOR2);
      }

      Date_info_average_draw(date_buf,COLOR1);



      //display.setFont(&FreeSans9pt7b);
      display.setTextSize(1);
      version_buf[0] = Version_Nrd;
      version_buf[1] = '.';
      version_buf[2] = Version_Nrf1;
      version_buf[3] = Version_Nrf2;
      memset(text_buf, 0, sizeof(text_buf));
      if (EPD_darw_title_loop_EN){
        sprintf(text_buf, "SCD40 sensor APP V%s by ZELL 2026", version_buf);
        EPD_draw_title_info(text_buf, COLOR1);
      }
      else{
        sprintf(text_buf, "SCD40 V%s ZELL",version_buf);
        EPD_draw_title_info_tiny(text_buf, COLOR1);
      }
      
      if (wifi_sucess_cnt > 0) {

        Quote_text_draw(quote_text, EPD_RED);
      }

      if(Sensor_temperature_history_record_EN){
            //sprintf(temp_stats_buf, "Tmax:%.1f Tmin:%.1f", 
            ESP_LOGD(TAG, "Update temperature_history_max_min now...");
            sprintf(text_buf, "T:v%3.1f^%.1f;Daily:%.1f/%.1f", 
            temperature_history_min,temperature_history_max,temperature_daily_min, temperature_daily_max);
            //EPD_draw_Tmax_Tmin_info(text_buf, COLOR1); //old, pre 2.28
            EPD_draw_Tmax_Tmin_info_Bot(text_buf, COLOR1);
      }

      sprintf(text_buf, "EPD_R:%lu", EPD_refresh_cnt);
      EPD_draw_EPD_refresh_cnt(text_buf,COLOR2);

      // Draw sensor presence indicators (red dots, bottom-right)
#if BMP580Sensor_EN
      if (BMP580Sensor_exist) {
        display.fillCircle(EPD_width - 20, EPD_height - 4, 2, EPD_BLACK);
      }
#endif

#if SHT40Sensor_EN
      if (SHT40Sensor_exist) {
        display.fillCircle(EPD_width - 24, EPD_height - 4, 2, EPD_RED);
      }
#endif
      if (MCP9804_sensor_exist) {
        display.fillCircle(EPD_width - 28, EPD_height - 4, 2, EPD_BLACK);
      }

#if BMP580Sensor_EN
      if (BMP580Sensor_exist) {
        EPD_draw_pressure_top_center(BMP580_Pressure_cur, EPD_BLACK);
      }
#endif

      ESP_LOGI(TAG, "Push EPD display refresh now...");
      display.display();
      EPD_refresh_cnt++;

      display.setFont();
      delay(100);
      sys_on_time_second += EPD_display_refresh_time + 3;
      ESP_LOGI(TAG, "EPD refresh done! Total cnt:%u", EPD_refresh_cnt);
    }
  }


  if(EEPROM_save_EN){
    if(0==sys_cnt % EEPROM_save_interval){
      
      saveTemperatureStatsToEEPROM();
    }


  }

  // Periodic JSON output of all sensor data
  unsigned long now = millis();
  if (json_output_EN && (now - lastJsonOutput >= (unsigned long)JSON_OUTPUT_INTERVAL_S * 1000UL)) {
    lastJsonOutput = now;
    StaticJsonDocument<384> jsonDoc;
    jsonDoc["co2_ppm"] = CO2_cur;
    jsonDoc["temp_scd40"] = serialized(String(Temperature_cur, 1));
    if (MCP9804_sensor_exist && !isnan(current_temp_MCP9804)) {
      jsonDoc["temp_mcp9804"] = serialized(String(current_temp_MCP9804, 2));
    } else {
      jsonDoc["temp_mcp9804"] = (char*)NULL;
    }
#if SHT40Sensor_EN
    if (SHT40Sensor_exist && !isnan(SHT40_Temperature_cur)) {
      jsonDoc["temp_sht40"] = serialized(String(SHT40_Temperature_cur, 2));
      jsonDoc["hum_sht40"] = serialized(String(SHT40_Humidity_cur, 1));
    }
#endif

#if BMP580Sensor_EN
    if (BMP580Sensor_exist && !isnan(BMP580_Temperature_cur)) {
      jsonDoc["temp_BMP580"] = serialized(String(BMP580_Temperature_cur, 2));
      jsonDoc["Pressure_BMP580"] = serialized(String(BMP580_Pressure_cur, 1));
    }
#endif
    jsonDoc["humidity"] = serialized(String(Humidity_cur, 1));
    jsonDoc["display_temp"] = serialized(String(Display_Temperature, 2));
    jsonDoc["display_hum"] = serialized(String(Display_Humidity, 1));
    jsonDoc["temp_source"] = Display_Temp_Source;
    jsonDoc["co2_avg"] = CO2_ave;
    jsonDoc["co2_max"] = CO2_Max;
    jsonDoc["co2_min"] = CO2_Min;
    jsonDoc["uptime_s"] = sys_on_time_second;
    char jsonBuf[384];
    serializeJson(jsonDoc, jsonBuf, sizeof(jsonBuf));
    Serial.println(jsonBuf);
  }

#if BLE_SENSOR_RECEIVE_EN
  handleSerialCommands();
  // Periodic BLE scan
  if (BLE_receive_EN && (millis() - last_BLE_receive_ms >= BLE_receive_interval_ms)) {
    bool got_data = BLE_sensor_receive_scan();
    if (got_data) {
      BLE_sensor_receive_print_latest();
    }
    last_BLE_receive_ms = millis();
  }
#endif

#if BLE_SENSOR_BROADCAST_EN
  if (BLE_broadcast_EN && (millis() - last_BLE_broadcast_ms >= BLE_broadcast_interval_ms)) {
    BLE_sensor_broadcast_update(Display_Temperature, Display_Humidity, CO2_cur, BMP580_Pressure_cur);
    last_BLE_broadcast_ms = millis();
  }
#endif

  ESP_LOGW(TAG, "Going into light sleep, WS2812 LED off (UART wakeup on GPIO%d)", UART_RX_PIN);
  digitalWrite(Boot_pin, 0);
  FastLED.clear();
  leds[0] = CRGB::Black;
  FastLED.show();
#if BLE_SENSOR_RECEIVE_EN
  if (BLE_receive_EN) {
    BLE_sensor_receive_stop();
  }
#endif
#if BLE_SENSOR_BROADCAST_EN
  if (BLE_broadcast_EN) {
    BLE_sensor_broadcast_stop();
  }
#endif
  Serial.flush();
  Serial.end();  // release UART pins before reconfiguring GPIO

  // Enable both timer and GPIO (UART RX) wakeup sources
  esp_sleep_enable_timer_wakeup(sleepTime);
  gpio_set_direction(UART_RX_PIN, GPIO_MODE_INPUT);
  gpio_pullup_en(UART_RX_PIN);
  gpio_wakeup_enable(UART_RX_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_light_sleep_start();

  // Restore UART after wakeup
  Serial.begin(115200);
  delay(50);

  // Check wakeup cause
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  if (wakeup_cause == ESP_SLEEP_WAKEUP_GPIO) {
    delay(100);
    while (Serial.available()) Serial.read();  // discard wakeup trigger bytes
    ESP_LOGI(TAG, "[UART WAKEUP] CMD window open for %u ms", UART_CMD_WINDOW_MS);
    unsigned long cmd_start = millis();
    while (millis() - cmd_start < UART_CMD_WINDOW_MS) {
      if (Serial.available()) {
#if BLE_SENSOR_RECEIVE_EN
        handleSerialCommands();
#endif
        break;
      }
      delay(10);
    }
    ESP_LOGI(TAG, "[UART WAKEUP] CMD window closed.");
  } else {
    ESP_LOGD(TAG, "Wake up from timer (normal)");
  }

  sys_sleep_time_second += sleepTime_seconds;
  delay(10);
  digitalWrite(Boot_pin, 1);
#if BLE_SENSOR_BROADCAST_EN
  if (BLE_broadcast_EN) {
    BLE_sensor_broadcast_start();
  }
#endif

  sys_on_time_second += sleepTime_seconds;

  //delay(1000);
}

//need test, 1.Mar.2026
void EPD_drawIcon(int x, int y, int w, int h, const uint8_t *bitmap, uint16_t color) {
    display.drawBitmap(x, y, bitmap, w, h, color);
}

void EPD_drawIcon_BG(int x, int y, int w, int h, const uint8_t *bitmap, uint16_t color,uint16_t color_BG) {
    display.drawBitmap(x, y, bitmap, w, h, color,color_BG);
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
  pixel_pos_y = EPD_height / 2;
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
//EPD_refresh_cnt
void EPD_draw_EPD_refresh_cnt(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  //pixel_pos_x = EPD_width- 48;  //Botton right
  //pixel_pos_x = EPD_width / 2 + EPD_width / 5 - text_height;  //Botton right
  pixel_pos_x = EPD_width / 2 + EPD_width / 5+text_height;  //Botton right
  //pixel_pos_y = EPD_height - text_height;
  pixel_pos_y = EPD_text_pos_Y;
  //display.clearBuffer();
  display.setFont(&Picopixel);  
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void EPD_draw_title_info(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  pixel_pos_x = text_height;  
  pixel_pos_y = 0;
  // display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void EPD_draw_title_info_tiny(const char *text, uint16_t color) {
  uint8_t text_height = 6;
  pixel_pos_x = text_height;
  pixel_pos_y = text_height;
  // display.clearBuffer();
  display.setFont(&Picopixel);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
  display.setFont();
  display.setTextSize(1);
}

void EPD_draw_pressure_top_center(float pressure_hPa, uint16_t color) {
  char buf[16];
  sprintf(buf, "%.2f hPa", pressure_hPa);

  //display.setFont(&FreeSans9pt7b);
  display.setFont();
  display.setTextSize(1);
  display.setTextColor(color);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);

  int16_t px = (EPD_width - w) / 2 - x1;

  int16_t py = 12; //FreeSans9pt7b
  py = 0; //def font

  display.setCursor(px, py);
  display.print(buf);
  display.setFont();
  display.setTextSize(1);
}

void Date_info_average_draw(const char *text, uint16_t color) {
  uint8_t text_height = 8;
  pixel_pos_x = EPD_width-EPD_width/4;  
  pixel_pos_y = text_height+2;
  //display.clearBuffer();
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
  pixel_pos_y = EPD_height / 2 - 2 * text_height + 16;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Sensor_info_draw_with_offset(const char *text, uint16_t color, int16_t Y_offset) {
  //first line
  uint8_t text_height = 18;
  pixel_pos_x = 4;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 - 2 * text_height + 16+Y_offset;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void CO2_Sensor_info_draw(const char *text, uint16_t color) {
  //first line
  uint8_t text_height = 18;
  pixel_pos_x = 4;  // EPD_weight/2;
  //pixel_pos_y = EPD_height / 2 - 2 * text_height + 16;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, CO2_value_pos_Y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

//

void CO2_Sensor_text_draw(const char *text, uint16_t color) {
  //first line
  uint8_t text_height = 8;
  pixel_pos_x = 4;  // EPD_weight/2;
  //pixel_pos_y = EPD_height / 2 - 2 * text_height + 16;
  //display.clearBuffer();
  display.setFont();
  display.setTextSize(1);
  display.setCursor(CO2_text_pos_X, CO2_text_pos_Y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void SensorT_info_draw(const char *text, uint16_t color) {
  //first line
  uint8_t text_height = 18;
  uint8_t text_width = 12;
  pixel_pos_x = EPD_width / 2 + 4 * text_width;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 - 2 * text_height + 6;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}
//Hum
void Sensor_info2_draw(const char *text, uint16_t color) {
  //second line
  uint8_t text_height = 18;
  uint8_t text_width = 12;
  //pixel_pos_x = EPD_width/2-4;  // EPD_weight/2;
  pixel_pos_x = EPD_width / 2 + 4 * text_width;
  pixel_pos_y = EPD_height / 2 - text_height + 8;
  //display.clearBuffer();
  //display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Sensor_ppm_draw(const char *text, uint16_t color) {
  //second line
  uint8_t text_height = 8;
  uint8_t text_width = 6;
  //pixel_pos_x = EPD_width/2-4;  // EPD_weight/2;
  pixel_pos_x = EPD_width / 2 + 3 * text_width;
  pixel_pos_y = EPD_height / 2 - 2*text_height ;
  //display.clearBuffer();
  display.setTextSize(1);
  //display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setCursor(ppm_text_pos_X, ppm_text_pos_Y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void Sensor_ppm_draw_with_icon(const char *text, uint16_t color) {
  //second line
  uint8_t text_height = 8;
  uint8_t text_width = 6;
  //pixel_pos_x = EPD_width/2-4;  // EPD_weight/2;
  pixel_pos_x = EPD_width / 2 + text_width;
  pixel_pos_y = EPD_height / 2 - 2*text_height ;
  //display.clearBuffer();
  display.setTextSize(1);
  //display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setCursor(ppm_text_pos_X, ppm_text_pos_Y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}


void Sensor_info_MinMax_draw(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = 6;  // EPD_weight/2;
  pixel_pos_y = EPD_height / 2 - 4;
  //display.clearBuffer();
  display.setTextSize(2);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void EPD_draw_Tmax_Tmin_info(const char *text, uint16_t color){
  uint8_t text_height = 8;
  pixel_pos_x = text_height;  
  pixel_pos_y = text_height;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void EPD_draw_Tmax_Tmin_info_Bot(const char *text, uint16_t color){
  uint8_t text_height = 8;
  pixel_pos_x = text_height/2;  
  pixel_pos_y = EPD_height-text_height;
  //display.clearBuffer();
  display.setTextSize(1);
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
  pixel_pos_x = 4;  // EPD_width/2;
  pixel_pos_y = EPD_height - 2 * text_height + 8;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}
void Time_info_draw(const char *text, uint16_t color) {
  uint8_t text_height = 16;
  pixel_pos_x = EPD_width-EPD_width/4-24;  // EPD_weight/2;
  pixel_pos_y = EPD_height - 2 * text_height + 8;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}

void Time_info_title_draw(const char *text, uint16_t color) {
  uint8_t text_width = 6;
  pixel_pos_x = EPD_width-14*text_width-text_width/2;  
  pixel_pos_y = 0 ;
  //display.clearBuffer();
  display.setTextSize(1);
  display.setCursor(pixel_pos_x, pixel_pos_y);
  display.setTextColor(color);
  display.setTextWrap(true);
  display.print(text);
}



void show_IIC_pins(void) {
  ESP_LOGD(TAG, "IIC: SDA=%d SCL=%d", SDA, SCL);
}

void show_SPI_pins(void) {
  ESP_LOGD(TAG, "SPI: MOSI=%d MISO=%d SCK=%d SS=%d", MOSI, MISO, SCK, SS);
}

void WiFi_connect_quote_test(uint8_t retry_cnt) {
  uint8_t try_cnt = 0;
  ESP_LOGI(TAG, "Attempting to connect to SSID: %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    try_cnt++;
    if (retry_cnt < try_cnt) {
      ESP_LOGI(TAG, "WiFi Connection failed!");
      WiFi_connect_status = 0;
      return;
    }
  }
  WiFi_connect_status = 1;
  ESP_LOGI(TAG, "WiFi Connected to %s", ssid);

  ESP_LOGD(TAG, "Starting connection to server: %s...", server);
  client.setInsecure();
  if (!client.connect(server, 443)) {
    ESP_LOGI(TAG, "Server connection failed!");
    return;
  }
  ESP_LOGD(TAG, "Connected to server!");
  // Make a HTTP request:
  client.print("GET ");
  client.print(path);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(server);
  client.println("Connection: close");
  client.println();

  // Check HTTP status
  char status[32] = { 0 };
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    ESP_LOGI(TAG, "Unexpected response: %s", status);
    return;
  }
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      ESP_LOGD(TAG, "Headers received");
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
    ESP_LOGI(TAG, "deserializeJson() failed: %s", error.c_str());
    return;
  }
  JsonObject root_0 = doc[0];
  const char *root_0_text = root_0["text"];
  const char *root_0_author = root_0["author"];
  ESP_LOGI(TAG, "Quote: %s", root_0_text);
  ESP_LOGD(TAG, "Author: %s", root_0_author);
  //quote_text=  root_0_text;
  strcpy(quote_text, root_0_text);
  //Draw something here!
  if (sys_cnt < EPD_refresh_interval) {
    Quote_text_draw(root_0_text, EPD_BLACK);
  } else
    Quote_text_draw(root_0_text, EPD_RED);
  display.display();
  wifi_sucess_cnt++;
  while (client.available() > 0) {
    client.readStringUntil('\r');
  }

  client.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  ESP_LOGI(TAG, "WiFi turned off");
}

void updateTimeDisplay() {
  sys_on_time_hour = sys_on_time_second / 3600;
  uint32_t remaining_seconds = sys_on_time_second % 3600;

  if (sys_on_time_hour > 0) {
    // 显示小时和分钟
    uint32_t minutes = remaining_seconds / 60;
    uint32_t seconds = remaining_seconds % 60;
    snprintf(timeDisplayStr, sizeof(timeDisplayStr),
             "%uh:%02um:%02us", sys_on_time_hour, minutes, seconds);
  } else {
    // 只显示秒数
    uint32_t minutes = sys_on_time_second / 60;
    uint32_t seconds = sys_on_time_second % 60;
    snprintf(timeDisplayStr, sizeof(timeDisplayStr),
             "%02u m:%02u s", minutes, seconds);
  }
  ESP32_active_ratio =  100.0*(float)sys_active_time_second/(float)sys_on_time_second ;
}


void saveTemperatureStatsToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    
    int addr = 0;
    EEPROM.put(addr, temperature_history_max);
    addr += sizeof(float);
    EEPROM.put(addr, temperature_history_min);
    addr += sizeof(float);
    EEPROM.put(addr, temperature_history_sum);
    addr += sizeof(float);
    EEPROM.put(addr, temperature_reading_count);
    addr += sizeof(uint32_t);
    
    EEPROM.commit();
    EEPROM.end();
    
    ESP_LOGW(TAG, "Temperature history stats saved to EEPROM");
}

void loadTemperatureStatsFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    
    int addr = 0;
    EEPROM.get(addr, temperature_history_max);
    addr += sizeof(float);
    EEPROM.get(addr, temperature_history_min);
    addr += sizeof(float);
    EEPROM.get(addr, temperature_history_sum);
    addr += sizeof(float);
    EEPROM.get(addr, temperature_reading_count);
    
    // Recalculate average
    if (temperature_reading_count > 0) {
        temperature_history_avg = temperature_history_sum / temperature_reading_count;
    }
    
    EEPROM.end();
    
    ESP_LOGW(TAG, "Temperature history stats loaded from EEPROM");
}

void Show_ESP32_sys_info(void) {
  ESP_LOGI(TAG, "CPU:%u MHz  XTAL:%u MHz  APB:%u MHz",
           getCpuFrequencyMhz(), getXtalFrequencyMhz(), (uint32_t)(getApbFrequency()/1000000));
  ESP_LOGI(TAG, "--ESP32 Memory and Storage--");
  ESP_LOGI(TAG, "  Program Size: %.1f KB", ESP.getSketchSize() / 1024.0);
  ESP_LOGI(TAG, "  Free Program Space: %.1f KB", ESP.getFreeSketchSpace() / 1024.0);
  ESP_LOGI(TAG, "  Flash Size: %.1f MB  Free Heap: %.1f KB",
           ESP.getFlashChipSize() / (1024.0 * 1024.0), ESP.getFreeHeap() / 1024.0);
#ifdef BOARD_HAS_PSRAM
  if (psramFound()) {
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "  PSRAM: Total %.1f MB, Free %.1f KB",
             (heap_info.total_free_bytes + heap_info.total_allocated_bytes) / (1024.0 * 1024.0),
             heap_info.total_free_bytes / 1024.0);
  } else {
    ESP_LOGI(TAG, "  PSRAM: Not detected");
  }
#else
  ESP_LOGI(TAG, "  PSRAM: Not supported on this board");
#endif
}

#ifdef BMP580Sensor_EN

void BMP580_setting_readback_helper(void){
  Serial.print(F("Temperature Oversampling: "));
  switch(bmp580.getTemperatureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print pressure oversampling inline
  Serial.print(F("Pressure Oversampling: "));
  switch(bmp580.getPressureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  // Pretty print IIR filter coefficient inline
  Serial.print(F("IIR Filter Coefficient: "));
  switch(bmp580.getIIRFilterCoeff()) {
    case BMP5XX_IIR_FILTER_BYPASS:   Serial.println(F("Bypass (No filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_1:  Serial.println(F("1 (Light filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_3:  Serial.println(F("3 (Medium filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_7:  Serial.println(F("7 (More filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_15: Serial.println(F("15 (Heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_31: Serial.println(F("31 (Very heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_63: Serial.println(F("63 (Maximum filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_127:Serial.println(F("127 (Maximum filtering)")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print output data rate inline
  Serial.print(F("Output Data Rate: "));
  switch(bmp580.getOutputDataRate()) {
    case BMP5XX_ODR_240_HZ:   Serial.println(F("240 Hz")); break;
    case BMP5XX_ODR_218_5_HZ: Serial.println(F("218.5 Hz")); break;
    case BMP5XX_ODR_199_1_HZ: Serial.println(F("199.1 Hz")); break;
    case BMP5XX_ODR_179_2_HZ: Serial.println(F("179.2 Hz")); break;
    case BMP5XX_ODR_160_HZ:   Serial.println(F("160 Hz")); break;
    case BMP5XX_ODR_149_3_HZ: Serial.println(F("149.3 Hz")); break;
    case BMP5XX_ODR_140_HZ:   Serial.println(F("140 Hz")); break;
    case BMP5XX_ODR_129_8_HZ: Serial.println(F("129.8 Hz")); break;
    case BMP5XX_ODR_120_HZ:   Serial.println(F("120 Hz")); break;
    case BMP5XX_ODR_110_1_HZ: Serial.println(F("110.1 Hz")); break;
    case BMP5XX_ODR_100_2_HZ: Serial.println(F("100.2 Hz")); break;
    case BMP5XX_ODR_89_6_HZ:  Serial.println(F("89.6 Hz")); break;
    case BMP5XX_ODR_80_HZ:    Serial.println(F("80 Hz")); break;
    case BMP5XX_ODR_70_HZ:    Serial.println(F("70 Hz")); break;
    case BMP5XX_ODR_60_HZ:    Serial.println(F("60 Hz")); break;
    case BMP5XX_ODR_50_HZ:    Serial.println(F("50 Hz")); break;
    case BMP5XX_ODR_45_HZ:    Serial.println(F("45 Hz")); break;
    case BMP5XX_ODR_40_HZ:    Serial.println(F("40 Hz")); break;
    case BMP5XX_ODR_35_HZ:    Serial.println(F("35 Hz")); break;
    case BMP5XX_ODR_30_HZ:    Serial.println(F("30 Hz")); break;
    case BMP5XX_ODR_25_HZ:    Serial.println(F("25 Hz")); break;
    case BMP5XX_ODR_20_HZ:    Serial.println(F("20 Hz")); break;
    case BMP5XX_ODR_15_HZ:    Serial.println(F("15 Hz")); break;
    case BMP5XX_ODR_10_HZ:    Serial.println(F("10 Hz")); break;
    case BMP5XX_ODR_05_HZ:    Serial.println(F("5 Hz")); break;
    case BMP5XX_ODR_04_HZ:    Serial.println(F("4 Hz")); break;
    case BMP5XX_ODR_03_HZ:    Serial.println(F("3 Hz")); break;
    case BMP5XX_ODR_02_HZ:    Serial.println(F("2 Hz")); break;
    case BMP5XX_ODR_01_HZ:    Serial.println(F("1 Hz")); break;
    case BMP5XX_ODR_0_5_HZ:   Serial.println(F("0.5 Hz")); break;
    case BMP5XX_ODR_0_250_HZ: Serial.println(F("0.25 Hz")); break;
    case BMP5XX_ODR_0_125_HZ: Serial.println(F("0.125 Hz")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print power mode inline
  Serial.print(F("Power Mode: "));
  switch(bmp580.getPowerMode()) {
    case BMP5XX_POWERMODE_STANDBY:     Serial.println(F("Standby")); break;
    case BMP5XX_POWERMODE_NORMAL:      Serial.println(F("Normal")); break;
    case BMP5XX_POWERMODE_FORCED:      Serial.println(F("Forced")); break;
    case BMP5XX_POWERMODE_CONTINUOUS:  Serial.println(F("Continuous")); break;
    case BMP5XX_POWERMODE_DEEP_STANDBY:Serial.println(F("Deep Standby")); break;
    default: Serial.println(F("Unknown")); break;
  }
  // Print sensor details using the unified sensor API
  Serial.println(F("=== Temperature Sensor Details ==="));
  sensor_t temp_sensor;
  bmp_temp->getSensor(&temp_sensor);
  Serial.print(F("Sensor Name: ")); Serial.println(temp_sensor.name);
  Serial.print(F("Sensor Type: ")); Serial.println(temp_sensor.type);
  Serial.print(F("Driver Ver:  ")); Serial.println(temp_sensor.version);
  Serial.print(F("Unique ID:   ")); Serial.println(temp_sensor.sensor_id);
  Serial.print(F("Min Value:   ")); Serial.print(temp_sensor.min_value); Serial.println(F(" °C"));
  Serial.print(F("Max Value:   ")); Serial.print(temp_sensor.max_value); Serial.println(F(" °C"));
  Serial.print(F("Resolution:  ")); Serial.print(temp_sensor.resolution); Serial.println(F(" °C"));
  Serial.println();

  Serial.println(F("===24bit Pressure Sensor Details ==="));
  sensor_t pressure_sensor;
  bmp_pressure->getSensor(&pressure_sensor);
  Serial.print(F("Sensor Name: ")); Serial.println(pressure_sensor.name);
  Serial.print(F("Sensor Type: ")); Serial.println(pressure_sensor.type);
  Serial.print(F("Driver Ver:  ")); Serial.println(pressure_sensor.version);
  Serial.print(F("Unique ID:   ")); Serial.println(pressure_sensor.sensor_id);
  Serial.print(F("Min Value:   ")); Serial.print(pressure_sensor.min_value); Serial.println(F(" hPa"));
  Serial.print(F("Max Value:   ")); Serial.print(pressure_sensor.max_value); Serial.println(F(" hPa"));
  Serial.print(F("Resolution:  ")); Serial.print(pressure_sensor.resolution); Serial.println(F(" hPa"));
  Serial.println();
  Serial.println();
}

void BMP580_read_value_test(void){

  Serial.print(F("BMP580 Temperature = "));
  Serial.print(bmp580.temperature);
  Serial.println(F(" °C"));

  Serial.print(F("BMP580 Pressure = "));
  Serial.print(bmp580.pressure);
  Serial.println(F(" hPa"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bmp580.readAltitude(sea_level_pressure_hpa));
  Serial.println(F(" m"));

}
#endif

// 插入新数据
void add_CO2_data(float CO2_cur) {
    CO2_history[CO2_array_write_idx] = CO2_cur;
    CO2_array_write_idx = (CO2_array_write_idx + 1) % CO2_history_ave_size;
    //if (CO2_data_cnt < CO2_history_ave_size) {
    //    CO2_data_cnt++;
    //}
}

// 计算当前存储数据的平均值（如果有效数据量不足，可酌情处理）
/*
float get_CO2_average() {
    if (CO2_data_count == 0) 
      return 0;
    
    float sum = 0;
    for (int i = 0; i < CO2_data_count; i++) {
        sum += CO2_history[i];
    }
    return sum / CO2_data_cnt;
}
*/
float get_CO2_average() {
    int count = CO2_data_cnt;
    if (count > CO2_history_ave_size) {
        count = CO2_history_ave_size;   // 修正为不超过数组大小
    }
    if (count == 0) return 0;
    
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += CO2_history[i];
    }
    return sum / count;
}
// ========== UART Command Handler ==========
#if BLE_SENSOR_RECEIVE_EN
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "ble_rx_en") {
    if (!BLE_receive_EN) {
      BLE_receive_EN = true;
      BLE_prefs.begin("myapp", false);
      BLE_prefs.putBool("ble_rx_en", true);
      BLE_prefs.end();
      BLE_sensor_receive_init();
      Serial.println("[CMD]>: BLE receive ENABLED.");
    } else {
      Serial.println("[CMD]>: BLE receive already enabled.");
    }
  } else if (cmd == "ble_rx_dis") {
    if (BLE_receive_EN) {
      BLE_receive_EN = false;
      BLE_prefs.begin("myapp", false);
      BLE_prefs.putBool("ble_rx_en", false);
      BLE_prefs.end();
      BLE_sensor_receive_stop();  // only stop scan, don't deinit BLE stack (shared with broadcast)
      Serial.println("[CMD]>: BLE receive DISABLED.");
    } else {
      Serial.println("[CMD]>: BLE receive already disabled.");
    }
  } else if (cmd.startsWith("ble_rx_interval ")) {
    uint32_t val = (uint32_t)cmd.substring(16).toInt();
    if (val >= 500 && val <= 600000) {
      BLE_receive_interval_ms = val;
      BLE_prefs.begin("myapp", false);
      BLE_prefs.putULong("ble_rx_intv", BLE_receive_interval_ms);
      BLE_prefs.end();
      Serial.printf("[CMD]>: BLE receive interval set to %lu ms (saved).\r\n", BLE_receive_interval_ms);
    } else {
      Serial.println("[CMD]>: Invalid value! Range: 500 ~ 600000 ms.");
    }
  } else if (cmd == "ble_rx_show") {
    BLE_sensor_receive_print_latest();
  } else if (cmd == "ble_rx_history") {
    BLE_sensor_receive_print_history();
  } else if (cmd == "ble_rx_stats") {
    BLE_sensor_receive_print_stats();
  } else if (cmd == "ble_rx_scan") {
    if (BLE_receive_EN) {
      Serial.println("[CMD]>: Manual BLE scan...");
      bool found = BLE_sensor_receive_scan();
      if (found) {
        BLE_sensor_receive_print_latest();
      } else {
        Serial.println("[CMD]>: No C3 sensor found in this scan.");
      }
    } else {
      Serial.println("[CMD]>: BLE receive not enabled. Send 'ble_rx_en' first.");
    }
#if BLE_SENSOR_BROADCAST_EN
  } else if (cmd == "ble_tx_en") {
    if (!BLE_broadcast_EN) {
      BLE_broadcast_EN = true;
      BLE_TX_prefs.begin("myapp", false);
      BLE_TX_prefs.putBool("ble_tx_en", true);
      BLE_TX_prefs.end();
      BLE_sensor_broadcast_init();
      Serial.println("[CMD]>: BLE broadcast ENABLED.");
    } else {
      Serial.println("[CMD]>: BLE broadcast already enabled.");
    }
  } else if (cmd == "ble_tx_dis") {
    if (BLE_broadcast_EN) {
      BLE_broadcast_EN = false;
      BLE_TX_prefs.begin("myapp", false);
      BLE_TX_prefs.putBool("ble_tx_en", false);
      BLE_TX_prefs.end();
      BLE_sensor_broadcast_stop();  // only stop advertising, don't deinit BLE stack (shared with RX)
      Serial.println("[CMD]>: BLE broadcast DISABLED.");
    } else {
      Serial.println("[CMD]>: BLE broadcast already disabled.");
    }
  } else if (cmd.startsWith("ble_tx_interval ")) {
    uint32_t val = (uint32_t)cmd.substring(16).toInt();
    if (val >= 1000 && val <= 600000) {
      BLE_broadcast_interval_ms = val;
      BLE_TX_prefs.begin("myapp", false);
      BLE_TX_prefs.putULong("ble_tx_intv", BLE_broadcast_interval_ms);
      BLE_TX_prefs.end();
      Serial.printf("[CMD]>: BLE broadcast interval set to %lu ms (saved).\r\n", BLE_broadcast_interval_ms);
    } else {
      Serial.println("[CMD]>: Invalid value! Range: 1000 ~ 600000 ms.");
    }
#endif
  } else if (cmd == "json_en") {
    json_output_EN = true;
    Serial.printf("[CMD]>: JSON output ENABLED (interval %u s).\r\n", JSON_OUTPUT_INTERVAL_S);
  } else if (cmd == "json_dis") {
    json_output_EN = false;
    Serial.println("[CMD]>: JSON output DISABLED.");
  } else if (cmd == "status") {
    Serial.println("\r\n========== SYSTEM STATUS ==========");
    Serial.printf("Loop count:              %lu\r\n", sys_cnt);
    Serial.printf("Free heap:               %u bytes\r\n", ESP.getFreeHeap());
    Serial.printf("Uptime:                  %lu s\r\n", sys_on_time_second);
    Serial.printf("JSON output:             %s (every %u s)\r\n", json_output_EN ? "ON" : "OFF", JSON_OUTPUT_INTERVAL_S);
#if BLE_SENSOR_BROADCAST_EN
    Serial.printf("BLE broadcast:           %s (every %lu ms, sent: %u)\r\n", BLE_broadcast_EN ? "ON" : "OFF", BLE_broadcast_interval_ms, BLE_tx_seq);
#endif
    Serial.printf("BLE receive:             %s\r\n", BLE_receive_EN ? "YES" : "NO");
    Serial.printf("BLE scan mode:           Active (ScanResp enabled)\r\n");
    Serial.printf("BLE receive interval:    %lu ms\r\n", BLE_receive_interval_ms);
    Serial.printf("BLE total scans:         %lu\r\n", BLE_RX_scan_count);
    Serial.printf("BLE scan failures:       %lu\r\n", BLE_RX_scan_fail_count);
    Serial.printf("BLE total received:      %lu\r\n", BLE_RX_total_count);
    if (BLE_RX_latest.valid) {
      unsigned long age = (millis() - BLE_RX_latest.timestamp_ms) / 1000;
      Serial.printf("BLE latest (age %lus, RSSI=%d, %s): T=%.2f H=%.2f P=%.1f\r\n",
          age, BLE_RX_latest.rssi,
          BLE_RX_latest.from_scan_resp ? "ScanResp" : "MfrData",
          BLE_RX_latest.temperature, BLE_RX_latest.humidity, BLE_RX_latest.pressure);
    } else {
      Serial.println("BLE latest:              No data yet");
    }
    Serial.println("===================================\r\n");
  } else if (cmd == "help") {
    Serial.println("[CMD] Available commands:");
    Serial.println("  json_en            - Enable JSON periodic output");
    Serial.println("  json_dis           - Disable JSON periodic output");
#if BLE_SENSOR_BROADCAST_EN
    Serial.println("  ble_tx_en          - Enable BLE CO2 broadcast");
    Serial.println("  ble_tx_dis         - Disable BLE CO2 broadcast");
    Serial.println("  ble_tx_interval N  - Set broadcast interval (1000~600000 ms)");
#endif
    Serial.println("  ble_rx_en          - Enable BLE receive");
    Serial.println("  ble_rx_dis         - Disable BLE receive");
    Serial.println("  ble_rx_interval N  - Set scan interval (500~600000 ms)");
    Serial.println("  ble_rx_scan        - Manual scan now");
    Serial.println("  ble_rx_show        - Show latest received data");
    Serial.println("  ble_rx_history     - Show all buffered data");
    Serial.println("  ble_rx_stats       - Show scan statistics");
    Serial.println("  status             - System status");
  } else if (cmd.length() > 0) {
    Serial.printf("[CMD]>: Unknown command: %s (type 'help' for list)\r\n", cmd.c_str());
  }
}
#endif
//end main ino

/*

21:42:43.754 -> Build:Mar 27 2021
21:42:43.754 -> rst:0x15 (USB_UART_CHIP_RESET),boot:0x0 (DOWNLOAD(USB/UART0))
21:42:43.763 -> Saved PC:0x40378d58
21:42:43.765 -> waiting for download
21:42:48.422 -> ESP-ROM:esp32s3-20210327
21:42:48.428 -> Build:Mar 27 2021
21:42:48.428 -> rst:0x1 (POWERON),boot:0x28 (SPI_FAST_FLASH_BOOT)
21:42:48.433 -> SPIWP:0xee
21:42:48.433 -> mode:DIO, clock div:1
21:42:48.433 -> load:0x3fce2820,len:0x116c
21:42:48.439 -> load:0x403c8700,len:0xc2c
21:42:48.439 -> load:0x403cb700,len:0x3108
21:42:48.439 -> entry 0x403c88b8

21:42:48.685 -> <Adafruit 2.13' BW+R EPD SCD40 sensor test by Zell 2026>
21:42:48.685 -> -Version 1.20, 13.Feb.2025 by Zell-
21:42:48.706 -> -FW Compile time: 21:41:33, date: Feb 13 2026
21:42:48.706 -> CPU Frequency: 240 MHz
21:42:48.706 -> XTAL Frequency: 40 MHz
21:42:48.706 -> APB Bus Frequency: 80 MHz
21:42:48.706 -> --ESP32 Memory and Storage--
21:42:48.788 ->   Program Size: 1127.8 KB
21:42:48.788 ->   Free Program Space: 3264.0 KB
21:42:48.788 ->   Flash Size: 8.0 MB
21:42:48.793 ->   Free Heap: 287.8 KB
21:42:48.793 ->   PSRAM: Not supported on this board
21:42:48.799 -> ---Code Configuration---
21:42:48.799 -> #Init_test_EPD_EN 0;SCD40Sensor_EN 1;Loop_perodic_update_EN 1
21:42:48.804 -> >:EPD_width 250;EPD_height 122
21:42:48.825 -> >:Code FW using EPD_Driver_IC:SSD1680
21:42:48.825 -> >:ESP32 sleep interval:10 s, EPD update interval:300 s
21:42:48.825 -> ESP32s3 SPI interface:
21:42:48.825 -> MOSI: 11
21:42:48.825 -> MISO: 13
21:42:48.825 -> SCK: 12
21:42:48.825 -> SS: 10
21:42:48.825 -> ESP32s3 IIC interface:
21:42:48.825 -> SDA: 8
21:42:48.847 -> SCL: 9
21:42:48.847 -> Turn on Onboard WS2812 LED
21:42:48.847 -> >:init SPI...
21:42:48.847 -> >:init EPD...
21:42:48.880 -> Starting EPD chip detection...Init SPI pins now...

21:42:48.880 -> 1. Trying to read SSD series chip ID...
21:42:48.990 ->   SSD ID Read: 0xFF 0xFF 0xFF 0xFF

21:42:48.990 -> 2. Trying to read UC8151D chip ID...
21:42:49.104 ->   UC8151D ID Read: 0xFF

21:42:49.104 -> 3. Trying to read IL0373 status...
21:42:49.214 ->   IL0373 Status: 0x00

21:42:49.214 -> Detection completed!
21:42:49.341 -> >:init IIC...
21:42:49.850 -> SCD40 Sensor detected !
21:42:50.350 -> #>:SCD40 Periodic measurement is disabled!
21:42:50.350 -> #>:SCD40 Low power mode enabled!
21:42:50.356 -> #>:SCD40 setSensorAltitude:100!
21:42:50.356 -> #>:Temperature_history_record enabled! Initialize temperature history variables now!
21:42:50.374 -> Daily temperature statistics reset!
21:42:50.374 -> Temperature statistics reset!
21:42:50.374 -> Daily temperature statistics reset!


*/

/*
V1.35

完成。修改内容：

  1. 新增全局变量 bool json_output_EN = true; — 默认开启 JSON 输出
  2. JSON 输出条件 — 在定时判断中增加 json_output_EN && 前置条件
  3. UART 命令 — 添加 json_en 和 json_dis 两个命令来开关
  4. status 显示 — 增加 JSON output: ON/OFF (every 30 s) 状态行
  5. help 列表 — 增加 json_en / json_dis 命令说明

  
V1.33 

● 修改完成。总结一下所做的更改：

  数据源优先级逻辑（SHT40 > MCP9804 > SCD40）：

  1. 新增变量 — Display_Temperature, Display_Humidity, Display_Temp_Source（0=SCD40, 1=MCP9804, 2=SHT40）
  2. SHT40 初始化修复 — 移除了 while(1) 死循环，SHT40 不存在时不再卡死，改为和其他传感器一致的处理方式
  3. 优先级选择逻辑（loop 每次传感器读取后）：
    - SHT40 存在且数据有效 → 用 SHT40 的温度和湿度
    - MCP9804 存在且数据有效 → 用 MCP9804 温度 + SCD40 湿度（MCP9804 无湿度传感）
    - 否则 → 用 SCD40 原始数据作为 fallback
  4. GUI 显示 — EPD 上显示的温湿度从 Temperature_cur/Humidity_cur 改为 Display_Temperature/Display_Humidity
  5. 温度历史统计 — updateTemperatureStats() 也使用优先级选择后的温度
  6. JSON 输出 — 增加了 temp_sht40, hum_sht40, display_temp, display_hum, temp_source 字段，便于调试和数据记录

。已添加 #define SHT40Sensor_EN 1 宏定义（位于其他 _EN 宏附近），并用 #if SHT40Sensor_EN ... #endif 包裹了所有
  SHT40 相关代码：

  - #include <SHT40.h>
  - SHT40 sht40; 实例
  - SHT40Sensor_exist / SHT40_Temperature_cur / SHT40_Humidity_cur 变量
  - setup 中的初始化
  - loop 中的读取
  - 优先级选择中的 SHT40 分支
  - JSON 输出中的 SHT40 字段

  将 SHT40Sensor_EN 设为 0 即可完全禁用 SHT40 支持，此时优先级退化为 MCP9804 > SCD40。

*/
/*
14:25:07.293 -> I2C Scanning...
14:25:07.305 -> [scan]>>:I2C device found at address 0x44
14:25:07.305 -> [scan]>>:I2C device found at address 0x62 SCD40!

*/

/*
14:34:20.404 -> [scan]>>:I2C device found at address 0x44
14:34:20.404 -> [scan]>>:I2C device found at address 0x47
14:34:20.418 -> [scan]>>:I2C device found at address 0x62
14:34:20.418 -> [scan]>>:I2C device found at address 0x7E
*/
