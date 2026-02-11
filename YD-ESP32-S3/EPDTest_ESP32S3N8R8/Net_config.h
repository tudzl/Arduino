#ifndef EPD_Net_H
#define EPD_Net_H

#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <ArduinoJson.h>


const char* ssid     = "Staubli_SD";     // your network SSID (name of wifi network)
const char* password = "smartdevice"; // your network password

const char*  server = "www.adafruit.com";
const char*  path   = "/api/quotes.php";

// NTP服务器配置（推荐使用国内服务器）
const char *ntpServer1 = "cn.pool.ntp.org";     // 中国NTP服务器池
const char *ntpServer2 = "ntp.aliyun.com";      // 阿里云NTP服务器
const char *ntpServer3 = "time1.cloud.tencent.com"; // 腾讯云NTP服务器

const char *ntpServer4 = "pool.ntp.org";
const char *ntpServer5 = "time.nist.gov";
const long gmtOffset_sec = 8*3600;
const int daylightOffset_sec = 0;
//const char *time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
// 或者使用TZ格式的时区字符串（更标准）
const char *time_zone = "CST-8"; 

static unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 60000; // 每30秒检查一次

WiFiClientSecure client;
// 全局变量
struct tm timeinfo;
char time_buf[20];
char date_buf[20];

bool timeSynced = false;
unsigned long lastSyncTime = 0;
const unsigned long syncInterval = 3600000; // 每小时同步一次（1小时=3600000毫秒）
void printLocalTime() ;
void timeavailable(struct timeval *t) {
  Serial.println(">:Time synchronized successfully!");
  timeSynced = true;
  lastSyncTime = millis();
  printLocalTime();
}

// Print local time (multiple formats)
void printLocalTime() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("!>:Unable to get time");
    return;
  }
  
  // Format 1: Full date and time
  Serial.printf("Date: %04d-%02d-%02d\n", 
               timeinfo.tm_year + 1900, 
               timeinfo.tm_mon + 1, 
               timeinfo.tm_mday);
  
  // Format 2: Day of week
  const char* weekday[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  Serial.printf("Day: %s\n", weekday[timeinfo.tm_wday]);
  
  // Format 3: Time
  Serial.printf("⏰ Time: %02d:%02d:%02d\n", 
               timeinfo.tm_hour, 
               timeinfo.tm_min, 
               timeinfo.tm_sec);
  
  // Format 4: Timestamp
  time_t now;
  time(&now);
  Serial.printf("⏳ Timestamp: %ld\n", now);
  
  Serial.println("-------------------------");
}

// Get formatted time string
String getFormattedTime() {
  if (!getLocalTime(&timeinfo)) {
    return "Time not synchronized";
  }
  
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// Get timestamp
time_t getTimestamp() {
  time_t now;
  time(&now);
  return now;
}

// Check if time needs to be re-synced
void checkTimeSync() {
  if (!timeSynced || (millis() - lastSyncTime > syncInterval)) {
    Serial.println("🔄 Synchronizing time...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
    timeSynced = false;
  }
}

// WiFi connection function
bool connectWiFi() {
  Serial.printf(">:📶 Connecting to WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  const int maxAttempts = 20; // 10 second timeout (20 * 500ms)
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected successfully");
    Serial.printf("📡 IP Address: %s\n", WiFi.localIP().toString().c_str());
    return true;
  } else {
    Serial.println("\n✗ WiFi connection failed");
    return false;
  }
}

void initNTP() {
  // Set time sync callback
  sntp_set_time_sync_notification_cb(timeavailable);
  
  // Enable SNTP service
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  
  // Method 1: Fixed offset (simple)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
  
  // Or Method 2: Timezone string (more standard)
  // configTzTime(time_zone, ntpServer1, ntpServer2, ntpServer3);
  
  Serial.println("⏲️ NTP client initialized");
}

void Init_time_sync(void){

  // 3. Wait for initial time sync
  Serial.println("\n⏳ Waiting for initial time synchronization...");
  int syncAttempts = 0;
  while (!timeSynced && syncAttempts < 10) {
    delay(500);
    syncAttempts++;
    
    // Try to get time
    if (getLocalTime(&timeinfo)) {
      timeSynced = true;
      Serial.println("✓ Initial time sync completed");
      printLocalTime();
      break;
    }
    
    if (syncAttempts % 5 == 0) {
      Serial.println("⏳ Still synchronizing...");
    }
  }
  
  if (!timeSynced) {
    Serial.println("⚠️  Time sync timeout, check network connection");
  }
  
  Serial.println("\n✅ NTP Time System initialization completed");
  Serial.println("---------------------------------------------");
}

// Optional: Function to get specific time format
String getFormattedDateTime() {
  if (!getLocalTime(&timeinfo)) {
    return "Time not synchronized";
  }
  
  const char* weekday[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  char buffer[50];
  
  // Format: 2024-01-20 Sat 14:30:45
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %s %02d:%02d:%02d",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           weekday[timeinfo.tm_wday],
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);
  
  return String(buffer);
}

#endif
/*
https://www.programiz.com/c-programming/online-compiler/

20:44:40.058 -> 
20:44:45.091 -> !>:Returning from light sleep
20:44:45.091 -> >>:loop running! 15
20:44:45.091 -> >>:sys on time since last boot 70 s!
20:44:45.096 -> Attempting to connect to SSID: Staubli_SD
20:44:45.173 -> ................>>: WiFi Connection failed!
20:44:48.378 -> >>:CO2 Max:1693 ;CO2 Min:890
20:44:48.378 -> >>:updating EPD buffer now!
20:44:48.382 -> >>:Push EPD display refresh now!
20:45:10.371 -> >>:EPD display refresh finished now!
20:45:10.371 -> >>:Going into light sleep mode,turn off WS2812 LED now!

*/