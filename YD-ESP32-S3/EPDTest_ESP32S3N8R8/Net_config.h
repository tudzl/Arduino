#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <ArduinoJson.h>


const char* ssid     = "Staubli_SD";     // your network SSID (name of wifi network)
const char* password = "smartdevice"; // your network password

const char*  server = "www.adafruit.com";
const char*  path   = "/api/quotes.php";

WiFiClientSecure client;