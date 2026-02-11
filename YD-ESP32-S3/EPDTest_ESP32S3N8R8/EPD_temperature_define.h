#ifndef EPD_temp_H
#define EPD_temp_H

extern float Temperature_cur;
// Temperature history tracking variables
float temperature_history_max = -999.0;     // Initialize with very low value
float temperature_history_min = 999.0;      // Initialize with very high value
float temperature_history_sum = 0.0;        // Sum of all temperature readings
uint32_t temperature_reading_count = 0;     // Total number of temperature readings
float temperature_history_avg = 0.0;        // Running average
float temperature_current_avg = 0.0;        // Current average

// For rolling average (last N readings)
#define TEMP_HISTORY_SIZE 60               // Keep last 60 readings for rolling average
float temperature_rolling_history[TEMP_HISTORY_SIZE]; // Circular buffer
int temp_history_index = 0;                // Current index in circular buffer
bool temp_buffer_filled = false;           // Flag if buffer is full
float temperature_rolling_avg = 0.0;       // Rolling average

// For daily tracking (24 hours)
#define TEMP_DAILY_SAMPLES 288             // 5-minute intervals (288 = 24h * 12)
float temperature_daily_min = 999.0;
float temperature_daily_max = -999.0;
float temperature_daily_sum = 0.0;
uint32_t daily_reading_count = 0;

// Temperature statistics structure (optional)
struct TemperatureStats {
    float current;
    float max;
    float min;
    float avg;
    float rolling_avg;
    uint32_t count;
    uint32_t timestamp;
};

TemperatureStats Temperature_stats;

// Time tracking for daily reset
unsigned long last_daily_reset_time = 0;
const unsigned long DAILY_RESET_INTERVAL = 24 * 60 * 60 * 1000; // 24 hours in milliseconds

void updateDailyTemperatureStats(float currentTemp);
void resetDailyTemperatureStats() ;
// 获取特定统计值
float getTemperatureMax() { return temperature_history_max; }
float getTemperatureMin() { return temperature_history_min; }
float getTemperatureAvg() { return temperature_history_avg; }
float getTemperatureRollingAvg() { return temperature_rolling_avg; }

// 添加新的函数来更新温度统计数据
void updateTemperatureStats(float currentTemp) {
    // Update global statistics
    //temperature_reading_count++;
    temperature_history_sum += currentTemp;
    temperature_history_avg = temperature_history_sum / temperature_reading_count;
    
    // Update max/min
    if (currentTemp > temperature_history_max) {
        temperature_history_max = currentTemp;
    }
    if (currentTemp < temperature_history_min) {
        temperature_history_min = currentTemp;
         //Serial.printf("###>:temperature_daily_min updated to %3.1fC %.1f\r\n",currentTemp,temperature_daily_min);
    }
    
    // Update rolling average (circular buffer)
    temperature_rolling_history[temp_history_index] = currentTemp;
    temp_history_index = (temp_history_index + 1) % TEMP_HISTORY_SIZE;
    
    if (temp_history_index == 0) {
        temp_buffer_filled = true;
    }
    
    // Calculate rolling average
    float rolling_sum = 0.0;
    int count = temp_buffer_filled ? TEMP_HISTORY_SIZE : temp_history_index;
    
    for (int i = 0; i < count; i++) {
        rolling_sum += temperature_rolling_history[i];
    }
    
    if (count > 0) {
        temperature_rolling_avg = rolling_sum / count;
    }
    
    // Update daily statistics
    updateDailyTemperatureStats(currentTemp);
    
    // Update stats structure
    Temperature_stats.current = currentTemp;
    Temperature_stats.max = temperature_history_max;
    Temperature_stats.min = temperature_history_min;
    Temperature_stats.avg = temperature_history_avg;
    Temperature_stats.rolling_avg = temperature_rolling_avg;
    Temperature_stats.count = temperature_reading_count;
    Temperature_stats.timestamp = millis();
}

// 更新每日统计数据
void updateDailyTemperatureStats(float currentTemp) {
    daily_reading_count++;
    temperature_daily_sum += currentTemp;
    
    if (currentTemp > temperature_daily_max) {
        temperature_daily_max = currentTemp;
    }
    if (currentTemp < temperature_daily_min) {
        temperature_daily_min = currentTemp;
       
    }
    
    // Check if 24 hours have passed for daily reset
    unsigned long currentTime = millis();
    if (currentTime - last_daily_reset_time >= DAILY_RESET_INTERVAL) {
        resetDailyTemperatureStats();
        last_daily_reset_time = currentTime;
    }
}

// 重置每日统计数据
void resetDailyTemperatureStats() {
    temperature_daily_min = 999.0;
    temperature_daily_max = -999.0;
    temperature_daily_sum = 0.0;
    daily_reading_count = 0;
    Serial.println("Daily temperature statistics reset!");
}

// 重置所有温度统计数据
void resetTemperatureStats() {
    temperature_history_max = -999;
    temperature_history_min = 999;
    temperature_history_sum = 0;
    temperature_reading_count = 1;
    temperature_history_avg = 0;
    
    resetDailyTemperatureStats();
    
    Serial.println("Temperature statistics reset!");
}

// 打印温度统计数据
void printTemperatureStats() {
    Serial.println("====== Temperature Statistics ======");
    Serial.printf("Current: %.1fC\n", Temperature_cur);
    Serial.printf("Historical Max: %.1fC\n", temperature_history_max);
    Serial.printf("Historical Min: %.1fC\n", temperature_history_min);
    //Serial.printf(">>:Temperature_stats Min: %.1fC\n", Temperature_stats.min);
    Serial.printf("Historical Avg: %.1fC\n", temperature_history_avg);
    Serial.printf("Rolling Avg (%d samples): %.1fC\n", 
                 temp_buffer_filled ? TEMP_HISTORY_SIZE : temp_history_index,
                 temperature_rolling_avg);
    Serial.printf("Daily Max: %.1fC\n", temperature_daily_max);
    Serial.printf("Daily Min: %.1fC\n", temperature_daily_min);
    if (daily_reading_count > 0) {
        Serial.printf("Daily Avg: %.1fC\n", temperature_daily_sum / daily_reading_count);//°C
    }
    Serial.printf("Total Readings: %u\n", temperature_reading_count);
    Serial.println("===================================");
}



// 添加获取温度统计数据的函数（供其他部分使用）
TemperatureStats getTemperatureStats() {
    return Temperature_stats;
}



#endif