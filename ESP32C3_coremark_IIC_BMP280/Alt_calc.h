#ifndef calculateAltitude_head
#define calculateAltitude_head
/*

5. 杭州特定参数
年平均海平面气压
1016-1017 hPa（杭州地区年平均）
3. 季节对比
春季（3月-5月）： 1015-1020 hPa

夏季（6月-8月）： 1005-1015 hPa（受台风、副热带高压影响）

秋季（9月-11月）： 1015-1020 hPa

冬季（12月-2月）： 1020-1025 hPa

在输出中提供杭州不同区域的海拔参考值

*/

// 海拔计算相关参数
// 杭州的海平面参考气压（平均约为1012.5 hPa，根据季节和天气会有变化）
// 实际应用中建议根据当地气象站数据校准
//https://metar-taf.com/zh/metar/ZSHC
#define SEA_LEVEL_PRESSURE_HZ 1029.1  //need float 杭州平均海平面气压 (hPa)
#define TEMPERATURE_LAPSE_RATE 0.0065  // 温度递减率 (K/m)
#define GRAVITY 9.80665  // 重力加速度 (m/s^2)
#define MOLAR_MASS 0.0289644  // 干空气摩尔质量 (kg/mol)
#define UNIVERSAL_GAS_CONSTANT 8.314462618  // 通用气体常数 (J/(mol·K))
#define GAS_CONSTANT_DRY_AIR 287.058  // 干空气气体常数 (J/(kg·K))


// ==================== 海拔高度计算函数 ====================

/**
 * @brief 使用国际标准大气压公式计算海拔高度（简化版）
 * @param pressure 当前气压 (hPa)
 * @param seaLevelPressure 海平面参考气压 (hPa)
 * @return 海拔高度 (米)
 * 
 * 公式: h = 44330 * [1 - (P/P0)^(1/5.255)]
 * 其中: P是当前气压, P0是海平面气压
 */
float calculateAltitudeInternational(float pressure, float seaLevelPressure) {
  if (pressure <= 0 || seaLevelPressure <= 0) {
    return 0.0;
  }
  
  // 确保气压单位一致
  float p = pressure;
  float p0 = seaLevelPressure;
  
  // 国际标准大气压公式
  float altitude = 44330.0 * (1.0 - pow(p / p0, 1.0 / 5.255));
  
  return altitude;
}

/**
 * @brief 使用气压高度公式计算海拔高度（考虑温度影响）
 * @param pressure 当前气压 (hPa)
 * @param temperature 当前温度 (°C)
 * @param seaLevelPressure 海平面参考气压 (hPa)
 * @return 海拔高度 (米)
 * 
 * 公式: h = (T0 / L) * [1 - (P/P0)^(R*L/g)]
 * 其中: T0 = 288.15K (15°C), L = 0.0065 K/m, R = 287.058 J/(kg·K), g = 9.80665 m/s²
 */
float calculateAltitudeHypsometric(float pressure, float temperature, float seaLevelPressure) {
  if (pressure <= 0 || seaLevelPressure <= 0) {
    return 0.0;
  }
  
  // 转换为绝对温度 (K)
  float T0 = 288.15;  // 海平面标准温度 (15°C)
  float T = temperature + 273.15;  // 当前温度转换为开尔文
  
  // 使用平均温度进行更精确的计算
  float T_avg = (T0 + T) / 2.0;
  
  // 使用气压高度公式
  float altitude = (T_avg / TEMPERATURE_LAPSE_RATE) * 
                   (1.0 - pow(pressure / seaLevelPressure, 
                   (GAS_CONSTANT_DRY_AIR * TEMPERATURE_LAPSE_RATE) / GRAVITY));
  
  return altitude;
}

/**
 * @brief 综合计算海拔高度（主函数，默认使用国际标准公式）
 * @param pressure 当前气压 (hPa)
 * @param temperature 当前温度 (°C)，默认15°C
 * @param seaLevelPressure 海平面参考气压 (hPa)，默认杭州平均海平面气压
 * @return 海拔高度 (米)
 */
float calculateAltitude(float pressure, float temperature, float seaLevelPressure) {
  // 使用国际标准大气压公式（最常用）
  //return calculateAltitudeInternational(pressure, seaLevelPressure);

  return calculateAltitudeHypsometric(pressure,temperature, seaLevelPressure);
}



/**
 * @brief 打印海拔高度信息（包含多种计算方法的比较）
 * @param pressure 当前气压 (hPa)
 * @param temperature 当前温度 (°C)
 */
void printAltitudeInfo(float pressure, float temperature) {
  if (pressure <= 0) {
    Serial.println("Altitude: Invalid pressure reading");
    return;
  }
  
  // 计算海拔高度（使用国际标准公式）
  float altitude = calculateAltitudeInternational(pressure, SEA_LEVEL_PRESSURE_HZ);
  
  // 计算海拔高度（使用考虑温度的公式）
  float altitudeWithTemp = calculateAltitudeHypsometric(pressure, temperature, SEA_LEVEL_PRESSURE_HZ);
  
  Serial.println(">>:--Altitude Calculation (Hangzhou)--");
  Serial.printf("  Sea level pressure: %.1f hPa\r\n", SEA_LEVEL_PRESSURE_HZ);
  Serial.printf("  Current pressure: %.1f hPa\r\n", pressure);
  Serial.printf("  Current temperature: %.1f °C\r\n", temperature);
  Serial.printf("  Altitude (International formula): %.1f m\r\n", altitude);
  Serial.printf("  Altitude (with temperature): %.1f m\r\n", altitudeWithTemp);
  Serial.printf("  Difference: %.1f m\r\n", altitudeWithTemp - altitude);
  
  // 提供海拔范围判断
  if (altitude < 0) {
    Serial.println("  Location: Below sea level (or calibration needed)");
  } else if (altitude < 10) {
    Serial.println("  Location: Sea level area");
  } else if (altitude < 100) {
    Serial.println("  Location: Low altitude");
  } else if (altitude < 500) {
    Serial.println("  Location: Medium altitude");
  } else {
    Serial.println("  Location: High altitude");
  }
  
  // 杭州参考：市区海拔约10-20米，西湖周边约7米，西部山区可达400-500米
  Serial.println("  Note: Hangzhou urban area elevation: ~10-20m");
  Serial.println("        West Lake area: ~7m, West mountains: up to 400-500m");
}

float getHangzhouSeaLevelPressure(int month) {
  // 月份应为1-12
  if (month < 1 || month > 12) month = 1;
  
  // 杭州各月平均海平面气压（近似值，单位：hPa）
  const float monthly_pressure[12] = {
    1023.5, // 1月
    1022.0, // 2月
    1018.5, // 3月
    1016.0, // 4月
    1014.0, // 5月
    1009.5, // 6月
    1007.0, // 7月
    1008.5, // 8月
    1015.0, // 9月
    1018.5, // 10月
    1020.0, // 11月
    1022.5  // 12月
  };
  
  return monthly_pressure[month - 1];
}
/*

*/

#endif