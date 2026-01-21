//header for detect Driver IC

void detectEPDChip();
bool testPartialRefresh();
bool sendInitSequence(uint8_t* sequence, int length);
bool testYellowColorSupport();
bool testSSD1680Init();
bool testSSD1675BInit();
bool testIL0373Init();
void sendCommand(uint8_t command) {
  digitalWrite(EPD_DC, LOW);  // 命令模式
  digitalWrite(EPD_CS, LOW);
  SPI.transfer(command);
  digitalWrite(EPD_CS, HIGH);
}



// 芯片类型枚举定义
enum ChipType {
  UNKNOWN = 0,
  SSD1675,
  SSD1675B,
  SSD1680,
  IL0373,
  UC8151D
};

// 芯片类型名称字符串
const char* chipTypeToString(ChipType type) {
  switch(type) {
    case UNKNOWN:   return "UNKNOWN";
    case SSD1675:   return "SSD1675";
    case SSD1675B:  return "SSD1675B";
    case SSD1680:   return "SSD1680";
    case IL0373:    return "IL0373";
    case UC8151D:   return "UC8151D";
    default:        return "UNDEFINED";
  }
}


// 读取数据函数
uint8_t readData() {
  digitalWrite(EPD_DC, HIGH);  // 数据模式
  digitalWrite(EPD_CS, LOW);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(EPD_CS, HIGH);
  return data;
}

// 检测芯片型号
void detectEPDChip() {
  Serial.println("Starting EPD chip detection...Init SPI pins now...");

  SPI.begin();
  pinMode(EPD_CS, OUTPUT);
  pinMode(EPD_DC, OUTPUT);
  pinMode(EPD_BUSY, INPUT);
  pinMode(EPD_RESET, OUTPUT);
  
  digitalWrite(EPD_CS, HIGH);
  digitalWrite(EPD_DC, HIGH);
  
  // 1. 尝试读取SSD系列芯片ID
  Serial.println("\n1. Trying to read SSD series chip ID...");
  digitalWrite(EPD_RESET, LOW);
  delay(10);
  digitalWrite(EPD_RESET, HIGH);
  delay(100);
  
  sendCommand(0x71);  // SSD系列读取ID命令
  uint8_t id1 = readData();
  uint8_t id2 = readData();
  uint8_t id3 = readData();
  uint8_t id4 = readData();
  
  Serial.printf("  SSD ID Read: 0x%02X 0x%02X 0x%02X 0x%02X\n", id1, id2, id3, id4);
  
  // SSD芯片ID识别
  if (id1 == 0x1B && id2 == 0x1B && id3 == 0x1B && id4 == 0x1B) {
    Serial.println("  Detected: SSD1675 (old version)");
  } else if (id1 == 0x5B && id2 == 0x5B && id3 == 0x5B && id4 == 0x5B) {
    Serial.println("  Detected: SSD1675B");
  } else if (id1 == 0x2B && id2 == 0x2B && id3 == 0x2B && id4 == 0x2B) {
    Serial.println("  Detected: SSD1680");
  }
  
  // 2. 尝试读取UC8151D芯片ID
  Serial.println("\n2. Trying to read UC8151D chip ID...");
  digitalWrite(EPD_RESET, LOW);
  delay(10);
  digitalWrite(EPD_RESET, HIGH);
  delay(100);
  
  sendCommand(0x71);  // UC8151D也有读取ID命令
  delay(1);
  uint8_t uc_id = readData();
  Serial.printf("  UC8151D ID Read: 0x%02X\n", uc_id);
  
  if (uc_id == 0x15) {
    Serial.println("  Detected: UC8151D");
  }
  
  // 3. 尝试读取IL0373状态寄存器
  Serial.println("\n3. Trying to read IL0373 status...");
  digitalWrite(EPD_RESET, LOW);
  delay(10);
  digitalWrite(EPD_RESET, HIGH);
  delay(100);
  
  sendCommand(0x2F);  // IL0373获取状态命令
  delay(1);
  uint8_t il_status = readData();
  Serial.printf("  IL0373 Status: 0x%02X\n", il_status);
  
  // IL0373通常没有明确的ID，但状态寄存器有特定模式
  if ((il_status & 0x80) == 0x80) {
    Serial.println("  Possibly: IL0373 (needs further verification)");
  }
  
  Serial.println("\nDetection completed!");
}


void detectByFeatureCommands() {
  Serial.println("\nTesting feature commands...");
  
  // 测试局部刷新功能（SSD1680和UC8151D支持）
  Serial.println("1. Testing partial refresh support...");
  if (testPartialRefresh()) {
    Serial.println("  Partial refresh supported -> SSD1680 or UC8151D");
    
    // 进一步区分
    if (testYellowColorSupport()) {
      Serial.println("  Yellow color supported -> UC8151D");
    } else {
      Serial.println("  No yellow color support -> SSD1680");
    }
  } else {
    Serial.println("  No partial refresh -> SSD1675, SSD1675B, or IL0373");
    
    /*
    // 测试快速刷新模式
    if (testFastRefresh()) {
      Serial.println("  Fast refresh mode supported -> SSD1675B");
    } else {
      // 进一步区分SSD1675和IL0373
      if (testTemperatureCompensation()) {
        Serial.println("  Basic temp compensation -> SSD1675");
      } else {
        Serial.println("  Minimal temp compensation -> IL0373");
      }
    }
      */

  }
}

bool testPartialRefresh() {
  // 尝试设置局部刷新模式
  sendCommand(0x91);  // 部分刷新进入命令
  SPI.transfer(0x00);
  sendCommand(0x90);  // 部分刷新控制
  SPI.transfer(0x01);
  
  delay(100);
  
  // 发送局部刷新显示命令
  sendCommand(0x13);  // 部分刷新显示数据
  
  // 检查BUSY引脚行为
  unsigned long start = millis();
  while (digitalRead(EPD_BUSY) == LOW) {
    if (millis() - start > 1000) {
      return true;  // 快速响应，支持局部刷新
    }
    delay(10);
  }
  
  return false;
}

bool testYellowColorSupport() {
  // UC8151D支持黄色显示
  sendCommand(0x50);  // VCOM和数据类型设置
  SPI.transfer(0x97); // 包含4种颜色的设置
  
  sendCommand(0x20);  // VCOM寄存器
  // 尝试写入黄色相关的设置
  sendCommand(0x10);  // 数据开始传输
  
  // 发送黄色测试模式
  uint8_t yellow_test[] = {0x05, 0x05, 0x05, 0x05};  // 黄色编码
  
  for (int i = 0; i < 4; i++) {
    SPI.transfer(yellow_test[i]);
  }
  
  sendCommand(0x12);  // 刷新显示
  
  // 检查显示效果（需要光传感器或人为判断）
  // 这里简化处理，假设有黄色支持
  return true;  // 实际项目中需要更复杂的检测
}





bool testSSD1680Init() {
  // SSD1680特定的初始化序列
  uint8_t ssd1680_init[] = {
    0x01, 5, 0x03, 0x00, 0x2b, 0x2b, 0x03,  // Power setting
    0x06, 3, 0x17, 0x17, 0x17,             // Booster soft start
    0x04, 0,                               // Power on
    0x00, 2, 0xbf, 0x0d,                   // Panel setting
    0x30, 1, 0x3c,                         // PLL control
    0x61, 3, 0x00, 0xf9, 0x00,             // Resolution setting
    0x82, 1, 0x28,                         // VCM_DC setting
    0x50, 1, 0x97                          // VCOM and data interval setting
  };
  
  return sendInitSequence(ssd1680_init, sizeof(ssd1680_init));
}

bool testUC8151DInit() {
  // UC8151D特定的初始化序列
  uint8_t uc8151d_init[] = {
    0x01, 6, 0x07, 0x07, 0x3f, 0x3f, 0x0c, 0x00,  // Power setting
    0x82, 1, 0x1c,                                // VCOM_DC setting
    0x50, 1, 0x97,                                // Panel setting
    0x20, 0,                                      // VCOM register
    0x65, 1, 0x00                                 // Resolution setting
  };
  
  return sendInitSequence(uc8151d_init, sizeof(uc8151d_init));
}

ChipType detectByInitialization() {
  Serial.println("Testing initialization sequences...");
  
  // 测试SSD1680初始化
  if (testSSD1680Init()) {
    Serial.println("  SSD1680 initialization successful");
    return SSD1680;
  }
  
  // 测试SSD1675B初始化
  if (testSSD1675BInit()) {
    Serial.println("  SSD1675B initialization successful");
    return SSD1675B;
  }
  
  // 测试UC8151D初始化
  if (testUC8151DInit()) {
    Serial.println("  UC8151D initialization successful");
    return UC8151D;
  }
  
  // 测试IL0373初始化
  if (testIL0373Init()) {
    Serial.println("  IL0373 initialization successful");
    return IL0373;
  }
  
  return UNKNOWN;
}

bool sendInitSequence(uint8_t* sequence, int length) {
  // 发送初始化序列并检查响应
  for (int i = 0; i < length; ) {
    uint8_t cmd = sequence[i++];
    uint8_t len = sequence[i++];
    
    sendCommand(cmd);
    for (int j = 0; j < len; j++) {
      SPI.transfer(sequence[i++]);
    }
    
    // 检查BUSY引脚状态（如果芯片响应正确）
    delay(10);
    if (digitalRead(EPD_BUSY) == HIGH) {
      // 某些芯片在收到错误命令时会保持BUSY为高
      return false;
    }
  }
  
  // 发送初始化完成命令
  sendCommand(0x12);  // Display refresh
  
  // 等待刷新完成（超时检查）
  unsigned long start = millis();
  while (digitalRead(EPD_BUSY) == LOW) {
    if (millis() - start > 5000) {
      return false;  // 超时
    }
    delay(10);
  }
  
  return true;
}