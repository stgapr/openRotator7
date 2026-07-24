//openRotator7.ino - Mini Satellite-Antenna Rotator.
//Copyright (c) 2015-2025 Julie VK3FOWL and Joe VK3YSP
//Modified for ESP8266 with WiFi config web UI
//Release 8.1 - Bugfix release

//Includes
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "index.h"
#include "timer.h"
#include "lsm.h"
#include "mot.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>



uint8_t mac[6];
enum Mode { AP_MODE, STA_MODE };
Mode runMode;

#define FIRMWARE_VERSION "v8.1"
#define OTA_HOSTNAME "rotator7"
#define OTA_PASS     "rotota"

/* ============  Wi-Fi 参数  ============ */
#define AP_SSID_PREFIX    "Rotator7-"
#define AP_PASS           "12345678"
#define TCP_PORT          4533
#define WEB_PORT          80
#define SOCKET_PORT       81
#define EEPROM_SIZE       512
#define CONFIG_TIMEOUT    30000

/* ---------- Network ---------- */
WebSocketsServer   webSocket(SOCKET_PORT);
WiFiServer         tcpServer(TCP_PORT);   // 修复：使用宏定义
ESP8266WebServer   http(WEB_PORT);
// ============ EEPROM 配置结构 ============
struct RotorConfig {
  uint32_t magic;
  uint8_t  extSensor;      // 0=内置, 1=外置
  int      azGain;
  int      elGain;
  char     staSsid[32];
  char     staPass[64];
  uint16_t checksum;
};

#define CONFIG_ADDR     256
#define CONFIG_MAGIC    0x524F544F
#define SENSOR_CAL_ADDR 0
#define EEPUT(addr, data) EEPROM.put(addr, data)

static_assert(sizeof(RotorConfig) <= 256, "RotorConfig too large for EEPROM space");

RotorConfig cfg;

//Constants
const int MotorType = PWMDIR;
const int SensorType = LSM303DLHC;
#define SerialPort Serial
#define WINDUP_LIMIT 450

// ====== 引脚分配 ======
// 根据 ESP12E Motor Shield 的原理图
const int elPwmPin = D1;   // PWMA - GPIO5
const int elDirPin = D3;   // DA - GPIO0
//const int elBrkPin = D8;   // GPIO15 (刹车)

const int azPwmPin = D2;   // PWMB - GPIO4
const int azDirPin = D4;   // DB - GPIO2
//const int azBrkPin = D8;   // GPIO15 (与 elBrkPin 共用)


const int spkPin = D7;// GPIO13

//Motor gains
int azGain = 25;
int elGain = 25;
const float azAlpha = 0.5;
const float elAlpha = 0.5;
const float lsmAlpha = 0.02;

//Modes
enum Modes {tracking, monitoring, demonstrating, calibrating, debugging, pausing};

// ============================================================
//  核心姿态数据（全局唯一真相源）
// ============================================================
struct Attitude {
  float az;          // 当前方位角（度，-180~180）
  float el;          // 当前仰角（度，-90~90）
  float azSet;       // 目标方位角
  float elSet;       // 目标仰角
  float azError;     // 方位角误差
  float elError;     // 仰角误差
  float azWindup;    // 缠绕累积角度
  float azOffset;    // 缠绕偏移
  bool  windup;      // 缠绕状态
  float azLast;      // 上一次方位角（用于跨越南向检测）
  float elLast;      // 上一次仰角
};

Attitude att;  // 全局姿态

//其他全局变量
String line;
String tcpLine;
float azInc, elInc;
Modes mode;
bool needRestart = false;
unsigned long lastConfigSaveTime = 0;
bool configDirty = false;
bool extSensorDataReceived = false;  // 标记外置数据是否已收到
bool extSensorMonitorPrinted = false; // 用于监视模式下的首次提示

//Objects
Mot azMot(MotorType, azAlpha, azGain, azPwmPin, azDirPin);
Mot elMot(MotorType, elAlpha, elGain, elPwmPin, elDirPin);
Lsm lsm(SensorType, lsmAlpha);
Timer t1(100);

//WiFi扫描状态
bool scanRunning = false;
String scanResult = "";
unsigned long scanStartTime = 0;

// ============================================================
//  函数前向声明
// ============================================================
void reset(bool getCal);
void save();
void restore();
void printDebug();
void printCal();
void printMon();
void printAzEl();
void printAz();
void printEl();
void calibrate();
void updateDemoAngles(float *azSet, float *elSet, float *azInc, float *elInc);
float diffAngle(float a, float b);
void computeErrors();
void updateWindup(float az, float el);
void processExternalAngle(String line);
void processTargetAngle(String line);
void processUserCommands(String line);
void processEasycommCommands(String line);
void processCommands(void);
String readSerialLine();
void processPosition();
void processMotors();
void initOTA();
bool connectSTA();
void startAP(String apSSID);
void startScan();
void handleScan();
String generatePage();
void setupHTTP();
void handleTCP();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len);
void broadcastDash();
uint16_t calcCRC16(RotorConfig *c);
void defaultConfig();
void applyConfig();
void loadConfig();
void saveConfig();
void markConfigDirty();
void checkSaveConfig();

// ============================================================
//  防缠绕算法（与数据源无关）
// ============================================================
void updateWindup(float az, float el) {
  float azDiff = az - att.azLast;
  if (azDiff < -180) att.azOffset += 360;
  if (azDiff > 180) att.azOffset -= 360;
  att.azLast = az;
  att.elLast = el;
  att.azWindup = az + att.azOffset;
  if (abs(att.azWindup) > WINDUP_LIMIT) att.windup = true;
}

// ============================================================
//  误差计算（与数据源无关）
// ============================================================
float diffAngle(float a, float b) {
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}


void computeErrors() {
  // 1. 将目标仰角映射到传感器实际工作的 ±90° 范围
  float targetEl = att.elSet;
  float targetAz = att.azSet;
  
  // 如果目标仰角超过 90° 或小于 -90°，执行等效翻转
  if (targetEl > 90.0) {
    targetEl = 180.0 - targetEl;
    targetAz += 180.0; // 方位角翻转 180°
  } else if (targetEl < -90.0) {
    targetEl = -180.0 - targetEl;
    targetAz += 180.0; // 方位角翻转 180°
  }

  // 2. 将方位角归一化到 -180° ~ 180°，便于计算最短路径
  if (targetAz > 180.0) targetAz -= 360.0;
  if (targetAz < -180.0) targetAz += 360.0;

  // 3. 计算误差（与原逻辑一致，但使用了映射后的目标值）
  if (att.windup) {
    att.azError = constrain(att.azWindup, -180, 180);
    if (abs(att.azError) < 175) att.windup = false;
  } else {
    // 计算方位角误差，考虑 ±180° 边界
    float rawAzError = att.az - targetAz;
    if (rawAzError < -180) rawAzError += 360;
    if (rawAzError > 180) rawAzError -= 360;
    att.azError = rawAzError;
  }

  // 计算仰角误差（不需要特殊边界处理，因为范围小）
  att.elError = att.el - targetEl;
}
// ============================================================
//  核心控制流程（与数据源完全解耦）
// ============================================================
void processPosition() {
  bool dataUpdated = false;
  
  if (cfg.extSensor == 0) {
    // 内置传感器模式
    lsm.readGM();
    
    switch (mode) {
      case debugging:
        printDebug();
        return;
      case calibrating:
        calibrate();
        return;
      case pausing:
        azMot.halt();
        elMot.halt();
        return;
      default:
        lsm.getAzEl();
        att.az = lsm.az;
        att.el = lsm.el;
        dataUpdated = true;
        break;
    }
  } else {
    // 外置传感器模式：数据由 XZ/XL 命令更新
    if (mode == pausing) {
      azMot.halt();
      elMot.halt();
      return;
    }
    if (mode == calibrating || mode == debugging) {
      mode = tracking;
      SerialPort.println("External sensor mode: calibration/debug disabled");
    }
    if (extSensorDataReceived) {
      dataUpdated = true;
      extSensorDataReceived = false;
      extSensorMonitorPrinted = false;
    } else {
      // 监视模式下，即使没有新数据，也给出提示
      if (mode == monitoring && !extSensorMonitorPrinted) {
        SerialPort.println("Monitoring: waiting for external sensor data (XZ/XL)...");
        extSensorMonitorPrinted = true;
      }
    }
  }
  
  if (dataUpdated) {
    updateWindup(att.az, att.el);
    
    if (mode == demonstrating) {
      updateDemoAngles(&att.azSet, &att.elSet, &azInc, &elInc);
    }
    
    computeErrors();
    
    if (mode == monitoring) {
      printMon();
    }
  }
}

// ============================================================
//  命令解析（统一入口）
// ============================================================

// 处理 XZ/XL 外置传感器数据
void processExternalAngle(String line) {
  int xzPos = line.indexOf("XZ");
  int xlPos = line.indexOf("XL");
  bool updated = false;
  
  if (xzPos >= 0) {
    int start = xzPos + 2;
    int end = line.indexOf(' ', start);
    if (end < 0) end = line.length();
    String param = line.substring(start, end);
    float newAz = param.toFloat();
    if (newAz > 180) newAz = newAz - 360;
    att.az = newAz;
    updated = true;
  }
  
  if (xlPos >= 0) {
    int start = xlPos + 2;
    int end = line.indexOf(' ', start);
    if (end < 0) end = line.length();
    String param = line.substring(start, end);
    att.el = param.toFloat();
    updated = true;
  }
  
  if (updated) {
    extSensorDataReceived = true;
    updateWindup(att.az, att.el);
    computeErrors();
  }
}

// 处理目标角度命令（AZ/EL）
void processTargetAngle(String line) {
  int firstSpace = line.indexOf(' ');
  if (firstSpace > 0) {
    String param = line.substring(2, firstSpace);
    if (param.length() > 0) {
      att.azSet = param.toFloat();
      if (att.azSet > 180) att.azSet = att.azSet - 360;
    }
    int elPos = line.indexOf("EL", firstSpace);
    if (elPos > 0) {
      param = line.substring(elPos + 2);
      param.trim();
      if (param.length() > 0) {
        att.elSet = param.toFloat();
      }
    }
  } else {
    String param = line.substring(2);
    if (param.length() > 0) {
      att.azSet = param.toFloat();
      if (att.azSet > 180) att.azSet = att.azSet - 360;
    }
  }


  computeErrors();
}

// 处理用户命令（单字符命令）
void processUserCommands(String line) {
  String param;
  int firstSpace;
  char command = line.charAt(0);
  
  switch (command) {
    case 'r':
      SerialPort.println("Reset in progress");
      reset(true);
      SerialPort.println("Reset complete");
      break;
    case 'b':
      if (cfg.extSensor == 1) {
        SerialPort.println("Debug mode disabled in external sensor mode");
      } else {
        SerialPort.println("Debugging in progress: Press 'a' to abort");
        mode = debugging;
        t1.reset(100);
      }
      break;
    case 'm':
      SerialPort.println("Monitoring in progress: Press 'a' to abort");
      mode = monitoring;
      extSensorMonitorPrinted = false;
      t1.reset(100);
      break;
    case 'c':
      if (cfg.extSensor == 1) {
        SerialPort.println("Calibration only available in internal sensor mode");
      } else {
        SerialPort.println("Calibration in progress: Press 'a' to abort or 's' to save");
        reset(false);
        mode = calibrating;
        t1.reset(50);
      }
      break;
    case 'a':
      mode = tracking;
      t1.reset(100);
      reset(true);
      SerialPort.println("Function aborted");
      break;
    case 'e':
      param = line.substring(1);
      lsm.cal.md = param.toFloat();
      SerialPort.printf("Magnetic declination set to: %.1f\n", lsm.cal.md);
      break;
    case 's':
      if (cfg.extSensor == 1) {
        SerialPort.println("Save only available in internal sensor mode");
      } else {
        save();
        reset(true);
        SerialPort.println("Calibration saved");
      }
      break;
    case 'd':
      SerialPort.println("Demo in progress: Press 'a' to abort");
      t1.reset(50);
      mode = demonstrating;
      break;
    case 'h':
      SerialPort.println("Commands:");
      SerialPort.println("az el -(0..360 0..90) - Set target angle");
      SerialPort.println("r - Reset");
      SerialPort.println("eNN.N - Set magnetic declination");
      SerialPort.println("c - Calibrate (internal sensor only)");
      SerialPort.println("s - Save calibration (internal sensor only)");
      SerialPort.println("a - Abort");
      SerialPort.println("d - Demo");
      SerialPort.println("b - Debug (internal sensor only)");
      SerialPort.println("m - Monitor");
      SerialPort.println("p - Pause");
      SerialPort.println("XZxxx XLxxx - External angle report");
      break;
    case 'p':
      if (mode == pausing) mode = tracking;
      else { mode = pausing; SerialPort.println("Paused"); }
      break;
    default:
      firstSpace = line.indexOf(' ');
      if (firstSpace > 0) {
        param = line.substring(0, firstSpace);
        att.azSet = param.toFloat();
        param = line.substring(firstSpace + 1);
        att.elSet = param.toFloat();
      } else {
        param = line.substring(0);
        att.azSet = param.toFloat();
        att.elSet = 0.0;
      }
      computeErrors();
  }
}

// EasyComm 协议统一解析
void processEasycommCommands(String line) {
  line.trim();
  
  if (line.startsWith("XZ") || line.startsWith("XL")) {
    processExternalAngle(line);
    return;
  }
  
  if (line == "AZ") {
    printAz();
    return;
  }
  if (line == "EL") {
    printEl();
    return;
  }
  
  if (line.startsWith("AZ")) {
    processTargetAngle(line);
    return;
  }
}

// ============================================================
//  原有辅助函数
// ============================================================

void reset(bool getCal) {
  att.azSet = 0.0;
  att.elSet = 0.0;
  att.azLast = 0.0;
  att.elLast = 0.0;
  att.azWindup = 0.0;
  att.azOffset = 0.0;
  att.azError = 0.0;
  att.elError = 0.0;
  att.windup = false;
  mode = tracking;
  if (getCal) restore();
  azInc = 0.05;
  elInc = 0.05;
  t1.reset(100);
  printCal();
  lsm.calStart();
}

void save() {
  EEPROM.put(SENSOR_CAL_ADDR, lsm.cal);
  EEPROM.commit();
}

void restore() {
  EEPROM.get(SENSOR_CAL_ADDR, lsm.cal);
}

void printDebug() {
  SerialPort.print(lsm.mx); SerialPort.print(",");
  SerialPort.print(lsm.my); SerialPort.print(",");
  SerialPort.print(lsm.mz); SerialPort.print(",");
  SerialPort.print(lsm.gx); SerialPort.print(",");
  SerialPort.print(lsm.gy); SerialPort.print(",");
  SerialPort.println(lsm.gz);
}

void printCal() {
  SerialPort.print(lsm.cal.md, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.me.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ge.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.j, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.ms.k, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.i, 1); SerialPort.print(",");
  SerialPort.print(lsm.cal.gs.j, 1); SerialPort.print(",");
  SerialPort.println(lsm.cal.gs.k, 1);
}

// 修复：简化 printMon，全部使用 att 结构体
void printMon() {
  SerialPort.print(att.az, 0); SerialPort.print(",");
  SerialPort.print(att.el, 0); SerialPort.print(",");
  SerialPort.print(att.azSet, 0); SerialPort.print(",");
  SerialPort.print(att.elSet, 0); SerialPort.print(",");
  SerialPort.print(att.azWindup, 0); SerialPort.print(",");
  SerialPort.print(att.windup); SerialPort.print(",");
  SerialPort.print(att.azError, 0); SerialPort.print(",");
  SerialPort.println(att.elError, 0);
}

void printAzEl() {
  SerialPort.print("AZ");
  SerialPort.print((att.az < 0) ? (att.az + 360) : att.az, 1);
  SerialPort.print(" EL");
  SerialPort.print(att.el, 1);
  SerialPort.print("\n");
}

void printAz() {
  SerialPort.print("AZ");
  SerialPort.print((att.az < 0) ? (att.az + 360) : att.az, 1);
  SerialPort.print("\n");
}

void printEl() {
  SerialPort.print("EL");
  SerialPort.print(att.el, 1);
  SerialPort.print("\n");
}

void calibrate() {
  bool changed = lsm.calibrate();
  if (changed) {
    digitalWrite(spkPin, HIGH);
    printCal();
  } else {
    digitalWrite(spkPin, LOW);
  }
}

// 重命名：getAzElDemo -> updateDemoAngles
void updateDemoAngles(float *azSet, float *elSet, float *azInc, float *elInc) {
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  SerialPort.print(*azSet, 0); SerialPort.print(",");
  SerialPort.println(*elSet, 0);
}

// ============================================================
//  电机驱动
// ============================================================
void processMotors() {
/* //PWM模式下没有brake引脚
  bool azShouldBrake = (abs(att.azError) < 1.0);
  bool elShouldBrake = (abs(att.elError) < 1.0);
  
  if (azShouldBrake || elShouldBrake) {
    digitalWrite(azBrkPin, HIGH);
  } else {
    digitalWrite(azBrkPin, LOW);
  }
*/  
  azMot.drive(att.azError);
  elMot.drive(att.elError);
}

// ============================================================
//  串口命令处理
// ============================================================
void processCommands(void) {
  while (SerialPort.available() > 0) {
    char ch = SerialPort.read();
    switch (ch) {
      case 13:
        processUserCommands(line);
        line = "";
        break;
      case 10:
        processEasycommCommands(line);
        line = "";
        break;
      default:
        line += ch;
        break;
    }
  }
}

// 修复：使用 yield() 替代 delay(1)，减少阻塞
String readSerialLine() {
  String result = "";
  unsigned long start = millis();
  while (true) {
    yield();
    if (SerialPort.available() > 0) {
      char c = SerialPort.read();
      if (c == '\n' || c == '\r') {
        if (result.length() > 0) return result;
      } else {
        result += c;
      }
    }
    if (millis() - start > 60000) return "";
    delay(10);
  }
}

// ============================================================
//  配置管理
// ============================================================
// 修复：使用 offsetof 计算 CRC 长度
uint16_t calcCRC16(RotorConfig *c) {
  uint8_t *p = (uint8_t *)c;
  uint16_t crc = 0xFFFF;
  size_t len = offsetof(RotorConfig, checksum);  // 从结构体开头到 checksum 字段的偏移量
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0x8408;
      else crc >>= 1;
    }
  }
  return crc;
}

void defaultConfig() {
  cfg.magic = CONFIG_MAGIC;
  cfg.extSensor = 0;
  cfg.azGain = 25;
  cfg.elGain = 25;
  memset(cfg.staSsid, 0, sizeof(cfg.staSsid));
  memset(cfg.staPass, 0, sizeof(cfg.staPass));
  cfg.checksum = calcCRC16(&cfg);
  configDirty = true;
}

void applyConfig() {
  azGain = cfg.azGain;
  elGain = cfg.elGain;
  azMot.setGain(azGain);
  elMot.setGain(elGain);
}

void loadConfig() {
  EEPROM.get(CONFIG_ADDR, cfg);
  if (cfg.magic != CONFIG_MAGIC || cfg.checksum != calcCRC16(&cfg)) {
    SerialPort.println("Config invalid, using defaults");
    defaultConfig();
    saveConfig();
  } else {
    applyConfig();
    SerialPort.printf("Config loaded: extSensor=%d azGain=%d elGain=%d\n",
      cfg.extSensor, cfg.azGain, cfg.elGain);
  }
}

void saveConfig() {
  cfg.checksum = calcCRC16(&cfg);
  EEPROM.put(CONFIG_ADDR, cfg);
  EEPROM.commit();
  SerialPort.println("Config saved to EEPROM");
  configDirty = false;
  lastConfigSaveTime = millis();
}

void markConfigDirty() {
  configDirty = true;
  lastConfigSaveTime = millis();
}

void checkSaveConfig() {
  if (configDirty && (millis() - lastConfigSaveTime > 5000)) {
    saveConfig();
  }
}

// ============================================================
//  WiFi & Web
// ============================================================
void initOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() { SerialPort.println("OTA start"); });
  ArduinoOTA.onEnd([]() { SerialPort.println("\nOTA end - rebooting"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    SerialPort.printf("OTA Progress: %u%%\r", (p * 100) / t);
    yield();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SerialPort.printf("OTA Error[%u]: %s\n", error,
      (error == OTA_AUTH_ERROR) ? "Auth Failed" :
      (error == OTA_BEGIN_ERROR) ? "Begin Failed" :
      (error == OTA_CONNECT_ERROR) ? "Connect Failed" :
      (error == OTA_RECEIVE_ERROR) ? "Receive Failed" :
      (error == OTA_END_ERROR) ? "End Failed" : "Unknown");
  });
  ArduinoOTA.begin();
  SerialPort.print("OTA ready: ");
  SerialPort.print(WiFi.localIP());
  SerialPort.println(":8266");
}

bool connectSTA() {
  if (strlen(cfg.staSsid) > 0) {
    SerialPort.printf("Trying saved SSID: %s\n", cfg.staSsid);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfg.staSsid, cfg.staPass);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < CONFIG_TIMEOUT) {
      delay(500);
      SerialPort.print(".");
      yield();
    }
    if (WiFi.status() == WL_CONNECTED) {
      SerialPort.printf("\nSTA OK IP:%s\n", WiFi.localIP().toString().c_str());
      runMode = STA_MODE;
      return true;
    }
    SerialPort.println("\nSaved SSID failed, entering config mode");
  }
  return false;
}

void startAP(String apSSID) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID.c_str(), AP_PASS);
  SerialPort.printf("AP OK SSID:%s IP:192.168.4.1\n", apSSID.c_str());
  runMode = AP_MODE;
}

void startScan() {
  if (!scanRunning) {
    scanRunning = true;
    scanStartTime = millis();
    WiFi.scanNetworks(true);
    SerialPort.println("WiFi scan started");
  }
}

void handleScan() {
  if (scanRunning) {
    int n = WiFi.scanComplete();
    if (n >= 0) {
      String json = "{\"aps\":[";
      for (int i = 0; i < n; i++) {
        if (i > 0) json += ",";
        String ssid = WiFi.SSID(i);
        ssid.replace("\\", "\\\\");
        ssid.replace("\"", "\\\"");
        json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
      }
      json += "]}";
      scanResult = json;
      WiFi.scanDelete();
      scanRunning = false;
      SerialPort.printf("Scan complete: %d networks found\n", n);
    } else if (millis() - scanStartTime > 15000) {
      scanResult = "{\"aps\":[]}";
      WiFi.scanDelete();
      scanRunning = false;
      SerialPort.println("Scan timeout");
    }
  }
}

String generatePage() {
  String page = String(index_html);
  
  if (runMode == STA_MODE) {
    page.replace("%STA_IP%", WiFi.localIP().toString());
    page.replace("%STA_MAC%", WiFi.macAddress());
  } else {
    page.replace("%STA_IP%", "Not connected");
    page.replace("%STA_MAC%", WiFi.softAPmacAddress());
  }
  
  if (cfg.extSensor) {
    page.replace("%SENSOR_NO%", "");
    page.replace("%SENSOR_YES%", "checked");
  } else {
    page.replace("%SENSOR_NO%", "checked");
    page.replace("%SENSOR_YES%", "");
  }
  
  page.replace("%AZ_GAIN%", String(cfg.azGain));
  page.replace("%EL_GAIN%", String(cfg.elGain));
  page.replace("%STA_SSID%", String(cfg.staSsid));
  page.replace("%STA_PASS%", String(cfg.staPass));
  page.replace("%AP_OPTIONS%", "<option value=\"\">-- 刷新 --</option>");
  page.replace("%FIRMWARE_VERSION%", String(FIRMWARE_VERSION));
  return page;
}

void setupHTTP() {
  http.on("/", HTTP_GET, []() {
    http.send(200, "text/html", generatePage());
  });
  
  http.on("/scan", HTTP_GET, []() {
    if (scanRunning) {
      http.send(202, "text/plain", "Scanning...");
    } else if (scanResult.length() > 0) {
      http.send(200, "application/json", scanResult);
      scanResult = "";
    } else {
      startScan();
      http.send(202, "text/plain", "Scan started");
    }
  });
  
  http.on("/config", HTTP_POST, []() {
    String body = http.arg("plain");
    SerialPort.printf("Config received: %s\n", body.c_str());
    
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      SerialPort.printf("JSON parse error: %s\n", error.c_str());
      http.send(400, "text/plain", "Invalid JSON");
      return;
    }
    
    cfg.extSensor = doc["extSensor"] | 0;
    cfg.azGain = doc["azGain"] | 25;
    cfg.elGain = doc["elGain"] | 25;
    
    const char* ssid = doc["staSsid"] | "";
    const char* pass = doc["staPass"] | "";
    strlcpy(cfg.staSsid, ssid, sizeof(cfg.staSsid));
    strlcpy(cfg.staPass, pass, sizeof(cfg.staPass));
    
    saveConfig();
    applyConfig();
    
    http.send(200, "text/plain", "OK");
    
    if (strlen(cfg.staSsid) > 0 && runMode == AP_MODE) {
      needRestart = true;
    }
  });
  
  http.on("/cmd", HTTP_POST, []() {
    String cmd = http.arg("plain");
    cmd.trim();
    if (cmd.startsWith("AZ") || cmd.startsWith("EL") || 
        cmd.startsWith("XZ") || cmd.startsWith("XL")) {
      processEasycommCommands(cmd);
    } else {
      processUserCommands(cmd);
    }
    http.send(200, "text/plain", "OK\n");
  });
  
  http.on("/AzEl", HTTP_GET, []() {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.1f,%.1f\n", att.az, att.el);
    http.send(200, "text/plain", buf);
  });
}

void handleTCP() {
  static WiFiClient client;
  if (!client || !client.connected()) {
    client = tcpServer.available();
    return;
  }
  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      processEasycommCommands(tcpLine);
      tcpLine = "";
    } else {
      tcpLine += c;
    }
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  switch (type) {
    case WStype_DISCONNECTED:
      SerialPort.printf("WS[%u] disconnect\n", num);
      break;
    case WStype_CONNECTED:
      SerialPort.printf("WS[%u] connect from %s\n", num, webSocket.remoteIP(num).toString().c_str());
      break;
    case WStype_TEXT: {
      String cmd = "";
      for (size_t i = 0; i < len; i++) cmd += (char)payload[i];
      cmd.trim();
      if (cmd.startsWith("AZ") || cmd.startsWith("EL") || 
          cmd.startsWith("XZ") || cmd.startsWith("XL")) {
        processEasycommCommands(cmd);
      } else {
        processUserCommands(cmd);
      }
      break;
    }
    default:
      break;
  }
}

void broadcastDash() {
  char json[64];
  snprintf(json, sizeof(json), "{\"az\":%.1f,\"el\":%.1f}", att.az, att.el);
  webSocket.broadcastTXT(json);
}

// ============================================================
//  Setup & Loop
// ============================================================
void setup() {
  SerialPort.begin(115200);
  delay(100);
  
  Wire.setClock(400000);
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();
  
  pinMode(spkPin, OUTPUT);
  //pinMode(azBrkPin, OUTPUT);
  //digitalWrite(azBrkPin, LOW);
  
  // 开机启动提示音（两长音）
  for (int i = 0; i < 2; i++) {
    digitalWrite(spkPin, HIGH);
    delay(200);
    digitalWrite(spkPin, LOW);
    delay(300);
  }
  delay(100);
  
  WiFi.macAddress(mac);
  String apSSID = String(AP_SSID_PREFIX) + String(mac[4], HEX) + String(mac[5], HEX);
  
  bool forceAP = (digitalRead(0) == LOW);
  if (forceAP) {
    startAP(apSSID);
  } else {
    if (!connectSTA()) startAP(apSSID);
  }
  
  if (runMode == STA_MODE) initOTA();
  tcpServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  setupHTTP();
  
  reset(true);
  lsm.begin();
}

void loop() {
  yield();
  
  processCommands();
  handleTCP();
  http.handleClient();
  webSocket.loop();
  
  handleScan();
  checkSaveConfig();
  
  if (t1.tick()) {
    processPosition();
  }
  
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > 500) {
    lastBroadcast = millis();
    broadcastDash();
  }
  
  ArduinoOTA.handle();
  processMotors();
  
  if (needRestart) {
    needRestart = false;
    delay(1000);
    ESP.restart();
  }
}