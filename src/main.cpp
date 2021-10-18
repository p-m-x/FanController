#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <SDA5708.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// #define DEBUG

#define ONE_WIRE_BUS 3
#define TEMP_SENSOR_RESOLUTION 12
#define MAX_SENSORS_COUNT 2
#define PWM_OUT_PIN 8
#define PWM_MIN_DUTY_CYCLE 25
#define PWM_MAX_DUTY_CYCLE 255
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define SCREEN_ADDRESS 0x3c
#define MODBUS_REG_START_ADDRESS 0x00
#define MODBUS_OFFSET_DEV_ADDR 0
#define MODBUS_OFFSET_MAX_TEMP 1
#define MODBUS_OFFSET_TEMP_HYSTERESIS 2
#define MODBUS_OFFSET_FAN_SPEED 3
#define MODBUS_DEFAULT_SLAVE_ADDR 20
#define CONFIG_HASH "gtrfdokyp"

union
{
  float value;
  struct
  {
    uint16_t lowOrderByte;
    uint16_t highOrderByte;
  };
} temperatureUnion;

struct Config
{
  char hash[10];
  uint8_t tempThreshold;
  uint8_t tempHysteresis;
  int modbusSlaveAddr;
};

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float temperatures[MAX_SENSORS_COUNT];
uint8_t sensorsCount;
float currentMainTemp = 0;
float lastMainTemp = 0;
int tempErrorIdx = 0; // starts from 1
int temperatureTrend = 0;
bool firstLoop = true;
Config cfg = {};



void readTemperatures(void);
void adjustFanSpeed(void);
void printSpeedBar(int percentage);
void drawArrowUp(uint8_t x, uint8_t y);
void drawArrowDown(uint8_t x, uint8_t y);
void printMainTemperature();
void readConfig();
void writeConfig();
void setConfigDefaults();
void updateModbusRegisters();
void (*resetFunc)(void) = 0;

Ticker readTemperatureTicker(readTemperatures, 750 / (1 << (12 - TEMP_SENSOR_RESOLUTION)), 0, MILLIS);
Ticker adjustFanSpeedTicker(adjustFanSpeed, 1000, 0, MILLIS);
SDA5708 display(4, 5, 6, 7);

void setup()
{
  delay(1000);
  #ifdef DEBUG
  Serial.begin(9600);
  while(!Serial){}
  #endif
  
  readConfig();
  if (strcmp(cfg.hash, CONFIG_HASH) != 0) {
    setConfigDefaults();
    writeConfig();
  }

  // start the Modbus RTU server, with (slave) id 42
  if (!ModbusRTUServer.begin(cfg.modbusSlaveAddr, 9600)) {
    while (1);
  }
  ModbusRTUServer.configureInputRegisters(MODBUS_REG_START_ADDRESS, MAX_SENSORS_COUNT*2);
  ModbusRTUServer.configureHoldingRegisters(MODBUS_REG_START_ADDRESS, 4);
  updateModbusRegisters();

  display.begin();
  display.brightness(0);
  display.print("Fan");
  delay(1000);
  display.print("Control");
  delay(1000);
  display.clear();
  display.print("v1.0.0");
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);

  sensors.begin();
  sensorsCount = sensors.getDS18Count();
  display.clear();
  display.print("SENS: ");
  display.printAt(String(sensorsCount).c_str(), 6);

  if (sensorsCount == 0) return;
  delay(2000);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); // makes it async
  
  
  display.clear();
  readTemperatureTicker.start();
  adjustFanSpeedTicker.start();

  // first modbus poll is time consuming, call it before set wdt_enable
  ModbusRTUServer.poll();
  wdt_enable(WDTO_2S);
}


void loop()
{
  readTemperatureTicker.update();
  adjustFanSpeedTicker.update();
  ModbusRTUServer.poll();
  
  bool saveConfig = false;
  bool resetController = false;
  long modbusSlaveAddr = ModbusRTUServer.holdingRegisterRead(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_DEV_ADDR);
  if (modbusSlaveAddr != cfg.modbusSlaveAddr)
  {
    cfg.modbusSlaveAddr = modbusSlaveAddr;
    resetController = true;
    saveConfig = true;
  }
  long tempThreshold = ModbusRTUServer.holdingRegisterRead(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_MAX_TEMP);
  if (tempThreshold != cfg.tempThreshold) {
    if (tempThreshold > 125) tempThreshold = 125;
    if (tempThreshold < 0) tempThreshold = 0;
    cfg.tempThreshold = tempThreshold;
    saveConfig = true;
  }

  long tempHysteresis = ModbusRTUServer.holdingRegisterRead(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_TEMP_HYSTERESIS);
  if (tempHysteresis != cfg.tempHysteresis) {
    if (tempHysteresis > 125) tempHysteresis = 125;
    if (tempHysteresis < 0) tempHysteresis = 0;
    cfg.tempHysteresis = tempHysteresis;
    saveConfig = true;
  }
  
  if (saveConfig) {
    saveConfig = false;
    writeConfig();
    updateModbusRegisters();
    if (resetController) {
      resetController = false;
      resetFunc();
    }
  }

  wdt_reset();
}

void readTemperatures(void)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  tempErrorIdx = 0;
  currentMainTemp = -127;
  for (int t = 0; t < min(sensorsCount, MAX_SENSORS_COUNT); t++)
  {
    DeviceAddress addr;
    sensors.getAddress(addr, t);
    if (!sensors.isConnected(addr)){
      tempErrorIdx = t + 1;
      temperatures[t] = -127;
      
    } else {
      temperatures[t] = sensors.getTempC(addr);
      temperatureUnion.value = temperatures[t];
      ModbusRTUServer.inputRegisterWrite(MODBUS_REG_START_ADDRESS + (t * 2), temperatureUnion.highOrderByte);
      ModbusRTUServer.inputRegisterWrite(MODBUS_REG_START_ADDRESS + (t * 2 + 1), temperatureUnion.lowOrderByte);
    }
    if (temperatures[t] > currentMainTemp) {
      currentMainTemp = temperatures[t];
    }
  }
  
  sensors.requestTemperatures();
}

void adjustFanSpeed(void)
{
  
  temperatureTrend = (int)(currentMainTemp * 10) - (int)(lastMainTemp * 10);
  lastMainTemp = currentMainTemp;

  int8_t percent = 0;
  long dutyCycle = 0;
  if (tempErrorIdx > 0) {
    dutyCycle = PWM_MAX_DUTY_CYCLE;
    percent = 100;
  } 
  else if (currentMainTemp >= cfg.tempThreshold - cfg.tempHysteresis)
  {
    dutyCycle = map(currentMainTemp * 100, (cfg.tempThreshold - cfg.tempHysteresis) * 100, cfg.tempThreshold * 100, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);

    dutyCycle = min(dutyCycle, PWM_MAX_DUTY_CYCLE);
    percent = map(dutyCycle, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE, 0, 100);
  }
  ModbusRTUServer.holdingRegisterWrite(MODBUS_REG_START_ADDRESS+MODBUS_OFFSET_FAN_SPEED, percent);
  
  analogWrite(PWM_OUT_PIN, dutyCycle);

  printMainTemperature();

  printSpeedBar(percent);
  
}

void printSpeedBar(int percent)
{
  display.setCyrsor(7);
  long bars = map(percent, 0, 100, 0, 7);

  for (uint8_t i = 0; i <= 7; i++) 
  {
    if (7 - i <= bars) {
      display.sendByte(0b11111111 / 8);
    } else {
      display.sendByte(0b00000000 / 8);
    }
  }
}

void drawArrowUp(uint8_t x, uint8_t y)
{
}

void drawArrowDown(uint8_t x, uint8_t y)
{
}

void printMainTemperature()
{
  char value[8] = "";
  if (tempErrorIdx > 0) {
    display.clear();
    sprintf(value, "ERR T%d", tempErrorIdx);
    
  } else {
    sprintf(value, "%d.%d", (int)currentMainTemp, int(currentMainTemp * 10) % 10);

    if (temperatureTrend >= 0)
    {
      drawArrowUp(2, 28);
    }
    if (temperatureTrend <= 0)
    {
      drawArrowDown(2, 38);
    }
    display.print(value);
    display.digit(127, 4);
    display.printAt("   ", 5);
  } 
}

void readConfig() {

  int ee = 0;
  byte* p = (byte*)(void*)&cfg;
    unsigned int i;
    for (i = 0; i < sizeof(cfg); i++)
          *p++ = EEPROM.read(ee++);

  #ifdef DEBUG
  Serial.println("READ");
  Serial.print("modbusSlaveAddr: ");
  Serial.println(cfg.modbusSlaveAddr);
  Serial.print("tempThreshold: ");
  Serial.println(cfg.tempThreshold);
  Serial.print("tempHysteresis:");
  Serial.println(cfg.tempHysteresis);
  Serial.print("hash: ");
  Serial.println(cfg.hash);
  #endif
}

void writeConfig() {

  #ifdef DEBUG
  Serial.println("WRITE");
  Serial.print("modbusSlaveAddr: ");
  Serial.println(cfg.modbusSlaveAddr);
  Serial.print("tempThreshold: ");
  Serial.println(cfg.tempThreshold);
  Serial.print("tempHysteresis:");
  Serial.println(cfg.tempHysteresis);
  Serial.print("hash: ");
  Serial.println(cfg.hash);
  #endif

  int ee = 0;
  const byte* p = (const byte*)(const void*)&cfg;
    unsigned int i;
    for (i = 0; i < sizeof(cfg); i++)
          EEPROM.put(ee++, *p++);
}

void setConfigDefaults() {
  strcpy(cfg.hash, CONFIG_HASH);
  cfg.tempThreshold = 30;
  cfg.tempHysteresis = 5;
  cfg.modbusSlaveAddr = MODBUS_DEFAULT_SLAVE_ADDR;
}

void updateModbusRegisters() {
  ModbusRTUServer.holdingRegisterWrite(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_DEV_ADDR, cfg.modbusSlaveAddr);
  ModbusRTUServer.holdingRegisterWrite(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_MAX_TEMP, cfg.tempThreshold);
  ModbusRTUServer.holdingRegisterWrite(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_TEMP_HYSTERESIS, cfg.tempHysteresis);
}