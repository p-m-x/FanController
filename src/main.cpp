#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// #define DEBUG

#define ONE_WIRE_BUS 3
#define TEMP_SENSOR_RESOLUTION 12
#define MAX_SENSORS_COUNT 2
#define PWM_OUT_PIN 9
#define PWM_MIN_DUTY_CYCLE 25
#define PWM_MAX_DUTY_CYCLE 255
#define MODBUS_REG_START_ADDRESS 0x00
#define MODBUS_OFFSET_DEV_ADDR 0
#define MODBUS_OFFSET_MAX_TEMP 1
#define MODBUS_OFFSET_TEMP_HYSTERESIS 2
#define MODBUS_OFFSET_FAN_SPEED 3
#define MODBUS_OFFSET_ERROR 4
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
bool tempSensError = false;
bool firstLoop = true;
Config cfg = {};

void readTemperatures(void);
void adjustFanSpeed(void);
void readConfig();
void writeConfig();
void setConfigDefaults();
void updateModbusRegisters();
void (*resetFunc)(void) = 0;

Ticker readTemperatureTicker(readTemperatures, 750 / (1 << (12 - TEMP_SENSOR_RESOLUTION)), 0, MILLIS);
Ticker adjustFanSpeedTicker(adjustFanSpeed, 1000, 0, MILLIS);

void setup()
{
  #ifdef DEBUG
  delay(1000);
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println(F("MODBUS RS485 Fan Controller v.1.0.0"));
  #endif

  sensors.begin();
  sensorsCount = sensors.getDS18Count();

#ifdef DEBUG
  Serial.print(F("Found: "));
  Serial.print(sensorsCount);
  Serial.println(F(" temperature sensor(s)"));
#endif

  if (sensorsCount == 0) {
    Serial.println("Set max fan speed!");
    return;
  };

  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); // makes it async
  
  readConfig();
  if (strcmp(cfg.hash, CONFIG_HASH) != 0) {
    setConfigDefaults();
    writeConfig();
  }

  // start the Modbus RTU server, with (slave) id 42
  if (!ModbusRTUServer.begin(20, 9600)) {
    #ifdef DEBUG
    Serial.println("MODBUS not initialized");
    #endif
    return;
  }
  ModbusRTUServer.configureInputRegisters(MODBUS_REG_START_ADDRESS, MAX_SENSORS_COUNT*2);
  ModbusRTUServer.configureHoldingRegisters(MODBUS_REG_START_ADDRESS, 5);
  updateModbusRegisters();

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
  currentMainTemp = -127;
  for (int t = 0; t < min(sensorsCount, MAX_SENSORS_COUNT); t++)
  {
    DeviceAddress addr;
    sensors.getAddress(addr, t);
    if (!sensors.isConnected(addr)){
      #ifdef DEBUG
      Serial.print("Temp. sensor #");
      Serial.print(t + 1);
      Serial.println(" error");
      #endif
      temperatures[t] = -127;    
      tempSensError = true;  
    } else {
      temperatures[t] = sensors.getTempC(addr);
      temperatureUnion.value = temperatures[t];
      ModbusRTUServer.inputRegisterWrite(MODBUS_REG_START_ADDRESS + (t * 2), temperatureUnion.highOrderByte);
      ModbusRTUServer.inputRegisterWrite(MODBUS_REG_START_ADDRESS + (t * 2 + 1), temperatureUnion.lowOrderByte);
      tempSensError = false;
    }
    ModbusRTUServer.holdingRegisterWrite(MODBUS_REG_START_ADDRESS + MODBUS_OFFSET_ERROR, tempSensError);
    if (temperatures[t] > currentMainTemp) {
      currentMainTemp = temperatures[t];
    }
  }
  
  sensors.requestTemperatures();
}

void adjustFanSpeed(void)
{
  long percent = 0;
  long dutyCycle = 0;
  if (tempSensError) {
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
  #ifdef DEBUG
  if (lastMainTemp != currentMainTemp) {
    Serial.print(F("PWM: "));
    Serial.print(percent);
    Serial.print(F("% temp: "));
    Serial.print(currentMainTemp);
    Serial.println("st. C");
  }
  #endif
  lastMainTemp = currentMainTemp;
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
