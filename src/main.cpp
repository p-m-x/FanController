#define RS845_DEFAULT_DE_PIN 3
#define RS845_DEFAULT_RE_PIN -1
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <SDA5708.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <avr/wdt.h>

#define ONE_WIRE_BUS 6
#define TEMP_SENSOR_RESOLUTION 12
#define MAX_SENSORS_COUNT 2
#define PWM_OUT_PIN 7
#define PWM_MIN_DUTY_CYCLE 25
#define PWM_MAX_DUTY_CYCLE 255
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define SCREEN_ADDRESS 0x3c

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float temperatures[MAX_SENSORS_COUNT];
uint8_t sensorsCount;
float tempThreshold = 33.0;
float tempHysteresis = 5;
float currentMainTemp = 0;
float lastMainTemp = 0;
int tempErrorIdx = 0; // starts from 1
int temperatureTrend = 0;

void readTemperatures(void);
void adjustFanSpeed(void);
void printSpeedBar(int percentage);
void drawArrowUp(uint8_t x, uint8_t y);
void drawArrowDown(uint8_t x, uint8_t y);
void printMainTemperature();

Ticker readTemperatureTicker(readTemperatures, 750 / (1 << (12 - TEMP_SENSOR_RESOLUTION)), 0, MILLIS);
Ticker adjustFanSpeedTicker(adjustFanSpeed, 1000, 0, MILLIS);
SDA5708 display(2, 3, 4, 5);

void setup()
{
  display.begin();
  display.brightness(0);
  display.print("Fan");
  delay(1000);
  display.print("Control");
  delay(1000);
  display.clear();
  display.print("v1.0.0");
  delay(1000);
  
  sensors.begin();
  sensorsCount = sensors.getDS18Count();
  display.clear();
  display.print("SENS: ");
  display.printAt(String(sensorsCount).c_str(), 6);

  if (sensorsCount == 0) return;
  delay(2000);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); // makes it async
  wdt_enable(WDTO_500MS);
  
  ModbusRTUServer.begin(20, 9600);
  ModbusRTUServer.configureHoldingRegisters(0, 1);

  display.clear();
  readTemperatureTicker.start();
  adjustFanSpeedTicker.start();
}

void loop()
{
  readTemperatureTicker.update();
  adjustFanSpeedTicker.update();
  wdt_reset();
}

void readTemperatures(void)
{
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
    }
    if (temperatures[t] > currentMainTemp) {
      currentMainTemp = temperatures[t];
    }
  }
  sensors.requestTemperatures();
  Serial.println(currentMainTemp);
}

void adjustFanSpeed(void)
{
  
  temperatureTrend = (int)(currentMainTemp * 10) - (int)(lastMainTemp * 10);
  lastMainTemp = currentMainTemp;

  long percent = 0;
  long dutyCycle = 0;
  if (tempErrorIdx > 0) {
    dutyCycle = PWM_MAX_DUTY_CYCLE;
    percent = 100;
  } 
  else if (currentMainTemp >= tempThreshold - tempHysteresis)
  {
    dutyCycle = map(currentMainTemp * 100, (tempThreshold - tempHysteresis) * 100, tempThreshold * 100, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);

    dutyCycle = min(dutyCycle, PWM_MAX_DUTY_CYCLE);
    percent = map(dutyCycle, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE, 0, 100);
  }
  
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