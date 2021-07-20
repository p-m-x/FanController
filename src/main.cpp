#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>      // Add a custom font
#include <Fonts/FreeSans12pt7b.h>     // Add a custom font
#include <Fonts/FreeSansBold18pt7b.h> // Add a custom font

#define ONE_WIRE_BUS 2
#define TEMP_SENSOR_RESOLUTION 12
#define MAX_SENSORS_COUNT 2
#define PWM_OUT_PIN 6
#define PWM_MIN_DUTY_CYCLE 70
#define PWM_MAX_DUTY_CYCLE 255
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3c

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);

float temperatures[MAX_SENSORS_COUNT];
uint8_t sensorsCount;
float tempThreshold = 33.0;
float tempHysteresis = 5;
float lastMaxTemp = 0;
long dutyCycle = 0; // OFF

void readTemperatures(void);
void adjustFanSpeed(void);
void printSpeedBar(int percentage);
void drawArrowUp(uint8_t x, uint8_t y);
void drawArrowDown(uint8_t x, uint8_t y);

Ticker readTemperatureTicker(readTemperatures, 750 / (1 << (12 - TEMP_SENSOR_RESOLUTION)), 0, MILLIS);
Ticker adjustFanSpeedTicker(adjustFanSpeed, 1000, 0, MILLIS);

void setup()
{
  delay(100);
  sensors.begin();
  sensorsCount = sensors.getDS18Count();

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.dim(0);
  display.display();

  display.setCursor(0, 20);
  display.print("Fan controller");
  display.setCursor(0, 30);
  display.print("ver. 0.0.1");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(0, 20);
  display.print("Found: ");
  display.print(sensorsCount);
  display.print(" sensor(s)");
  display.display();
  delay(2000);

  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); // makes it async

  readTemperatureTicker.start();
  adjustFanSpeedTicker.start();
}

void loop()
{
  readTemperatureTicker.update();
  adjustFanSpeedTicker.update();
}

void readTemperatures(void)
{
  for (int t = 0; t < min(sensorsCount, MAX_SENSORS_COUNT); t++)
  {
    temperatures[t] = sensors.getTempCByIndex(t);
  }
  sensors.requestTemperatures();
}

void adjustFanSpeed(void)
{
  float maxTemp = 0;
  for (int t = 0; t < min(sensorsCount, MAX_SENSORS_COUNT); t++)
  {
    if (maxTemp < temperatures[t])
    {
      maxTemp = temperatures[t];
    }
  }
  int trend = (int)(maxTemp * 10) - (int)(lastMaxTemp * 10);
  lastMaxTemp = maxTemp;

  long percent = 0;
  if (maxTemp >= tempThreshold - tempHysteresis)
  {
    if (dutyCycle < PWM_MIN_DUTY_CYCLE)
    {
      analogWrite(PWM_OUT_PIN, PWM_MAX_DUTY_CYCLE);
      delay(500);
    }
    dutyCycle = map(maxTemp * 100, (tempThreshold - tempHysteresis) * 100, tempThreshold * 100, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE);

    dutyCycle = min(dutyCycle, PWM_MAX_DUTY_CYCLE);
    percent = map(dutyCycle, PWM_MIN_DUTY_CYCLE, PWM_MAX_DUTY_CYCLE, 0, 100);
  }
  else
  {
    dutyCycle = 0;
  }
  analogWrite(PWM_OUT_PIN, dutyCycle);

  display.clearDisplay();
  display.setFont(&FreeSansBold18pt7b);
  display.setCursor(15, 44);
  char temp[40] = "";
  sprintf(temp, "%d.%d", (int)maxTemp, int(maxTemp * 10) % 10);
  display.print(temp);
  display.setFont(&FreeSans12pt7b);

  display.drawCircle(89, 24, 3, WHITE);
  display.setCursor(95, 38);
  display.print("C");

  display.setFont(NULL);
  display.setCursor(10, 55);
  display.print("T1: ");
  sprintf(temp, "%d.%d", (int)temperatures[0], int(temperatures[0] * 10) % 10);
  display.print(temp);
  display.print("   T2: ");

  printSpeedBar(percent);
  if (trend >= 0)
  {
    drawArrowUp(2, 28);
  }
  if (trend <= 0)
  {
    drawArrowDown(2, 38);
  }

  if (adjustFanSpeedTicker.counter() % 2 == 0)
  {
    display.fillCircle(3, 60, 3, WHITE);
  }
  display.display();
}

void printSpeedBar(int percent)
{
  uint8_t barOffset = 30;
  uint8_t barHeight = 12;
  uint8_t gap = 2;
  display.drawRect(barOffset, 0, SCREEN_WIDTH - barOffset, barHeight, WHITE);
  display.setCursor(2, 3);
  display.setFont(NULL);
  if (percent > 0)
  {
    display.print(percent);
    display.print("%");
    int progressBar = map(percent, 0, 100, 1, SCREEN_WIDTH - barOffset - gap * 2);
    display.fillRect(barOffset + gap, gap, progressBar, barHeight - gap * 2, WHITE);
  }
  else
  {
    display.print("OFF");
  }
}

void drawArrowUp(uint8_t x, uint8_t y)
{
  display.fillTriangle(x + 3, y, x + 6, y + 3, x, y + 3, WHITE);
}

void drawArrowDown(uint8_t x, uint8_t y)
{
  display.fillTriangle(x, y, x + 6, y, x + 3, y + 3, WHITE);
}