#include <Arduino.h>
#include <Wire.h>
#include "INA238.h"
#include <SPI.h>
#include <Mcp320x.h>
#include <Servo.h>

#define SPI_CS       8
#define ADC_VREF     5000
#define ADC_CLK      1600000
#define ESC_PIN      9
#define PULSE_INPUT  2
#define ADC_INPUT    A1

const uint8_t INA238_ADDRESS1 = 0x40;
INA238 powerMonitor(INA238_ADDRESS1);
MCP3204 adc(ADC_VREF, SPI_CS);
Servo esc;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long prevPulseTime = 0;
volatile bool newPulse = false;
unsigned long lastReportTime = 0;
const float pulsesPerRevolution = 1.0;

void setup() {
  initializeSerial();
  initializeINA238();
  initializeSPI();
  initializeRPMMeasurement();
  initializeESC();
}

void loop() {
  reportINA238Data();
  readAndReportADC();
  calculateAndReportRPMandPWM();
  Serial.println("-----------------------");
  delay(1000);
}

void initializeSerial() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("プログラム開始");
}

void initializeINA238() {
  if (!powerMonitor.begin()) {
    Serial.println("INA238が見つかりません。配線を確認してください。");
    while (1);
  }
  if (!powerMonitor.configure()) {
    Serial.println("INA238の設定に失敗しました。");
  }
  if (!powerMonitor.adcConfigure()) {
    Serial.println("INA238のADC設定に失敗しました。");
  }
  reportShuntCalibrationData();
}

void reportShuntCalibrationData() {
  uint16_t shuntCalibrationData;
  if (powerMonitor.readShuntCal(shuntCalibrationData)) {
    Serial.print("INA238 Shunt Calibration Data: ");
    Serial.println(shuntCalibrationData);
  } else {
    Serial.println("Error reading shunt calibration data.");
  }
}

void initializeSPI() {
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(ADC_CLK, MSBFIRST, SPI_MODE0));
}

void initializeRPMMeasurement() {
  pinMode(PULSE_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_INPUT), pulseDetected, FALLING);
}

void initializeESC() {
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(5000);
}

void reportINA238Data() {
  Serial.print("Current: ");
  Serial.print(powerMonitor.readCurrent());
  Serial.println(" mA");
  // その他のINA238データの出力も同様に行います。
}

void readAndReportADC() {
  uint32_t t1 = micros();
  uint16_t raw = adc.read(MCP3204::Channel::SINGLE_0);
  uint32_t t2 = micros();
  reportADCValue(raw, adc.toAnalog(raw), t2 - t1);
}

void reportADCValue(uint16_t raw, uint16_t val, uint32_t sampleTime) {
  Serial.print("ADC Value: ");
  Serial.print(raw);
  Serial.print(" (");
  Serial.print(val);
  Serial.println(" mV)");
  Serial.print("Sampling time: ");
  Serial.print(static_cast<double>(sampleTime) / 1000, 4);
  Serial.println(" ms");
}

void calculateAndReportRPMandPWM() {
  int pwmValue = map(analogRead(ADC_INPUT), 0, 1023, 1000, 2000);
  esc.writeMicroseconds(pwmValue);
  Serial.print("PWM: ");
  Serial.print(pwmValue);
  Serial.println(" us");
  reportRPM();
}

void reportRPM() {
  if (newPulse) {
    unsigned long timeDifference = lastPulseTime - prevPulseTime;
    float rpm = (timeDifference > 0) ? (60000000.0 / timeDifference) / pulsesPerRevolution : 0;
    newPulse = false;
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
}

void pulseDetected() {
  prevPulseTime = lastPulseTime;
  lastPulseTime = micros();
  newPulse = true;
}
