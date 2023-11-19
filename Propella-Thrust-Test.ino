// 実験装置メインプログラム
// 作成日: 2023-11-18
// 最終更新日: 2023-11-18

#include <Arduino.h>
#include <Wire.h>
#include "INA238.h"
#include <SPI.h>
#include <Mcp320x.h>
#include <Servo.h>

#define SPI_CS       8       // SPI slave select
#define ADC_VREF     5000    // 5V Vref
#define ADC_CLK      1600000 // SPI clock 1.6MHz
#define ESC_PIN      9       // ESCのピン番号
#define SERVO_PIN    3       // ダクト偏向用サーボのピン番号
#define PULSE_INPUT  2       // RPM計測用の入力ピン
#define ADC_INPUT_ESC    A1      // ESC操作用のアナログ入力ピン
#define ADC_INPUT_SERVO  A0      // サーボ操作用のアナログ入力ピン

// INA238のI2Cアドレスを指定する
const uint8_t INA238_ADDRESS1 = 0x40; // デバイス1のアドレス

// INA238 インスタンスを作成。I2CアドレスはデータシートまたはINA238.hで定義されているアドレスを使用する。
INA238 powerMonitor(INA238_ADDRESS1);

MCP3204 adc(ADC_VREF, SPI_CS);
Servo esc;
Servo duct;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long prevPulseTime = 0;
volatile bool newPulse = false;
unsigned long lastReportTime = 0;
const float pulsesPerRevolution = 1.0;
int pwmValueESC = 1000;
int pwmValueServo = 1000;

unsigned long previousMillis = 0;  // 前回のデータ読み取り時間
const long reportInterval = 500;  // 読み取り間隔（ミリ秒）

void setup() {
  initializeSerial();
  initializeINA238();
  initializeSPI();
  initializeRPMMeasurement();
  initializeESC();
  initializeServo();
}

void loop() {
  manageESC();
  manageServo();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= reportInterval) {
    reportINA238Data();
    readLoadcell();
    readMotorRPM();
    reportESC_Signal();
    reportServoSignal();
    Serial.println("");
    previousMillis = currentMillis;  // 最後の読み取り時間を更新
  }
}

void initializeSerial() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Start Program");
}

void initializeINA238() {
  if (!powerMonitor.begin()) {
    Serial.println("INA238が見つかりません。配線を確認してください。");
    while (1); // エラーが発生した場合は無限ループ
  }
  if (!powerMonitor.configure()) {
    Serial.println("INA238の設定に失敗しました。");
  }
  if (!powerMonitor.adcConfigure()) {
    Serial.println("INA238のADC設定に失敗しました。");
  }
  Serial.println("INA238 ADC設定完了");

  uint16_t shuntCalibrationData;
  bool success = powerMonitor.readShuntCal(shuntCalibrationData);
  if (success) {
    // キャリブレーションデータが有効であるため、シリアルポート経由で出力する
    Serial.print("INA238 Shunt Calibration Data: ");
    Serial.println(shuntCalibrationData);
  } else {
    // エラーが発生したため、シリアルポート経由でエラーメッセージを出力する
    Serial.println("Error reading shunt calibration data.");
  }
}

void initializeSPI() {  // ロードセルADC用のSPIピン設定
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings); 
}

void initializeRPMMeasurement() {
  // RPM計測用のピン設定
  pinMode(PULSE_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_INPUT), pulseDetected, FALLING);
}

void initializeESC() {
  // ESCの設定
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000); // ESCをアーム
  delay(5000); // スタートアップのための待ち時間
}

void initializeServo() {
  // ESCの設定
  duct.attach(SERVO_PIN);
  //duct.writeMicroseconds(1000);
}

void reportINA238Data() {
  // INA238からデータを読み取る
  float current = powerMonitor.readCurrent(); // 電流を読み取る
  float shunt = powerMonitor.readShuntVoltage(); // シャント電圧を読み取る
  float voltage = powerMonitor.readVoltage(); // 電圧を読み取る
  float power = powerMonitor.readPower(); // 電力を読み取る
  float temperature = powerMonitor.readTemperature(); // 温度を読み取る

  // 読み取ったデータをシリアルモニタに出力
  Serial.print("C:");
  Serial.print(current);
  Serial.print(",");

  Serial.print("SV:");
  Serial.print(shunt);
  Serial.print(",");

  Serial.print("V:");
  Serial.print(voltage);
  Serial.print(",");

  Serial.print("P:");
  Serial.print(power);
  Serial.print(",");

  Serial.print("T:");
  Serial.print(temperature);
  Serial.print(",");  
}

void readMotorRPM() {
  float rpm = 0;
  if (newPulse) {
    unsigned long timeDifference = lastPulseTime - prevPulseTime;
    if (timeDifference > 0) {
      rpm = (60000000.0 / timeDifference) / pulsesPerRevolution;
    }
    newPulse = false;
  }  
  Serial.print("RPM:");
  Serial.print(rpm);
  Serial.print(","); 
}

void manageESC() {
  int sensorValue = analogRead(ADC_INPUT_ESC);
  pwmValueESC = map(sensorValue, 0, 1023, 1000, 2000);
  esc.writeMicroseconds(pwmValueESC);
}

void reportESC_Signal() {
  Serial.print("ESC:");
  Serial.print(pwmValueESC);
  Serial.print(","); 
}

void reportServoSignal() {
  Serial.print("SRV:");
  Serial.print(pwmValueServo);
  //Serial.print(","); 
}

void manageServo() {
  int sensorValue = analogRead(ADC_INPUT_SERVO);
  pwmValueServo = map(sensorValue, 0, 1023, 1000, 2000);
  duct.writeMicroseconds(pwmValueServo);
}
void readLoadcell() {
  uint32_t t1 = micros();
  uint16_t raw = adc.read(MCP3204::Channel::SINGLE_0);
  uint32_t t2 = micros();
  uint16_t val = adc.toAnalog(raw);

  // ADC値の出力
  Serial.print("THST:");
  Serial.print(raw);
  Serial.print(",");
  //Serial.print(" (");
  //Serial.print(val);
  //Serial.println(" mV)");

  // サンプリング時間の出力
  //Serial.print("Sampling time: ");
  //Serial.print(static_cast<double>(t2 - t1) / 1000, 4);
  //Serial.println(" ms");
}

void pulseDetected() {
  prevPulseTime = lastPulseTime;
  lastPulseTime = micros();
  newPulse = true;
}
