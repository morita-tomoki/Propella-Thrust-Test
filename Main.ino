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
#define PULSE_INPUT  2       // RPM計測用の入力ピン
#define ADC_INPUT    A1      // ADC用のアナログ入力ピン

// INA238のI2Cアドレスを指定する
const uint8_t INA238_ADDRESS1 = 0x40; // デバイス1のアドレス

// INA238 インスタンスを作成。I2CアドレスはデータシートまたはINA238.hで定義されているアドレスを使用する。
INA238 powerMonitor(INA238_ADDRESS1);

MCP3204 adc(ADC_VREF, SPI_CS);
Servo esc;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long prevPulseTime = 0;
volatile bool newPulse = false;
unsigned long lastReportTime = 0;
const float pulsesPerRevolution = 1.0;


void setup() {
  // シリアル通信を開始
  Serial.begin(9600);
  while (!Serial) {
    ; // シリアルポートが利用可能になるまで待機
  }
  Serial.println("プログラム開始");

  // INA238を開始し、通信ができるか確認
  if (!powerMonitor.begin()) {
    Serial.println("INA238が見つかりません。配線を確認してください。");
    while (1); // エラーが発生した場合は無限ループ
  }

  // INA238を設定
  if (!powerMonitor.configure()) {
    Serial.println("INA238の設定に失敗しました。");
  }

  // ADCを設定
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

  // ロードセルADC用のSPIピン設定
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  // RPM計測用のピン設定
  pinMode(PULSE_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PULSE_INPUT), pulseDetected, FALLING);

  // ESCの設定
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000); // ESCをアーム
  delay(5000); // スタートアップのための待ち時間
}

void loop() {
  // INA238からデータを読み取る
  float current = powerMonitor.readCurrent(); // 電流を読み取る
  float shunt = powerMonitor.readShuntVoltage(); // シャント電圧を読み取る
  float voltage = powerMonitor.readVoltage(); // 電圧を読み取る
  float power = powerMonitor.readPower(); // 電力を読み取る
  float temperature = powerMonitor.readTemperature(); // 温度を読み取る

  // 読み取ったデータをシリアルモニタに出力
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" mA");

  Serial.print("Shunt Voltage: ");
  Serial.print(shunt);
  Serial.println(" mV");

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("Power: ");
  Serial.print(power);
  Serial.println(" W");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  // ADC値の読み取り
  readADC();

  // RPMの計算とPWM値の出力
  manageRPMandPWM();
  
  Serial.println("-----------------------");

  delay(1000); // 1秒間隔でデータを読み取る
}

void manageRPMandPWM() {
  // PWM値の計算
  int sensorValue = analogRead(ADC_INPUT);
  int pwmValue = map(sensorValue, 0, 1023, 1000, 2000);
  esc.writeMicroseconds(pwmValue);

  // RPMの計算
  float rpm = 0;
  if (newPulse) {
    unsigned long timeDifference = lastPulseTime - prevPulseTime;
    if (timeDifference > 0) {
      rpm = (60000000.0 / timeDifference) / pulsesPerRevolution;
    }
    newPulse = false;
  }

  // PWM値とRPMの出力
  Serial.print("PWM: ");
  Serial.print(pwmValue);
  Serial.println(" us");
  
  Serial.print("RPM: ");
  Serial.println(rpm);
}

void readADC() {
  uint32_t t1 = micros();
  uint16_t raw = adc.read(MCP3204::Channel::SINGLE_0);
  uint32_t t2 = micros();
  uint16_t val = adc.toAnalog(raw);

  // ADC値の出力
  Serial.print("ADC Value: ");
  Serial.print(raw);
  Serial.print(" (");
  Serial.print(val);
  Serial.println(" mV)");

  // サンプリング時間の出力
  Serial.print("Sampling time: ");
  Serial.print(static_cast<double>(t2 - t1) / 1000, 4);
  Serial.println(" ms");
}

void pulseDetected() {
  prevPulseTime = lastPulseTime;
  lastPulseTime = micros();
  newPulse = true;
}
