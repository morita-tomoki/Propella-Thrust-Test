// INA238.cpp

#include "INA238.h"

INA238::INA238(uint8_t address) : _address(address) {}

bool INA238::begin(TwoWire &wirePort) {
  _wire = &wirePort;
  _wire->begin();
  
  // デバイスが応答するかチェックする処理を追加する
  uint16_t data;
  if (!readRegister16(INA238_REG_CONFIG, data)) { // 例としてCONFIGレジスタを読み込む
    return false;
  }
  return true;
}

bool INA238::configure(ina238_rst_t rst = INA238_RST_NOMINAL, ina238_convDelay_t convDelay = INA238_CONVDLY_0MS, ina238_adcRange_t adcRange = INA238_ADCRANGE_163MV)
{
  uint16_t config = 0;
  config |= (rst << 15) | (convDelay << 6) | (adcRange << 4);
  return writeRegister(INA238_REG_CONFIG, config);
}

bool INA238::adcConfigure(ina238_averages_t avg, ina238_busConvTime_t busConvTime, ina238_shuntConvTime_t shuntConvTime, ina238_thermoConvTime_t thermoConvTime, ina238_mode_t mode)
{
  uint16_t config = 0;
  config |= (mode << 12) | (busConvTime << 9) | (shuntConvTime << 6) | (thermoConvTime << 3) | avg;
  return writeRegister(INA238_REG_ADC_CONFIG, config);
}

bool INA238::readShuntCal(uint16_t &data) {
  if (readRegister16(INA238_REG_SHUNT_CAL, data)) {
    return true; //正常に読み取り
  }
  return false; // エラーを示す
}

float INA238::readCurrent() {
  int16_t data;
  if (readRegister16_2(INA238_REG_CURRENT, data)) {
    return data * 2.5; // データを電流(mV)に変換する(ADCRANGE = 0, R = 2 mOhm, Imax = 81.92 Aの場合)
  }
  return -1; // エラーを示す
}

float INA238::readVoltage() {
  int16_t data;
  if (readRegister16_2(INA238_REG_VBUS, data)) {
    return data * 3.125 / 1000.0; // データを電圧(V)に変換する
  }
  return -1; // エラーを示す
}

float INA238::readShuntVoltage() {
  int16_t data;
  if (readRegister16_2(INA238_REG_VSHUNT, data)) {
    return data * 5.0 / 1000.0; // データを電圧(mV)に変換する(ADCRANGE = 0の場合)
  }
  return -1; // エラーを示す
}

float INA238::readPower() {
  uint32_t data;
  if (readRegister24(INA238_REG_POWER, data)) {
    return data * 0.2 * 2.5 / 1000.0; // データを電力(W)に変換する
  }
  return -1; // エラーを示す
}

float INA238::readTemperature() {
  uint16_t temp;
  if (readRegister16(INA238_REG_DIETEMP, temp)) {
    // 上位12ビットを取得するために右に4ビットシフト
    temp = temp >> 4;

    // 2の補数形式で負の値を処理
    int16_t data;
    if (temp & 0x0800) { // 12ビット目がセットされているかチェック
      data = -1 * ((~(temp - 1)) & 0x0FFF); // 上位4ビットを無視するためにマスク
    } else {
      data = temp;
    }

    return data * 125.0 / 1000.0; // データを温度(°C)に変換
  }
  return -1; // エラーを示す
}


bool INA238::readRegister16(uint8_t reg, uint16_t &data) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) {
    return false;
  }
  _wire->requestFrom(_address, (uint8_t)2);
  if (_wire->available() == 2) {
    data = _wire->read() << 8;
    data |= _wire->read();
    return true;
  }
  return false;
}

bool INA238::readRegister16_2(uint8_t reg, int16_t &data) {
  uint16_t temp_data;
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) {
    return false;
  }
  _wire->requestFrom(_address, (uint8_t)2);
  if (_wire->available() == 2) {
    temp_data = _wire->read() << 8;
    temp_data |= _wire->read();

    // 2の補数形式で負の値を処理
    if (temp_data & 0x8000) {
      data = -1 * ((~temp_data + 1) & 0xFFFF); // 16ビットでマスク
    } else {
      data = temp_data;
    }
    return true;
  }
  return false;
}

bool INA238::readRegister24(uint8_t reg, uint32_t &data) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) {
    return false;
  }
  _wire->requestFrom(_address, (uint8_t)3);
  if (_wire->available() == 3) {
    data = (uint32_t)_wire->read() << 16;  // 最初の8ビットを左に16ビットシフト
    data |= (uint32_t)_wire->read() << 8;   // 次の8ビットを左に8ビットシフト
    data |= _wire->read();                  // 最後の8ビットをそのまま追加
    return true;
  }
  return false;
}


bool INA238::writeRegister(uint8_t reg, uint16_t data) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write((uint8_t)(data >> 8));
  _wire->write((uint8_t)data);
  return _wire->endTransmission() == 0;
}
