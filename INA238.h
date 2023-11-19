// INA238.h

#ifndef INA238_H
#define INA238_H

#include <Arduino.h>
#include <Wire.h>

#define INA238_ADDRESS              (0x40)

#define INA238_REG_CONFIG           (0x00)
#define INA238_REG_ADC_CONFIG       (0x01)
#define INA238_REG_SHUNT_CAL        (0x02)
#define INA238_REG_VSHUNT           (0x04)
#define INA238_REG_VBUS             (0x05)
#define INA238_REG_DIETEMP          (0x06)
#define INA238_REG_CURRENT          (0x07)
#define INA238_REG_POWER            (0x08)
#define INA238_REG_DIAG_ALRT        (0x0B)
#define INA238_REG_SOVL             (0x0C)
#define INA238_REG_SUVL             (0x0D)
#define INA238_REG_BOVL             (0x0E)
#define INA238_REG_BUVL             (0x0F)
#define INA238_REG_TEMP_LIMIT       (0x10)
#define INA238_REG_PWR_LIMIT        (0x11)
#define INA238_REG_MANUFACTURER_ID  (0x3E)
#define INA238_REG_DEVICE_ID        (0x3F)

typedef enum
{
    INA238_AVERAGES_1             = 0x0,
    INA238_AVERAGES_4             = 0x1,
    INA238_AVERAGES_16            = 0x2,
    INA238_AVERAGES_64            = 0x3,
    INA238_AVERAGES_128           = 0x4,
    INA238_AVERAGES_256           = 0x5,
    INA238_AVERAGES_512           = 0x6,
    INA238_AVERAGES_1024          = 0x7
} ina238_averages_t;

typedef enum
{
    INA238_BUS_CONV_TIME_50US     = 0x0,
    INA238_BUS_CONV_TIME_84US     = 0x1,
    INA238_BUS_CONV_TIME_150US    = 0x2,
    INA238_BUS_CONV_TIME_280US    = 0x3,
    INA238_BUS_CONV_TIME_540US    = 0x4,
    INA238_BUS_CONV_TIME_1052US   = 0x5,
    INA238_BUS_CONV_TIME_2074US   = 0x6,
    INA238_BUS_CONV_TIME_4120US   = 0x7
} ina238_busConvTime_t;

typedef enum
{
    INA238_SHUNT_CONV_TIME_50US   = 0x0,
    INA238_SHUNT_CONV_TIME_84US   = 0x1,
    INA238_SHUNT_CONV_TIME_150US  = 0x2,
    INA238_SHUNT_CONV_TIME_280US  = 0x3,
    INA238_SHUNT_CONV_TIME_540US  = 0x4,
    INA238_SHUNT_CONV_TIME_1052US = 0x5,
    INA238_SHUNT_CONV_TIME_2074US = 0x6,
    INA238_SHUNT_CONV_TIME_4120US = 0x7
} ina238_shuntConvTime_t;

typedef enum
{
    INA238_THERMO_CONV_TIME_50US   = 0x0,
    INA238_THERMO_CONV_TIME_84US   = 0x1,
    INA238_THERMO_CONV_TIME_150US  = 0x2,
    INA238_THERMO_CONV_TIME_280US  = 0x3,
    INA238_THERMO_CONV_TIME_540US  = 0x4,
    INA238_THERMO_CONV_TIME_1052US = 0x5,
    INA238_THERMO_CONV_TIME_2074US = 0x6,
    INA238_THERMO_CONV_TIME_4120US = 0x7
} ina238_thermoConvTime_t;

typedef enum
{
    INA238_MODE_POWERDOWN           = 0x0,
    INA238_MODE_BUS_TRIG            = 0x1,
    INA238_MODE_BHUNT_TRIG          = 0x2,
    INA238_MODE_BHUNT_BUS_TRIG      = 0x3,
    INA238_MODE_TEMP_TRIG           = 0x4,
    INA238_MODE_BUS_TEMP_TRIG       = 0x5,
    INA238_MODE_SHUNT_TEMP_TRIG     = 0x6,
    INA238_MODE_SHUNT_BUS_TEMP_TRIG = 0x7,
    INA238_MODE_POWERDOWN2          = 0x8,
    INA238_MODE_BUS_CONT            = 0x9,
    INA238_MODE_SHUNT_CONT          = 0xA,
    INA238_MODE_SHUNT_BUS_CONT      = 0xB,
    INA238_MODE_TEMP_CONT           = 0xC,
    INA238_MODE_BUS_TEMP_CONT       = 0xD,
    INA238_MODE_SHUNT_TEMP_CONT     = 0xE,
    INA238_MODE_SHUNT_BUS_TEMP_CONT = 0xF
} ina238_mode_t;

typedef enum
{
    INA238_RST_NOMINAL             = 0x0,
    INA238_RST_SYSTEM_RESET        = 0x1
} ina238_rst_t;

typedef enum
{
    INA238_CONVDLY_0MS             = 0x0,
    INA238_CONVDLY_2MS             = 0x1,
    INA238_CONVDLY_510MS           = 0xFF
} ina238_convDelay_t;

typedef enum
{
    INA238_ADCRANGE_163MV          = 0x0,
    INA238_ADCRANGE_40MV           = 0x1
} ina238_adcRange_t;

class INA238 {
public:
  INA238(uint8_t address);
  bool begin(TwoWire &wirePort = Wire);
  float readCurrent();
  float readVoltage();
  float readPower();
  bool configure(
    ina238_rst_t rst = INA238_RST_NOMINAL,
    ina238_convDelay_t convDelay = INA238_CONVDLY_0MS,
    ina238_adcRange_t adcRange = INA238_ADCRANGE_163MV
    );
  bool adcConfigure(
    ina238_averages_t avg = INA238_AVERAGES_1,
    ina238_busConvTime_t busConvTime = INA238_BUS_CONV_TIME_1052US,
    ina238_shuntConvTime_t shuntConvTime = INA238_SHUNT_CONV_TIME_1052US,
    ina238_thermoConvTime_t thermoConvTime = INA238_THERMO_CONV_TIME_1052US,
    ina238_mode_t mode = INA238_MODE_SHUNT_BUS_TEMP_CONT
    );
  bool readShuntCal(uint16_t &data);
  float readShuntVoltage();
  float readTemperature();
  // その他の必要な関数をここに追加する

private:
  TwoWire *_wire;
  uint8_t _address;
  bool readRegister16(uint8_t reg, uint16_t &data);
  bool readRegister16_2(uint8_t reg, int16_t &data);
  bool readRegister24(uint8_t reg, uint32_t &data);
  bool writeRegister(uint8_t reg, uint16_t data);
};

#endif // INA238_H
