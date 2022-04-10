#include "MCP4725.h"
#include "i2c.h"

/* Init function */
MCP4725 MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725Ax_ADDRESS addr, float ref_vol)
{
	MCP4725 _MCP4725;

	_MCP4725._i2cAddress = (uint16_t)(addr<<1);
	_MCP4725.hi2c = hi2c;

	MCP4725_setReferenceVoltage(&_MCP4725, ref_vol);

	return _MCP4725;
}

uint8_t MCP4725_isConnected(MCP4725* _MCP4725)
{
	return HAL_I2C_IsDeviceReady(_MCP4725->hi2c, _MCP4725->_i2cAddress, 2, 100) == HAL_OK;
}

void MCP4725_setReferenceVoltage(MCP4725* _MCP4725, float value)
{
	if (value == 0) {
		_MCP4725->_refVoltage = MCP4725_REFERENCE_VOLTAGE;
	} else {
		_MCP4725->_refVoltage = value;
	}
	_MCP4725->_bitsPerVolt = (float)MCP4725_STEPS / _MCP4725->_refVoltage;
}

float MCP4725_getReferenceVoltage(MCP4725* _MCP4725)
{
  return _MCP4725->_refVoltage;
}

uint8_t MCP4725_setValue(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND mode, MCP4725_PD powerType)
{
  if (value > MCP4725_MAX_VALUE) {
	  value = MCP4725_MAX_VALUE;
  }

  return MCP4725_writeComand(_MCP4725, value, mode, powerType);
}

uint8_t MCP4725_setVoltage(MCP4725* _MCP4725, float voltage, MCP4725_COMMAND mode, MCP4725_PD powerType)
{
  uint16_t value = 0;

  if (voltage >= _MCP4725->_refVoltage) {
	  value = MCP4725_MAX_VALUE;
  }
  else if (voltage <= 0) {
	  value = 0;
  }
  else {
	  value = voltage * _MCP4725->_bitsPerVolt;
  }

  return MCP4725_writeComand(_MCP4725, value, mode, powerType);
}

uint16_t MCP4725_getValue(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_DAC_REG);

  if (value != MCP4725_ERROR) {
	  return value >> 4;
  }
  return value;	// error
}

float MCP4725_getVoltage(MCP4725* _MCP4725)
{
  float value = MCP4725_getValue(_MCP4725);

  if (value != MCP4725_ERROR) {
	  return value / _MCP4725->_bitsPerVolt; // bits to voltage
  }
  return value; // error
}

uint16_t MCP4725_getStoredValue(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_EEPROM);

  if (value != MCP4725_ERROR) {
	  return value & 0x0FFF;
  }
  return value; // error
}

float MCP4725_getStoredVoltage(MCP4725* _MCP4725)
{
  float value = MCP4725_getStoredValue(_MCP4725);

  if (value != MCP4725_ERROR) {
	  return value / _MCP4725->_bitsPerVolt;
  }
  return value; // error
}

uint16_t MCP4725_getPowerType(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_SETTINGS);

  if (value != MCP4725_ERROR)
  {
	  value &= 0x0006;
	  return value >> 1;
  }

  return value; // error
}

/* get power type from eeprom */
uint16_t MCP4725_getStoredPowerType(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_EEPROM);

  if (value != MCP4725_ERROR)
  {
    value = value << 1;
    return  value >> 14;
  }

  return value; // error
}

void MCP4725_reset(MCP4725* _MCP4725)
{
	uint8_t buffer[1] = {MCP4725_GENERAL_CALL_RESET};
	HAL_I2C_Master_Transmit(_MCP4725->hi2c, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 1000);
}

void MCP4725_wakeUP(MCP4725* _MCP4725)
{
	uint8_t buffer[1] = {MCP4725_GENERAL_WAKE_UP};
	HAL_I2C_Master_Transmit(_MCP4725->hi2c, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 1000);
}

uint8_t MCP4725_getEepromBusyFlag(MCP4725* _MCP4725)
{
  uint16_t value = MCP4725_readRegister(_MCP4725, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR) return (value & 0x80)==0x80;		//1 - completed, 0 - incompleted
                              return 0;										//collision on i2c bus
}

uint8_t	MCP4725_writeComand(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND mode, MCP4725_PD powerType)
{
	uint8_t buffer[3];
	HAL_StatusTypeDef I2C_Stat;

	switch (mode)
	{
    	case MCP4725_FAST_MODE:
			buffer[0] = mode | (powerType << 4)  | highByte(value);
			buffer[1] = lowByte(value);

			I2C_Stat = HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, 2, 1000);
		break;

    	case MCP4725_REGISTER_MODE: case MCP4725_EEPROM_MODE:
    		value = value << 4;
			buffer[0] = mode  | (powerType << 1);
			buffer[1] = highByte(value);
			buffer[2] = lowByte(value);

			I2C_Stat = HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, 3, 1000);
		break;
  }

  if (I2C_Stat != HAL_OK) return 0;                   //send data over i2c & check for collision on i2c bus

  if (mode == MCP4725_EEPROM_MODE)
  {
    if (MCP4725_getEepromBusyFlag(_MCP4725) == 1) return 1;
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME);
    if (MCP4725_getEepromBusyFlag(_MCP4725) == 1) return 1;
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME);
  }
  return 1;
}

uint16_t MCP4725_readRegister(MCP4725* _MCP4725, MCP4725_READ dataType)
{
	uint16_t value = dataType;
	uint16_t ret_val = 0 ;
	uint8_t buffer[dataType];
	HAL_StatusTypeDef I2C_Stat;

	I2C_Stat = HAL_I2C_Master_Receive(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, dataType, 1000);

	if (I2C_Stat != HAL_OK) {
		return MCP4725_ERROR;
	}

	switch (dataType)
	{
    	case MCP4725_READ_SETTINGS:
    		ret_val = buffer[0];
    	break;

    	case MCP4725_READ_REG:
    	case MCP4725_READ_EEPROM:
    		ret_val = buffer[value-2];
    		ret_val = (ret_val << 8) | buffer[value-1];
    	break;
  }
  return ret_val;
}
