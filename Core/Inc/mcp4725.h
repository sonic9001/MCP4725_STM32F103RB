#ifndef MCP4725_h
#define MCP4725_h

#include "main.h"

#define lowerByte(x)	((uint8_t)(x%256))	// get lower byte function
#define higherByte(x)	((uint8_t)(x/256))	// get higher byte function

#define MCP4725_GENERAL_CALL_ADDRESS 0x00	// general call address
#define MCP4725_GENERAL_CALL_RESET   0x06	// general call reset
#define MCP4725_GENERAL_WAKE_UP      0x09	// general call wake-up

#define MCP4725_RESOLUTION           12		// dacs resolution
#define MCP4725_STEPS                4096	// dacs avaible steps
#define MCP4725_EEPROM_WRITE_TIME    25		// write time

#define MCP4725_REFERENCE_VOLTAGE    5.0   	// ref voltage set to 5 V
#define MCP4725_MAX_VALUE            4095 	// steps max value, 4096 - 1
#define MCP4725_ERROR                0xFFFF // error define

/* adress enum */
typedef enum
{
  MCP4725A0_A00	= 0x60,
  MCP4725A0_A01	= 0x61,
  MCP4725A1_A00	= 0x62,
  MCP4725A1_A01	= 0x63,
  MCP4725A2_A00	= 0x64,
  MCP4725A2_A01	= 0x65
}
MCP4725Ax_ADDRESS;

/* modes enum */
typedef enum
{
  MCP4725_FAST_MODE		= 0x00,
  MCP4725_REGISTER_MODE	= 0x40,
  MCP4725_EEPROM_MODE	= 0x60
}
MCP4725_COMMAND;

/* read type enum */
typedef enum
{
  MCP4725_READ_SETTINGS	= 1,
  MCP4725_READ_EEPROM	= 3,
  MCP4725_READ_REG		= 5,
}
MCP4725_READ;

/* powerdown enum */
typedef enum
{
  MCP4725_PD_OFF  = 0x00,
  MCP4725_PD_1K   = 0x01,
  MCP4725_PD_100K = 0x02,
  MCP4725_PD_500K = 0x03
}
MCP4725_PD;

typedef struct MCP
{
	I2C_HandleTypeDef* hi2c;
	MCP4725Ax_ADDRESS _i2cAddress;
	float ref_voltage;
	uint16_t bitsPerVolt;
} MCP4725;

MCP4725 MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725Ax_ADDRESS addr, float ref_vol);
uint8_t	MCP4725_isConnected(MCP4725* _MCP4725);
void MCP4725_setReferenceVoltage(MCP4725* _MCP4725, float value);
float MCP4725_getReferenceVoltage(MCP4725* _MCP4725);
uint8_t	MCP4725_setValue(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND mode, MCP4725_PD powerType);
uint8_t	MCP4725_setVoltage(MCP4725* _MCP4725, float voltage, MCP4725_COMMAND mode, MCP4725_PD powerType);
uint16_t MCP4725_getValue(MCP4725* _MCP4725);
float MCP4725_getVoltage(MCP4725* _MCP4725);
uint16_t MCP4725_getStoredValue(MCP4725* _MCP4725);
float MCP4725_getStoredVoltage(MCP4725* _MCP4725);
uint16_t MCP4725_getPowerType(MCP4725* _MCP4725);
uint16_t MCP4725_getStoredPowerType(MCP4725* _MCP4725);
void MCP4725_reset(MCP4725* _MCP4725);
void MCP4725_wakeUP(MCP4725* _MCP4725);
uint8_t	MCP4725_getEepromBusyFlag(MCP4725* _MCP4725);
uint8_t	MCP4725_writeComand(MCP4725* _MCP4725, uint16_t value, MCP4725_COMMAND mode, MCP4725_PD powerType);
uint16_t MCP4725_readRegister(MCP4725* _MCP4725, MCP4725_READ dataType);
#endif
