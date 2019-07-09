/*****************************************
  NS_energyShield2.h

  Created by Aaron D. Liebold
  on January 30, 2017

  Distributed under the MIT license
  Copyright 2017 NightShade Electronics
  https://opensource.org/licenses/MIT
  
*****************************************/

#include <Wire.h>
#include "Arduino.h"

#ifndef NS_ENERGYSHIELD2_H
#define NS_ENERGYSHIELD2_H

// Define RTC TWI slave address
#ifndef RTC_SLAVE_ADDR 
#define RTC_SLAVE_ADDR 0x51
#endif

// Define DAC TWI slave address
#ifndef DAC_SLAVE_ADDR 
#define DAC_SLAVE_ADDR 0x60
#endif

// Define Fuel Gauge TWI slave address
#ifndef FG_SLAVE_ADDR 
#define FG_SLAVE_ADDR 0x55
#endif

// Define capacity of battery in mAh
#ifndef BATTERY_CAPACITY 
#define BATTERY_CAPACITY 1800
#endif

// Define termination voltage of battery in mV
#ifndef BATTERY_TERMVOLT_MV 
#define BATTERY_TERMVOLT_MV 3000
#endif

// Define termination current of battery in mV
#ifndef BATTERY_TERMCUR_MA 
#define BATTERY_TERMCUR_MA 65
#endif

// Define alarm state-of-charge in percent (%)
#ifndef ALARM_SOC 
#define ALARM_SOC 10
#endif

//use IIC2

class NS_energyShield2 
{
	public:
				NS_energyShield2();
				NS_energyShield2(uint16_t batteryCapacity_mAh);
		
		// RTC Functions
		void 	setTimeDate(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfMonth, uint8_t dayOfWeek, uint8_t month, uint8_t year);
		void 	readClock();	
		uint8_t second();
		uint8_t minute();	
		uint8_t hour();
		uint8_t dayOfMonth();
		uint8_t dayOfWeek();
		uint8_t month();
		uint8_t year();
		void 	clearAlarms();
		void	writeAlarms(long alarmTimeSeconds);
		void	sleepSeconds(long timeInSeconds);
		
		// Solar Functions
		void 	setVMPP(int MPP_Voltage_mV, bool writeEEPROM);
		int		readVMPP();
		uint16_t inputVoltage();
		uint16_t inputVoltage(uint8_t pin);
				
		// Fuel gauge functions
		uint16_t batteryVoltage();
		int16_t	 batteryCurrent();
		int16_t  temperature();
		uint16_t SOC();
		uint16_t fullChargeCapacity();
		uint16_t remainingCapacity();
		int		 batteryAlert(uint8_t alarmSOC);
		
		// Setup function
		int 	 begin(TwoWire *wire);
		
		// Added for compatibility		
		int		 voltage();
		int		 current();
		int		 percent();
		int		 Vadp(int pin);
		
	
	private:
		uint8_t  _timeDate[7];
		uint16_t _batteryCapacity;
		TwoWire *nsWire;

		// Utility functions
		

		void TWI_writeByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data);
		uint8_t TWI_readByte(uint8_t slaveAddress, uint8_t registerAddress);

		void writeCommand(uint8_t slaveAddress, uint8_t cmdByte1, uint16_t dataWord);
		uint16_t readCommand(uint8_t slaveAddress, uint8_t cmdByte1);

		uint16_t readSubCommand(uint8_t slaveAddress, uint16_t controlData);

		int setupFuelGauge(uint8_t slaveAddress, uint16_t newDesignCapacity_mAh, uint16_t newTerminationVoltage_mV, uint16_t chargeTerminationCurrent_mA, uint8_t alarmSOC);
		int checkIfSealed(uint8_t slaveAddress);

		uint8_t decodeBCD(uint8_t BCD);
		uint8_t encodeBCD(uint8_t value);
};



#endif