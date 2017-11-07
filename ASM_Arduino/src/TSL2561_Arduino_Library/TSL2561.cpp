/**************************************************************************/
/*! 
    @file     tsl2561.c
    @author   K. Townsend (microBuilder.eu / adafruit.com)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL, Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

/*#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "TSL2561.h"*/

#if defined (ESP32)
  #include <pgmspace.h>
#else
  #include <avr/pgmspace.h>
  #include <util/delay.h>
#endif

#include <stdlib.h>
#include <Wire.h>		// added
#include "TSL2561.h"

TSL2561::TSL2561(uint8_t addr) 
{
  _addr = addr;
  _initialized = false;
  _integration = TSL2561_INTEGRATIONTIME_13MS;
  _gain = TSL2561_GAIN_16X;
}

bool TSL2561::begin(void) 
{
  uint8_t data=0;
  uint8_t result;
  _initialized = false;
  
  //Wire.begin();
  read8(TSL2561_REGISTER_ID, &data);
  //Serial.print("\r\n ID 0x"); Serial.print(data, HEX);
  if ( data & 0x0A ) {
	_initialized = true;
  } 
  else {
    return false;
  }
 
  // Set default integration time and gain
  setTiming(_integration);
  setGain(_gain);
  // Note: by default, the device is in power down mode on bootup
  disable();

  return true;
}

bool TSL2561::enable(void)
{
  // Enable the device by setting the control bit to 0x03
  return write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

bool TSL2561::disable(void)
{
  // Disable the device by setting the control bit to 0x03
  return write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}


bool TSL2561::setGain(tsl2561Gain_t gain) 
{
  uint8_t result;
  
  // Restart if Need
  if (!_initialized) {
	result = begin();
	if (result != 0) return result;
	}

  result = enable();
  _gain = gain;
  result = write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, _integration | _gain);  
  result = disable();
  return result;
}

bool TSL2561::setTiming(tsl2561IntegrationTime_t integration)
{
  uint8_t result;
  
  // Restart if Need
  if (!_initialized) {
	result = begin();
	if (result != 0) return result;
	}
	
  result = enable();
  _integration = integration;
  result = write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, _integration | _gain);  
  result = disable();
  return result;
}

uint32_t TSL2561::calculateLux(uint16_t ch0, uint16_t ch1)
{
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;

  switch (_integration)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: // No scaling ... integration time = 402ms
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  // Scale for gain (1x or 16x)
  if (!_gain) chScale = chScale << 4;

  // scale the channel values
  channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

  // find the ratio of the channel values (Channel1/Channel0)
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  // round the ratio value
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  unsigned long temp;
  temp = ((channel0 * b) - (channel1 * m));

  // do not allow negative lux value
  if (temp < 0) temp = 0;

  // round lsb (2^(LUX_SCALE-1))
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  // strip off fractional portion
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  // Signal I2C had no errors
  return lux;
}

uint32_t TSL2561::getFullLuminosity (void)
{
  uint32_t rez=0;
  uint16_t ch1, ch2;

  if (!_initialized) {
	if (begin() == false) return 0;
	}
	
  // Enable the device by setting the control bit to 0x03
  enable();

  // Wait x ms for ADC to complete
  switch (_integration)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      delay(14);
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      delay(102);
      break;
    default:
      delay(403);
      break;
  }
  
  read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW, &ch1);
  read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW, &ch2);
  rez = (ch2 << 16) | ch1;
  disable();

  return rez;
}

uint16_t TSL2561::getLuminosity (uint8_t channel) 
{
  uint8_t result;
  uint32_t x = getFullLuminosity();
  
  if (x == 0) {
	return 0;
  }

  if (channel == 0) {
    // Reads two byte value from channel 0 (visible + infrared)
    return (x & 0xFFFF);
  } else if (channel == 1) {
    // Reads two byte value from channel 1 (infrared)
    return (x >> 16);
  } else if (channel == 2) {
    // Reads all and subtracts out just the visible!
    return ( (x & 0xFFFF) - (x >> 16));
  }
  
  // unknown channel!
  return 0;
}

// Generic I2C read register (single byte)
bool TSL2561::read8(uint8_t reg, uint8_t* data)
{
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0)	{_initialized = false; return false;}
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr,(uint8_t)1);
  *data = Wire.read();
  if (Wire.endTransmission() != 0)	{_initialized = false; return false;}
  return true;
}

// Generic I2C read registers (two bytes, LSB first)
bool TSL2561::read16(uint8_t reg, uint16_t* data)
{
  uint8_t lwByte, upByte;
  
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0)	{_initialized = false; return false;}
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr, 2);
  lwByte = Wire.read();
  upByte = Wire.read();
  *data = (upByte << 8) | lwByte;
  if (Wire.endTransmission() != 0)	{_initialized = false; return false;}
  return true;
}

// Generic I2C write data to register (single byte)
bool TSL2561::write8 (uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(data);
  if (Wire.endTransmission() != 0)	{_initialized = false; return false;}
  return true;
}
