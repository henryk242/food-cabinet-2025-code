/**************************************************************************/
/*! 
    @file     MCCI_FRAM_I2C.h
    @author   KTOWN (Adafruit Industries)
    @author   Terry Moore (MCCI Corporation)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, Adafruit Industries
    Portions copyright (c) 2017, MCCI Corporation
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

#ifndef _G_MCCI_FRAM_I2C_H_
#define _G_MCCI_FRAM_I2C_H_

#pragma once

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Wire.h>
#include <cstdint>



class G_MCCI_FRAM_I2C {
 public:
  G_MCCI_FRAM_I2C(void);

  // definitions for the MB85RC FRAM chip.
  class MB85RC
    {
  public:
    static constexpr std::uint8_t   kDefaultAddress = 0x50;
    static constexpr std::uint8_t   kSlaveID = 0xF8;
    };

  // create a version number for comparison
  static constexpr std::uint32_t
  makeVersion(
      std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0
      )
      {
      return ((std::uint32_t)major << 24u) | ((std::uint32_t)minor << 16u) | ((std::uint32_t)patch << 8u) | (std::uint32_t)local;
      }

  // version of library, for use by clients in static_asserts -- set version by editing here:
  static constexpr std::uint32_t getVersion() { return makeVersion(2,0,0,0); }

  // extract major number from version
  static constexpr std::uint8_t
  getMajor(std::uint32_t v)
      {
      return std::uint8_t(v >> 24u);
      }

  // extract minor number from version
  static constexpr std::uint8_t
  getMinor(std::uint32_t v)
      {
      return std::uint8_t(v >> 16u);
      }

  // extract patch number from version
  static constexpr std::uint8_t
  getPatch(std::uint32_t v)
      {
      return std::uint8_t(v >> 8u);
      }

  // extract local number from version
  static constexpr std::uint8_t
  getLocal(std::uint32_t v)
      {
      return std::uint8_t(v);
      }

  // set up and probe device  
  boolean  begin(
		uint8_t addr = MB85RC::kDefaultAddress, 
		TwoWire *pWire = &Wire
		);

  // write a single byte
  void     write8 (uint16_t framAddr, uint8_t value);

  // write a buffer
  void     write  (uint16_t framAddr, uint8_t const *pBuffer, size_t nBuffer);

  // read a single byte
  uint8_t  read8  (uint16_t framAddr);

  // read a buffer
  uint8_t   read   (uint16_t framAddr, uint8_t *pBuffer, size_t nBuffer);

  struct DeviceInfo
 	{
	uint16_t uMfg;
	uint16_t uProduct;
	};

  bool getDeviceID(DeviceInfo& Info);

 private:
  uint8_t m_i2c_addr;
  boolean m_framInitialized;
  TwoWire *m_pWire;

  void prepIO(void) const;
  uint8_t getI2cAddr(uint16_t framAddr) const;

};

typedef G_MCCI_FRAM_I2C MCCI_FRAM_I2C;

#endif
