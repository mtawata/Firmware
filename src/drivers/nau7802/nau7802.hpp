/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file nau7802.hpp
 *
 * @author Michael Tawata
 *
 * Driver for the Qwiic Scale NAU7802 load cell ADC module.
 * Default I2C address 0x2A is used.
 */

#ifndef	DRIVERS_NAU7802_HPP_
#define	DRIVERS_NAU7802_HPP_

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <lib/parameters/param.h>
#include <perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/nau7802_measurement.h>

#include <drivers/drv_device.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>


#define NAU7802_BASE_ADDRESS 	0x2A
#define NAU7802_BUS_DEFAULT 	PX4_I2C_BUS_EXPANSION

#define NAU7802_MEAS_RATE 320 //hz
#define MEAS_INTERVAL (1e6/NAU7802_MEAS_RATE) //us

#define NAU7802_PATH "/dev/nau7802"

#define NAU7802_PU_CTRL 0x00
#define NAU7802_CTRL1 0x01
#define NAU7802_CTRL2 0x02
#define NAU7802_OCAL1_B2 0x03
#define NAU7802_OCAL1_B1 0x04
#define NAU7802_OCAL1_B0 0x05
#define NAU7802_GCAL1_B3 0x06
#define NAU7802_GCAL1_B2 0x07
#define NAU7802_GCAL1_B1 0x08
#define NAU7802_GCAL1_B0 0x09
#define NAU7802_OCAL2_B2 0x0A
#define NAU7802_OCAL2_B1 0x0B
#define NAU7802_OCAL2_B0 0x0C
#define NAU7802_GCAL2_B3 0x0D
#define NAU7802_GCAL2_B2 0x0E
#define NAU7802_GCAL2_B1 0x0F
#define NAU7802_GCAL2_B0 0x10
#define NAU7802_I2C_CONTROL 0x11
#define NAU7802_ADCO_B2 0x12
#define NAU7802_ADCO_B1 0x13
#define NAU7802_ADCO_B0 0x14
#define NAU7802_ADC 0x15 	//Shared ADC and OTP 32:24
#define NAU7802_OTP_B1 0x16     //OTP 23:16 or 7:0?
#define NAU7802_OTP_B0 0x17     //OTP 15:8
#define NAU7802_PGA 0x1B
#define NAU7802_PGA_PWR 0x1C
#define NAU7802_DEVICE_REV 0x1F

#define NAU7802_PU_CTRL_RR 0
#define NAU7802_PU_CTRL_PUD 1
#define NAU7802_PU_CTRL_PUA 2
#define NAU7802_PU_CTRL_PUR 3
#define NAU7802_PU_CTRL_CS 4
#define NAU7802_PU_CTRL_CR 5
#define NAU7802_PU_CTRL_OSCS 6
#define NAU7802_PU_CTRL_AVDDS 7

#define NAU7802_CTRL1_GAIN 2
#define NAU7802_CTRL1_VLDO 5
#define NAU7802_CTRL1_DRDY_SEL 6
#define NAU7802_CTRL1_CRP 7

#define NAU7802_CTRL2_CALMOD 0
#define NAU7802_CTRL2_CALS 2
#define NAU7802_CTRL2_CAL_ERROR 3
#define NAU7802_CTRL2_CRS 4
#define NAU7802_CTRL2_CHS 7

#define NAU7802_PGA_CHP_DIS 0
#define NAU7802_PGA_INV 1
#define NAU7802_PGA_BYPASS_EN 2
#define NAU7802_PGA_OUT_EN 3
#define NAU7802_PGA_LDOMODE 4
#define NAU7802_PGA_RD_OTP_SEL 5

#define NAU7802_PGA_PWR_PGA_CURR 0
#define NAU7802_PGA_PWR_ADC_CURR 2
#define NAU7802_PGA_PWR_MSTR_BIAS_CURR 4
#define NAU7802_PGA_PWR_PGA_CAP_EN 7

#define NAU7802_LDO_2V4 0b111
#define NAU7802_LDO_2V7 0b110
#define NAU7802_LDO_3V0 0b101
#define NAU7802_LDO_3V3 0b100
#define NAU7802_LDO_3V6 0b011
#define NAU7802_LDO_3V9 0b010
#define NAU7802_LDO_4V2 0b001
#define NAU7802_LDO_4V5 0b000

#define NAU7802_GAIN_128 0b111
#define NAU7802_GAIN_64 0b110
#define NAU7802_GAIN_32 0b101
#define NAU7802_GAIN_16 0b100
#define NAU7802_GAIN_8 0b011
#define NAU7802_GAIN_4 0b010
#define NAU7802_GAIN_2 0b001
#define NAU7802_GAIN_1 0b000

#define NAU7802_SPS_320 0b111
#define NAU7802_SPS_80 0b011
#define NAU7802_SPS_40 0b010
#define NAU7802_SPS_20 0b001
#define NAU7802_SPS_10 0b000

#define NAU7802_CAL_SUCCESS 0
#define NAU7802_CAL_IN_PROGRESS 1
#define NAU7802_CAL_FAILURE 2

static constexpr uint8_t PX4_I2C_BUS_DEFAULT = PX4_I2C_BUS_EXPANSION;


class NAU7802 : public device::I2C, public px4::ScheduledWorkItem
{
public:
	NAU7802(int bus = NAU7802_BUS_DEFAULT, int address = NAU7802_BASE_ADDRESS, const char *path = NAU7802_PATH);

	~NAU7802();

	int init() override;

	int ioctl(device::file_t *filp, int cmd, unsigned long arg) override;
	ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	int measure();
	int collect();

	void start();
	void stop();

	void Run();
	void print_info();

private:
	bool _isConnected();
	bool _available();
	int32_t _getReading();
	// int32_t _getAverage(uint8_t samplesToTake);

	// void _calculateZeroOffset(uint8_t averageAmount = 8);
	// void _setZeroOffset(int32_t newZeroOffset);
	// int32_t _getZeroOffset(); 
	// void _calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount = 8);
	// void _setCalibrationFactor(float calFactor); 
	// float _getCalibrationFactor();
	// float _getWeight(bool allowNegativeWeights = false, uint8_t samplesToTake = 8);

	bool _setGain(uint8_t gainValue);        //x1, 2, 4, 8, 16, 32, 64, 128 are available
	bool _setLDO(uint8_t ldoValue);          //Set the onboard Low-Drop-Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
	bool _setSampleRate(uint8_t rate);       //Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available
	bool _setChannel(uint8_t channelNumber); //Select between 1 and 2

	bool _calibrateAFE();			//Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
	void _beginCalibrateAFE();               //Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
	bool _waitForCalibrateAFE(uint32_t timeout_ms = 0); //Wait for asynchronous AFE calibration to complete with optional timeout.
	uint8_t _calAFEStatus();                 //Check calibration status.

	bool _reset();

	bool _powerUp();   //Power up digital and analog sections of scale, ~2mA
	// bool _powerDown(); //Puts scale into low-power 200nA mode

	// bool _setIntPolarityHigh(); //Set Int pin to be high when data is ready (default)
	// bool _setIntPolarityLow();  //Set Int pin to be low when data is ready

	// uint8_t _getRevisionCode(); //Get the revision code of this IC. Always 0x0F.

	bool _setBit(uint8_t bitNumber, uint8_t registerAddress);   //Mask & set a given bit within a register
	bool _clearBit(uint8_t bitNumber, uint8_t registerAddress); //Mask & clear a given bit within a register
	bool _getBit(uint8_t bitNumber, uint8_t registerAddress);   //Return a given bit within a register

	uint8_t _getRegister(uint8_t registerAddress);             //Get contents of a register
	bool _setRegister(uint8_t registerAddress, uint8_t value); //Send a given value to be written to given address. Return true if successful

	// uORB::PublicationMulti<nau7802_measurement_s>	_pub{ORB_ID(nau7802_measurement)};
	orb_advert_t _nau7802_measurement_topic {nullptr};

	ringbuffer::RingBuffer  *_reports{nullptr};

	//counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "nau7802_read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "nau7802_com_err")};

	bool _sensor_ok;
	int _class_instance;
	int _measure_interval;
	int _orb_class_instance;

	int32_t _zero_offset;
	float _cal_factor;
};



#endif // DRIVERS_NAU7802_HPP_
