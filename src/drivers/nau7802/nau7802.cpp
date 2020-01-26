/**
 * @file nau7802.cpp
 *
 * @author Michael Tawata
 *
 * Driver for the Qwiic Scale NAU7802 load cell ADC module.
 */

#include "nau7802.hpp"

#define PX4_DEBUG_ALT PX4_DEBUG

NAU7802::NAU7802(int bus, int address, const char *path) :
	I2C("NAU7802", path, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_sensor_ok(false),
	_class_instance(-1),
	_measure_interval(0)
{
	PX4_INFO("NAU7802 constructed");
}

NAU7802::~NAU7802()
{
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_nau7802_measurement_topic != nullptr) {
		orb_unadvertise(_nau7802_measurement_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(NAU7802_PATH, _class_instance);
	}

	//free counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void
NAU7802::start()
{
	_reports->flush();
	measure();
	ScheduleNow();
}

void
NAU7802::stop()
{
	ScheduleClear();
}


int
NAU7802::init()
{
	PX4_DEBUG_ALT("NAU7802::init() starting...");
	int ret = PX4_ERROR;

	param_get(param_find("NAU7802_ZERO_OFF"), &_zero_offset);
	param_get(param_find("NAU7802_CAL_FCTR"), &_cal_factor);

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}


	//Check if the device ack's over I2C
	if (_isConnected() == false)
	{
		//There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
		if (_isConnected() == false)
			return (false);
	}

	PX4_DEBUG_ALT("device isConnected");

	bool result = true;
	PX4_DEBUG_ALT("reset");
	result &= _reset();

	PX4_DEBUG_ALT("powerUp");
	result &= _powerUp(); //Power on analog and digital sections of the scale
	PX4_DEBUG_ALT("setLDO");
	result &= _setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3V
	PX4_DEBUG_ALT("set gain");
	result &= _setGain(NAU7802_GAIN_128); //Set gain to 128
	PX4_DEBUG_ALT("set SPS to 320");
	result &= _setSampleRate(NAU7802_SPS_320); //Set samples per second to 320
	PX4_DEBUG_ALT("turn off CLK_CHP");
	result &= _setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.
	PX4_DEBUG_ALT("330pF decoupling");
	result &= _setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
	PX4_DEBUG_ALT("calibrateAFE");
	result &= _calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel
	
	PX4_DEBUG_ALT("NAU7802 calibration calls finished");

	if (!result)
		return ret;

	_reports = new ringbuffer::RingBuffer(2, sizeof(nau7802_measurement_s));

	set_device_address(NAU7802_BASE_ADDRESS);
	PX4_DEBUG_ALT("set_device_address");

	if (_reports == nullptr) {
		return ret;
	}
	PX4_DEBUG_ALT("reports init good");

	_class_instance = register_class_devname(NAU7802_PATH);
	PX4_DEBUG_ALT("devname registered");

	/* get a publish handle on the nau7802_measurement topic */
	struct nau7802_measurement_s ds_report = {};

	_nau7802_measurement_topic = orb_advertise(ORB_ID(nau7802_measurement), &ds_report);

	if (_nau7802_measurement_topic == nullptr) {
		PX4_ERR("failed to create nau7802_measurement object");
	}

	int ret2 = measure();

	if (ret2 == 0) {
		ret = PX4_OK;
		_sensor_ok = true;
		PX4_INFO("(%dHz) with address %d found",
			 (int)(1e6f / _measure_interval), NAU7802_BASE_ADDRESS);
	}

	PX4_DEBUG_ALT("NAU7802::init() returning...");
	return ret;
}

int
NAU7802::measure()
{
	int ret;

	uint8_t cmd = NAU7802_ADCO_B2;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (PX4_OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG_ALT("i2c::transfer returned %d", ret);
		return ret;
	}

	return ret;
}

int
NAU7802::collect()
{
	perf_begin(_sample_perf);

	uint8_t buf[3];

	int ret = transfer(nullptr, 0, &buf[0], 3);

	if (ret != PX4_OK) {
		PX4_DEBUG_ALT("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}


	uint32_t valueRaw = buf[0] << 16;
	valueRaw |= buf[1] << 8;
	valueRaw |= buf[2];

	int32_t valueShifted = (int32_t)(valueRaw << 8);
	int32_t value = (valueShifted >> 8);

	PX4_INFO("collected: %d, %x%x%x", value, buf[0], buf[1], buf[2]);

	float processed_value = (value - _zero_offset)/ _cal_factor;

	struct nau7802_measurement_s report;
	report.timestamp = hrt_absolute_time();
	report.raw_value = value;
	report.processed_value = processed_value;

	if (_nau7802_measurement_topic != nullptr) {
		orb_publish(ORB_ID(nau7802_measurement), _nau7802_measurement_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = PX4_OK;

	perf_end(_sample_perf);
	return ret;
}

void
NAU7802::Run()
{
	if (collect() != PX4_OK) {
		DEVICE_DEBUG("collect error");
		start();
		return;
	}

	if (measure() != PX4_OK) {
		DEVICE_DEBUG("measure error");
		start();
		return;
	}

	ScheduleDelayed(_measure_interval);
}

int
NAU7802::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_interval = MEAS_INTERVAL;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned interval = (1000000 / arg);

					/* check against maximum rate */
					if (interval < MEAS_INTERVAL) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}
		break;
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
NAU7802::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct nau7802_measurement_s);
	struct nau7802_measurement_s rbuf;
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(buffer)) {
				ret += sizeof(rbuf);
				buffer += sizeof(&rbuf);
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(MEAS_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(buffer)) {
			ret = sizeof(rbuf);
		}

	} while (0);

	return ret;
}

void
NAU7802::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u\n", _measure_interval);
	_reports->print_info("report queue");
}

//Returns true if device is present
//Tests for device ack to I2C address
bool 
NAU7802::_isConnected()
{
	return true; //MT: TODO
}

bool
NAU7802::_available()
{
	return (_getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
}

int32_t
NAU7802::_getReading()
{
	uint8_t cmd = NAU7802_ADCO_B2;
	uint8_t buf[3];

	if (transfer(&cmd, 1, nullptr, 0) != PX4_OK)
		return (false);
	if (transfer(nullptr, 0, &buf[0], 3) != PX4_OK)
		return (false);

	uint32_t valueRaw = buf[0] << 16;
	valueRaw |= buf[1] << 8;
	valueRaw |= buf[2];

	int32_t valueShifted = (int32_t)(valueRaw << 8);
	int32_t value = (valueShifted >> 8);

	return value;
}

//x1, 2, 4, 8 16, 32, 64, 128 available
bool
NAU7802::_setGain(uint8_t gainValue)
{
	if (gainValue > 0b111)
		gainValue = 0b111;
	uint8_t value = _getRegister(NAU7802_CTRL1);
	value &= 0b11111000;
	value |= gainValue;

	bool ret = _setRegister(NAU7802_CTRL1, value);

	if (ret)
		PX4_DEBUG_ALT("setGain %02x successful", gainValue);
	else
		PX4_DEBUG_ALT("setGain %02x unsuccessful", gainValue);

	return ret;
}

// set LDO voltage regulator
// 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5 available
bool
NAU7802::_setLDO(uint8_t ldoValue)
{
	if (ldoValue > 0b111)
		ldoValue = 0b111;

	uint8_t value = _getRegister(NAU7802_CTRL1);
	value &= 0b11000111;
	value |= ldoValue << 3;
	bool ret = _setRegister(NAU7802_CTRL1, value);
	ret &= _setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);

	if (ret)
		PX4_DEBUG_ALT("setLDO %02x successful", ldoValue);
	else
		PX4_DEBUG_ALT("setLDO %02x unsuccessful", ldoValue);

	return ret;
}

bool
NAU7802::_setSampleRate(uint8_t rate)
{
	if (rate > 0b111)
		rate = 0b111;

	uint8_t value = _getRegister(NAU7802_CTRL2);
	value &= 0b10001111;
	value |= rate << 4;

	return (_setRegister(NAU7802_CTRL2, value));
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool 
NAU7802::_calibrateAFE()
{
	_beginCalibrateAFE();
	return _waitForCalibrateAFE(1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void 
NAU7802::_beginCalibrateAFE()
{
	_setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
bool 
NAU7802::_waitForCalibrateAFE(uint32_t timeout_ms)
{
	hrt_abstime begin = hrt_absolute_time();
	// int cal_ready;

	while (_calAFEStatus() == NAU7802_CAL_IN_PROGRESS)
	{
		if ((timeout_ms > 0) && (hrt_elapsed_time(&begin)/1000 > timeout_ms)) {
			PX4_DEBUG_ALT("waitForCalibrateAFE timed out");
			return false;
		}
		px4_usleep(1000);
	}

	if (_calAFEStatus() == NAU7802_CAL_SUCCESS) {
		PX4_DEBUG_ALT("Calibration successful");
		return true;
	}
	else {
		PX4_DEBUG_ALT("Calibration failed");
		return false;
	}

}

//Check calibration status.
uint8_t 
NAU7802::_calAFEStatus()
{
	if (_getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
	{
		return NAU7802_CAL_IN_PROGRESS;
	}

	if (_getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
	{
		return NAU7802_CAL_FAILURE;
	}

	// Calibration passed
	return NAU7802_CAL_SUCCESS;
}

bool
NAU7802::_powerUp()
{
	_setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	_setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	uint8_t counter = 0;
	while (1)
	{
		if (_getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
			break;
		px4_usleep(1000);
		if (counter++ > 100) {
			PX4_DEBUG_ALT("powerUp failed...");
			return (false);
		}
	}
	PX4_DEBUG_ALT("powerUp successful!");
	return (true);
}

//Resets all registers to Power Off Defaults
bool NAU7802::_reset()
{
	_setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
	px4_usleep(1000);
	bool ret = _clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
	if (!ret)
		PX4_DEBUG_ALT("Reset failed");
	else
		PX4_DEBUG_ALT("Reset successful");
	return (ret); //Clear RR to leave reset state
}

bool
NAU7802::_setBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = _getRegister(registerAddress);
	value |= (1 << bitNumber); //Set this bit
	return (_setRegister(registerAddress, value));
}

//Mask & clear a given bit within a register
bool 
NAU7802::_clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = _getRegister(registerAddress);
	value &= ~(1 << bitNumber); //Set this bit
	return (_setRegister(registerAddress, value));
}

//Return a given bit within a register
bool 
NAU7802::_getBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value = _getRegister(registerAddress);
	value &= (1 << bitNumber); //Clear all but this bit
	return (value);
}

//Get contents of a register
uint8_t 
NAU7802::_getRegister(uint8_t registerAddress)
{
	uint8_t cmd = registerAddress;
	uint8_t buf[1] = {0xff};

	if (transfer(&cmd, 1, &buf[0], 1) != PX4_OK)
		goto fail;
	// if (transfer(nullptr, 0, &buf[0], 1) != PX4_OK)
	// 	goto fail;

	PX4_DEBUG_ALT("register %02x = %02x", registerAddress, buf[0]);

	return buf[0];
fail:
	PX4_DEBUG_ALT("_getRegister %02x failed", registerAddress);
	return -1;
}

//Send a given value to be written to given address
//Return true if successful
bool 
NAU7802::_setRegister(uint8_t registerAddress, uint8_t value)
{
	uint8_t send[2] = {registerAddress, value};
	bool ret = true;

	// if (transfer(&registerAddress, 1, nullptr, 0) != PX4_OK)
	// 	ret = false;
	// if (transfer(&value, 1, nullptr, 0) != PX4_OK)
	// 	ret = false;
	if (transfer(&send[0], 2, nullptr, 0) != PX4_OK)
		ret = false;

	if (ret)
		PX4_DEBUG_ALT("setRegister successful with %02x: %02x", registerAddress, value); 
	else
		PX4_DEBUG_ALT("setRegister failed with %02x: %02x", registerAddress, value);

	return ret;
}
