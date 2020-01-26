#include "nau7802.hpp"


// Driver 'main' command.
extern "C" __EXPORT int nau7802_main(int argc, char *argv[]);

namespace nau7802
{
NAU7802 *g_dev = nullptr;

int start();
int start_bus(uint8_t i2c_bus);
int stop();
int reset();
int info();
int test();

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	PX4_INFO("NAU7802 failed to start");

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function call only returns once the driver is up and running
 * or failed to detect the sensor.
 */
int
start_bus(uint8_t i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	g_dev = new NAU7802(i2c_bus, NAU7802_BASE_ADDRESS, NAU7802_PATH);

	/* check if the NAU7802 was instantiated */
	if (g_dev == nullptr) {
		goto fail;
	}

	if (PX4_OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(NAU7802_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	PX4_INFO("NAU7802 started successfully");

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

// stop the driver
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

// reset the driver
int reset()
{
	int fd = px4_open(NAU7802_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed ");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int 
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	PX4_INFO("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct nau7802_measurement_s report;
	ssize_t sz;
	int ret;

	int fd = px4_open(NAU7802_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'nau7802 start' if the driver is not running)", NAU7802_PATH);
		return PX4_ERROR;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return PX4_ERROR;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	px4_close(fd);

	PX4_INFO("PASS");
	return PX4_OK;
}

} //namespace nau7802


static void
nau7802_usage()
{
	PX4_INFO("usage: nau7802 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", PX4_I2C_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|reset");
}

int
nau7802_main(int argc, char *argv[])
{
	uint8_t i2c_bus = PX4_I2C_BUS_DEFAULT;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			nau7802_usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		nau7802_usage();
		return PX4_ERROR;
	}


	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return nau7802::start();

		} else {
			return nau7802::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return nau7802::stop();
	}

	if (!strcmp(argv[myoptind], "test")) {
		return nau7802::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return nau7802::reset();
	}

	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return nau7802::info();
	}

	nau7802_usage();
	return 0;
}
