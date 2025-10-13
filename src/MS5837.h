/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------

Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS

#include "Arduino.h"

#ifdef ENABLE_TEST_STUBS
	#include "WireTestStub.h"
#else
	#include <Wire.h>
#endif

static const char* read_state_labels[] = {"READ_INIT","PENDING_D1_CONVERSION","PENDING_D2_CONVERSION","READ_COMPLETE"};

class MS5837 {
public:
	enum read_state {READ_INIT, PENDING_D1_CONVERSION, PENDING_D2_CONVERSION, READ_COMPLETE};

	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;
	static const uint8_t MS5837_UNRECOGNISED;

	// OSR (Over-Sampling Ratio) constants
	// Resolution specs are RMS noise from datasheet (depth in saltwater, temp all OSR)
	static const uint8_t OSR_256;   // Depth: ±2.5cm, Temp: ±0.01°C
	static const uint8_t OSR_512;   // Depth: ±1.7cm, Temp: ±0.01°C
	static const uint8_t OSR_1024;  // Depth: ±1.1cm, Temp: ±0.01°C
	static const uint8_t OSR_2048;  // Depth: ±0.7cm, Temp: ±0.01°C
	static const uint8_t OSR_4096;  // Depth: ±0.5cm, Temp: ±0.01°C
	static const uint8_t OSR_8192;  // Depth: ±0.4cm, Temp: ±0.01°C

	MS5837();

#ifdef ENABLE_TEST_STUBS
	bool init(TwoWireX &wirePort = WireX);
	bool begin(TwoWireX &wirePort = WireX); // Calls init()
#else
	bool init(TwoWire &wirePort = Wire);
	bool begin(TwoWire &wirePort = Wire); // Calls init()
#endif

	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
	 * and MS5837::MS5837_02BA.
	 */
	void setModel(uint8_t model);
	uint8_t getModel();

	/** Provide the density of the working fluid in kg/m^3. Default is for
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	void setFluidDensityFreshWater();

	void setFluidDensitySaltWater();

	/** Set Over-Sampling Ratio. Valid options are OSR_256 (fastest, 0.6ms),
	 * OSR_512, OSR_1024, OSR_2048, OSR_4096, OSR_8192 (slowest, most precise, 17.2ms, default).
	 */
	void setOSR(uint8_t osr);

	bool calibrateAtSurfaceForAtmosphericPressure();

	/** The read from I2C takes up to 40 ms, so use sparingly is possible.
	 */
	bool read();
	bool read_original();

	read_state readAsync();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();

	uint32_t getNextStateEventTime() const
	{
		return next_state_event_time;
	}

	read_state getReadSensorState() const
	{
		return read_sensor_state;
	}

	const char* getReadSensorStateLabel() const
	{
		return read_state_labels[read_sensor_state];
	}

private:

	uint32_t next_state_event_time;
	read_state read_sensor_state;

	//This stores the requested i2c port
#ifdef ENABLE_TEST_STUBS
	TwoWireX * _i2cPort;
#else
	TwoWire * _i2cPort;
#endif

	uint16_t C[8]={0,0,0,0,0,0,0,0};
	uint32_t D1_pres, D2_temp;
	int32_t TEMP;
	int32_t P;
	uint8_t _model;
	float _atmosphericPressure;

	float _fluidDensity;

	// OSR configuration
	uint8_t _osr;
	uint8_t _convertD1Cmd;
	uint8_t _convertD2Cmd;
	uint32_t _conversionDelayUs;

	void requestD1Conversion();
	void retrieveD1ConversionAndRequestD2Conversion();
	void retrieveD2ConversionAndCalculate();

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	void calculate();

	uint8_t crc4(uint16_t n_prom[]);
};


#endif
