#include "MS5837.h"

const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

#ifdef ENABLE_TEST_STUBS
	#define MS5837_CONVERSION_PERIOD  200		// delay needed for conversion according to data sheet
#else
	#define MS5837_CONVERSION_PERIOD  20		// delay needed for conversion according to data sheet
#endif

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;
const uint8_t MS5837_02BA01 = 0x00; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_02BA21 = 0x15; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_30BA26 = 0x1A; // Sensor version: From MS5837_30BA datasheet Version PROM Word 0


MS5837::MS5837() {
	setFluidDensityFreshWater();
	D2_temp = 0;
	D1_pres = 0;
	P = 0;
	TEMP = 0;
	
	_model = MS5837_30BA;

	next_state_event_time = 0;
	read_sensor_state = READ_INIT;
}


#ifdef ENABLE_TEST_STUBS
bool MS5837::begin(TwoWireX &wirePort) {
	return (init(wirePort));
}
#else

bool MS5837::begin(TwoWire &wirePort) {
	return (init(wirePort));
}
#endif

#ifdef ENABLE_TEST_STUBS
bool MS5837::init(TwoWireX &wirePort)
#else
bool MS5837::init(TwoWire &wirePort)
#endif
{
	_i2cPort = &wirePort; //Grab which port the user wants us to use
	// Reset the MS5837, per datasheet
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_RESET);
	_i2cPort->endTransmission();

	// Wait for reset to complete
	delay(10);

	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_PROM_READ+i*2);
		_i2cPort->endTransmission();
		_i2cPort->requestFrom(MS5837_ADDR,(uint8_t)2);
		C[i] = (_i2cPort->read() << 8) | _i2cPort->read();
	}

	// Verify that data is correct with CRC
	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated != crcRead ) {
		#ifndef ENABLE_TEST_STUBS
			return false; // CRC fail
		#endif
	}
	uint8_t version = (C[0] >> 5) & 0x7F; // Extract the sensor version from PROM Word 0
	/*
	// This is buggy for 300 bar sensor
	// Set _model according to the sensor version
	if (version == MS5837_02BA01)
	{
		_model = MS5837_02BA;
	}
	else if (version == MS5837_02BA21)
	{
		_model = MS5837_02BA;
	}
	else if (version == MS5837_30BA26)
	{
		_model = MS5837_30BA;
	}
	else
	{
		_model = MS5837_UNRECOGNISED;
	}
	*/
	// The sensor has passed the CRC check, so we should return true even if
	// the sensor version is unrecognised.
	// (The MS5637 has the same address as the MS5837 and will also pass the CRC check)
	// (but will hopefully be unrecognised.)

	return true;
}

void MS5837::setModel(uint8_t model) {
	_model = model;
}

uint8_t MS5837::getModel() {
	return (_model);
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}

void MS5837::setFluidDensityFreshWater()
{
	fluidDensity = 997;
}

void MS5837::setFluidDensitySaltWater()
{
	fluidDensity = 1029;
}

MS5837::read_state MS5837::readAsync()
{
	Serial.printf("%lu   %s\n",millis(),read_state_labels[read_sensor_state]);
	switch(read_sensor_state)
	{
		case READ_INIT:
		case READ_COMPLETE:
		{
			requestD1Conversion();
			break;
		}
		case PENDING_D1_CONVERSION:
		{
			retrieveD1ConversionAndRequestD2Conversion();
			break;
		}
		case PENDING_D2_CONVERSION:
		{
			retrieveD2ConversionAndCalculate();
			break;
		}
		default:
		{
			break;
		}
	}

	return read_sensor_state;
}

bool MS5837::read_original() {
	//Check that _i2cPort is not NULL (i.e. has the user forgoten to call .init or .begin?)
	if (_i2cPort == NULL)
		return false;

	// Request D1 conversion
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_CONVERT_D1_8192);
	_i2cPort->endTransmission();

	delay(20); // Max conversion time per datasheet

	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_ADC_READ);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(MS5837_ADDR,(uint8_t)3);
	D1_pres = 0;
	D1_pres = _i2cPort->read();
	D1_pres = (D1_pres << 8) | _i2cPort->read();
	D1_pres = (D1_pres << 8) | _i2cPort->read();

	// Request D2 conversion
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_CONVERT_D2_8192);
	_i2cPort->endTransmission();

	delay(20); // Max conversion time per datasheet

	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_ADC_READ);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(MS5837_ADDR,(uint8_t)3);
	D2_temp = 0;
	D2_temp = _i2cPort->read();
	D2_temp = (D2_temp << 8) | _i2cPort->read();
	D2_temp = (D2_temp << 8) | _i2cPort->read();

	calculate();

	return true;
}

bool MS5837::read()
{
	//Check that _i2cPort is not NULL (i.e. has the user forgoten to call .init or .begin?)
	if (_i2cPort == NULL)
		return false;

	requestD1Conversion();
	delay(MS5837_CONVERSION_PERIOD+2);
	retrieveD1ConversionAndRequestD2Conversion();
	delay(MS5837_CONVERSION_PERIOD+2);
	retrieveD2ConversionAndCalculate();

	return true;
}

void MS5837::requestD1Conversion()
{
	if ((read_sensor_state == READ_COMPLETE || read_sensor_state == READ_INIT) && millis() >= next_state_event_time)
	{
		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_CONVERT_D1_8192);
		_i2cPort->endTransmission();
		read_sensor_state = PENDING_D1_CONVERSION;
		next_state_event_time = millis() + MS5837_CONVERSION_PERIOD;
	}
}

void MS5837::retrieveD1ConversionAndRequestD2Conversion()
{
	if (read_sensor_state == PENDING_D1_CONVERSION && millis() >= next_state_event_time)
	{
		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_ADC_READ);
		_i2cPort->endTransmission();

		_i2cPort->requestFrom(MS5837_ADDR,(uint8_t)3);
		D1_pres = 0;
		D1_pres = _i2cPort->read();
		D1_pres = (D1_pres << 8) | _i2cPort->read();
		D1_pres = (D1_pres << 8) | _i2cPort->read();

		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_CONVERT_D2_8192);
		_i2cPort->endTransmission();

		read_sensor_state = PENDING_D2_CONVERSION;
		next_state_event_time = millis() + MS5837_CONVERSION_PERIOD;
	}
}

void MS5837::retrieveD2ConversionAndCalculate()
{
	if (read_sensor_state == PENDING_D2_CONVERSION && millis() >= next_state_event_time)
	{
		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_ADC_READ);
		_i2cPort->endTransmission();

		_i2cPort->requestFrom(MS5837_ADDR,(uint8_t)3);
		D2_temp = 0;
		D2_temp = _i2cPort->read();
		D2_temp = (D2_temp << 8) | _i2cPort->read();
		D2_temp = (D2_temp << 8) | _i2cPort->read();

		calculate();

		read_sensor_state = READ_COMPLETE;
		next_state_event_time = millis();
	}
}


void MS5837::calculate() {
	// Given C1-C6 and D1_pres, D2_temp, calculated TEMP and P
	// Do conversion first and then second order temp compensation

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Terms called
	dT = D2_temp-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1_pres*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1_pres*SENS/(2097152l)-OFF)/(8192l);
	}

	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}

	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);

	if ( _model == MS5837_02BA ) {
		P = (((D1_pres*SENS2)/2097152l-OFF2)/32768l);
	} else {
		P = (((D1_pres*SENS2)/2097152l-OFF2)/8192l);

//    P = (P/10.04 + float(millis() % 2581))*10.04;
	}
}

float MS5837::pressure(float conversion) {
    if ( _model == MS5837_02BA ) {
        return P*conversion/100.0f;
    }
    else {
        return P*conversion/10.0f;
    }
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

float MS5837::depth() {
	return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
