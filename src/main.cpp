#define NO_BUILD_OF_MAIN_TO_PERMIT_LIB_USAGE_IN_MAKO

#ifndef NO_BUILD_OF_MAIN_TO_PERMIT_LIB_USAGE_IN_MAKO
#include <Arduino.h>
#include <M5StickCPlus.h>

#include <MS5837.h>
MS5837 testSensor;


#ifdef ENABLE_TEST_STUBS
    TwoWireX WireX;
#endif

class TwoWireX
{
protected:
    uint8_t num;
    int8_t sda;
    int8_t scl;

    size_t bufferSize;
    uint8_t *rxBuffer;
    size_t rxIndex;
    size_t rxLength;

    uint8_t *txBuffer;
    size_t txLength;
    uint16_t txAddress;

    uint32_t _timeOutMillis;
    bool nonStop;

private:

public:
    TwoWireX(uint8_t bus_num=0)
    { }
//    ~TwoWire();
    
    //call setPins() first, so that begin() can be called without arguments from libraries
    bool setPins(int sda, int scl)
    {return true;}
    
    bool begin(int sda, int scl, uint32_t frequency=0) // returns true, if successful init of i2c bus
    {return true;}

    // Explicit Overload for Arduino MainStream API compatibility
    inline bool begin()
    {
        return begin(-1, -1, static_cast<uint32_t>(0));
    }

    bool end()
    {
        return true;
    }

    size_t setBufferSize(size_t bSize)
    {
        return 100;
    }

    void setTimeOut(uint16_t timeOutMillis) // default timeout of i2c transactions is 50ms
    {    }

    uint16_t getTimeOut()
    { return 100;}

    bool setClock(uint32_t t)
    {return true;}

    uint32_t getClock()
    { return 100;}

    void beginTransmission(uint16_t address) { }
    void beginTransmission(uint8_t address) { }
    void beginTransmission(int address) { }

    uint8_t endTransmission(bool sendStop) { return 0; }
    uint8_t endTransmission(void) { return 0; }

    size_t requestFrom(uint16_t address, size_t size, bool sendStop) {return 1;}
    uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop) {return 1;}
    uint8_t requestFrom(uint16_t address, uint8_t size, uint8_t sendStop) {return 1;}
    size_t requestFrom(uint8_t address, size_t len, bool stopBit) {return 1;}
    uint8_t requestFrom(uint16_t address, uint8_t size) {return 1;}
    uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop) {return 1;}
    uint8_t requestFrom(uint8_t address, uint8_t size) {return 1;}
    uint8_t requestFrom(int address, int size, int sendStop) {return 1;}
    uint8_t requestFrom(int address, int size) {return 1;}

    size_t write(uint8_t) {return 1;}
    size_t write(const uint8_t *, size_t) {return 1;}
    int available(void) {return 1;}
    int read(void) {return 1;}
    int peek(void) {return 1;}
    void flush(void) {}

    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t)n);
    }
};

void setup()
{
    M5.begin();
    uint32_t start = millis();

    testSensor.begin();
    testSensor.read_original();
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.printf("%lu result: %f\n",start, testSensor.depth());
    M5.Lcd.printf("%lu result: %f\n",start, testSensor.depth());
    Serial.println("");
    Serial.println("");
}

int count=0;
void loop()
{
    if (count++ < 500)
    {
        testSensor.readAsync();
        delay (50);
    }
} 

#endif