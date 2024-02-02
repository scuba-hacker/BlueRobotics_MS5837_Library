#define NO_BUILD_OF_MAIN_TO_PERMIT_LIB_USAGE_IN_MAKO

#ifndef NO_BUILD_OF_MAIN_TO_PERMIT_LIB_USAGE_IN_MAKO
#include <Arduino.h>
#include <M5StickCPlus.h>

#include <MS5837.h>
MS5837 testSensor;


#ifdef ENABLE_TEST_STUBS
    TwoWireX WireX;
#endif

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