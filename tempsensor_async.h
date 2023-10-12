#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H

#define SENSOR_FAIL -999

#include <Arduino.h>
#include <OneWire.h>

class tempSensor
{
public:
    OneWire *onewire;
    static SemaphoreHandle_t onewire_mutex;
    byte address[8] = {0};
    byte sensor_type;
    String topic;
    float tempC = 0;
    float emaC = 0;
    int fail_cycles = 0;
    int ema_samples_max = 5;
    int ema_samples = 0;
    bool converting = 0, conversion_done = 0;
    bool bad_data = false;
    unsigned long conversion_timer = 0;
    unsigned long last_good_read = 0;
    tempSensor(OneWire *device);
    tempSensor();
    void setDevice(OneWire *device);
    bool search(byte addr[8]);
    bool setAddress(byte addr[8]);
    bool singleAddress();
    bool addEma(float new_val);
    bool addEma();
    bool convert();
    bool check(int unread_time = 60);
    float readC();
    float readC_only();
    bool readC_async();
    String tempString();
    String emaString();
    String addressString();

private:
    bool available();
};

#endif