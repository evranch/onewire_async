#include "tempsensor_async.h"

template <class T>
T ema(T new_value, T long_term, int num_samples)
{
    T return_val = (long_term * (num_samples - 1) + new_value) / num_samples;
    return return_val;
}

// OneWire *tempSensor::onewire = NULL;
SemaphoreHandle_t tempSensor::onewire_mutex = xSemaphoreCreateMutex();

tempSensor::tempSensor(OneWire *device)
{
    onewire = device;
}

tempSensor::tempSensor()
{
}

// Check if address is uninitialized or sensor has not successfully been read
// Default failure time 1 minute
bool tempSensor::check(int unread_time)
{
    if (!address[0])
        return false;
    if (millis() - last_good_read > unread_time * 1000UL)
        return false;
    return true;
}

bool tempSensor::available()
{
    if (!address[0])
    {
        Serial.println("Address not initialized");
        return false;
    }
    if (!xSemaphoreTake(onewire_mutex, 1000))
        return false;
    return true;
}

void tempSensor::setDevice(OneWire *device)
{
    onewire = device;
}

bool tempSensor::setAddress(byte addr[8])
{
    switch (addr[0])
    {
    case 0x10:
        // Serial.print("  Chip = DS18S20 "); // or old DS1820
        sensor_type = 1;
        break;
    case 0x28:
        // Serial.print("  Chip = DS18B20 ");
        sensor_type = 0;
        break;
    case 0x22:
        // Serial.print("  Chip = DS1822 ");
        sensor_type = 0;
        break;
    default:
        Serial.print("Device is not a DS18x20 family device. ");
        return false;
    }

    memcpy(address, addr, 8);
    return true;
}

bool tempSensor::search(byte addr[8])
{
    if (!xSemaphoreTake(onewire_mutex, 1000))
        return false;

    bool result = onewire->search(addr);
    xSemaphoreGive(onewire_mutex);
    return result;
}

bool tempSensor::singleAddress()
{
    byte addr[8];
    if (search(addr))
        return setAddress(addr);

    return false;
}

bool tempSensor::convert()
{
    if (!available())
        return false;

    bool result = onewire->reset();
    onewire->select(address);
    onewire->write(0x44, 0);
    xSemaphoreGive(onewire_mutex);

    return result;
}

bool tempSensor::readC_async()
{
    if (converting)
    {
        if (!conversion_done)
        {
            if (millis() - conversion_timer > 1000L)
            {
                conversion_done = true;
                converting = false;
                if (!bad_data)
                {
                    if (readC_only() != SENSOR_FAIL)
                    {
                        addEma();
                        last_good_read = millis();
                        return true;
                    }
                }
                else
                    bad_data = false; // After a bad CRC we have to throw away one bogus data point
            }
        }
    }
    else
    {
        convert();
        conversion_timer = millis();
        converting = true;
        conversion_done = false;
    }
    return false;
}

float tempSensor::readC()
{
    if (!convert())
        return SENSOR_FAIL;
    delay(1000);
    if (readC_only() != SENSOR_FAIL)
    {
        if (!bad_data)
            return tempC;
        else
            bad_data = false;
    }
    else
    {
        return SENSOR_FAIL;
    }
    return SENSOR_FAIL;
}

float tempSensor::readC_only()
{
    byte data[9];

    if (!available())
        return SENSOR_FAIL;

    onewire->reset();
    onewire->select(address);
    onewire->write(0xBE); // Read Scratchpad

    for (byte i = 0; i < 9; i++)
    {
        data[i] = onewire->read();
    }

    if (OneWire::crc8(data, 8) != data[8] || data[4] == 0)
    {
        Serial.println("BAD CRC");
        fail_cycles++;
        bad_data = true;
        xSemaphoreGive(onewire_mutex);

        return SENSOR_FAIL;
    }

    // fail_cycles = 0;
    //  Convert the data to actual temperature
    //  because the result is a 16 bit signed integer, it should
    //  be stored to an "int16_t" type, which is always 16 bits
    //  even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (sensor_type) // Old sensors
    {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10)
        {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    }
    else // New sensors
    {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
                            //// default is 12 bit resolution, 750 ms conversion time
    }
    xSemaphoreGive(onewire_mutex);

    tempC = (float)raw / 16.0;

    return tempC;
}

String tempSensor::tempString()
{
    return String(tempC);
}

String tempSensor::emaString()
{
    return String(emaC);
}

String tempSensor::addressString()
{
    String returnString;
    for (int i; i < 8; i++)
    {
        returnString.concat("0x");
        returnString.concat(String(address[i], 16));
        if (i < 7)
            returnString.concat(",");
    }
    return returnString;
}

bool tempSensor::addEma()
{
    return addEma(tempC);
}

bool tempSensor::addEma(float new_val)
{
    if (new_val != SENSOR_FAIL)
    {
        // Initialize with first value to avoid dip
        if (ema_samples == 0)
            emaC = new_val;
        else
            emaC = ema(new_val, emaC, ema_samples_max);
        if (ema_samples < ema_samples_max)
        {
            ema_samples++;
            return true;
        }
    }
    return false;
}