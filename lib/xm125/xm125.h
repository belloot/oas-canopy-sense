
// The "Open Air" Rule at Startup

// When you power on the device or call XM125_setup(), the radar immediately fires a series of bursts to see what reflections are caused strictly by its own casing or housing.
// The Problem: If your hand, a table, or your target object is sitting within 4–5 inches of the sensor while it is booting up, the radar will mistake that object for its own enclosure. It will record it, mask it out, and then become completely blind to anything at that specific distance for the rest of the runtime.
// The Fix: Make sure that when you power on the project or reset it, the sensor is pointed out into an empty room or up at a ceiling with nothing blocking it for the first 3 seconds.

// I2C_ADDRESS_FLOATING (0x52)
// I2C_ADDRESS_LOW      (0x51)
// I2C_ADDRESS_HIGH     (0x53)

////////////////////////////////////////TO USE MULTIPLE ADDRESSES DO THIS ///////////////////////////////////////////////////
// inside @file sfDevXM125Core.cpp change the line with theBus to the below to work with 0x51 0x52 and 0x53

// change these 2 lines
// if (theBus->address() != SFE_XM125_I2C_ADDRESS)
//     return ksfTkErrFail;

// into thes lines
// if (theBus->address() != SFE_XM125_I2C_ADDRESS && theBus->address() != 0x51 && theBus->address() != 0x53){
//     return ksfTkErrFail;
// }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef XM125_RADAR_H
#define XM125_RADAR_H

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_XM125_Arduino_Library.h"

class XM125Radar {
public:
    XM125Radar(
        uint8_t i2cAddress,
        TwoWire& wireBus
    );

    struct RadarMeasurement {
        uint8_t i2cAddress;
        uint32_t frame_id;
        uint32_t loop_start_ms;
        uint32_t retCode;
        uint32_t distances;
        int32_t p0_mm;
        int32_t p1_mm;
        int32_t p2_mm;
        int32_t p0_strength;
        int32_t p1_strength;
        int32_t p2_strength;
        uint32_t t_setup;
        uint32_t t_num;
        uint32_t t_dist;
        uint32_t t_str;
        uint32_t total_ms;
    };

    bool begin();
    RadarMeasurement measure();
    int32_t detectorReadingSetupFast();

private:
    SparkFunXM125Distance sensor;
    uint8_t address;
    TwoWire* wire;

    uint32_t frame_id = 0;

    static constexpr uint32_t RANGE_START_MM = 200;
    static constexpr uint32_t RANGE_END_MM = 1000;
    static constexpr uint32_t MAX_STEP_LENGTH = 4;
    static constexpr uint32_t MAX_PROFILE = 5;
    static constexpr uint32_t PEAK_SORTING = 1;
    static constexpr uint32_t REFLECTOR_SHAPE = 1;
    static constexpr uint32_t THRESHOLD_METHOD = 3;
    static constexpr uint32_t THRESHOLD_SENSITIVITY = 600;
    static constexpr uint32_t SIGNAL_QUALITY = 25000;

    static constexpr uint16_t SETUPDELAY = 10;
    static constexpr uint8_t MAX_ERROR_COUNT = 10;
    uint8_t error_count = 0;
    
};

#endif