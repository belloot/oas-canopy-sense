#include "xm125.h"

XM125Radar::XM125Radar(uint8_t i2cAddress,TwoWire& wireBus):address(i2cAddress), wire(&wireBus){}

bool XM125Radar::begin() {

    Serial.printf("XM125 begin addr = 0x%02X\n", address);
    if (sensor.begin(address, *wire) != 1) {
        Serial.println("Radar begin FAILED");
        return false;
    }

    sensor.setCommand(SFE_XM125_DISTANCE_RESET_MODULE);
    sensor.busyWait();
    delay(1000);

    uint32_t errorStatus = 0;
    sensor.getDetectorErrorStatus(errorStatus);

    if (errorStatus != 0) {
        Serial.print("Detector status error: ");
        Serial.println(errorStatus);
    }

    sensor.setStart(RANGE_START_MM);
    sensor.setEnd(RANGE_END_MM);
    sensor.setMaxStepLength(MAX_STEP_LENGTH);
    sensor.setMaxProfile(MAX_PROFILE);
    sensor.setPeakSorting(PEAK_SORTING);
    sensor.setReflectorShape(REFLECTOR_SHAPE);
    sensor.setThresholdMethod(THRESHOLD_METHOD);
    sensor.setThresholdSensitivity(THRESHOLD_SENSITIVITY);
    sensor.setSignalQuality(SIGNAL_QUALITY);
    sensor.setCloseRangeLeakageCancellation(false);

    sensor.setCommand(SFE_XM125_DISTANCE_APPLY_CONFIGURATION);
    sensor.busyWait();

    return true;
}

int32_t XM125Radar::detectorReadingSetupFast() {
    uint32_t errorStatus = 0;
    uint32_t calibrateNeeded = 0;
    uint32_t measDistErr = 0;

    sensor.getDetectorErrorStatus(errorStatus);
    if (errorStatus != 0) return 1;

    if (sensor.setCommand(SFE_XM125_DISTANCE_START_DETECTOR) != 0) return 2;

    sftk_delay_ms(SETUPDELAY);

    if (sensor.busyWait() != 0) return 3;

    sensor.getDetectorErrorStatus(errorStatus);
    if (errorStatus != 0) return 4;

    sftk_delay_ms(SETUPDELAY);
    
    sensor.getMeasureDistanceError(measDistErr);
    if (measDistErr == 1) return 5;

    sftk_delay_ms(SETUPDELAY);

    sensor.getCalibrationNeeded(calibrateNeeded);
    if (calibrateNeeded == 1) {
        sensor.setCommand(SFE_XM125_DISTANCE_RECALIBRATE);
        return 6;
    }

    return 0;
}
XM125Radar::RadarMeasurement XM125Radar::measure() {
    RadarMeasurement m;

    m.i2cAddress=address;
    m.frame_id = frame_id++;
    m.loop_start_ms = millis();

    m.retCode = 0;
    m.distances = 0;
    m.p0_mm = -1;
    m.p0_strength = 0;
    m.t_setup = 0;
    m.t_num = 0;
    m.t_p0dist = 0;
    m.t_p0str = 0;
    m.total_ms = 0;

    uint32_t t0 = millis();
    m.retCode = detectorReadingSetupFast();
    sensor.busyWait();
    m.t_setup = millis() - t0;

    if (m.retCode == 0) {
        error_count = 0;
        t0 = millis();
        sensor.getNumberDistances(m.distances);
        m.t_num = millis() - t0;
    }else {
        error_count++;

        Serial.printf("XM125 setup/read failed, retCode=%d, error_count=%u, address=0x%02X\n",
                  m.retCode, error_count, address);

        if (error_count >= MAX_ERROR_COUNT) {
            Serial.printf("XM125 stuck, resetting module, address=0x%02X\n", address);

            begin();          // re-apply configuration
            error_count = 0;
        }

        m.total_ms = millis() - m.loop_start_ms;
        return m;
    }

    sensor.busyWait();
    if (m.retCode == 0 && m.distances > 0) {
        uint32_t rawDistance = 0;
        int32_t rawStrength = 0;

        t0 = millis();
        if (sensor.getPeak0Distance(rawDistance) == ksfTkErrOk) {
            m.p0_mm = (int32_t)rawDistance;
        }
        m.t_p0dist = millis() - t0;

        t0 = millis();
        if (sensor.getPeak0Strength(rawStrength) == ksfTkErrOk) {
            m.p0_strength = rawStrength / 1000;
        }
        m.t_p0str = millis() - t0;
    }

    m.total_ms = millis() - m.loop_start_ms;

    return m;
}