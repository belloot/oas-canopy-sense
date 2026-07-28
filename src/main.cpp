#include <Arduino.h>
#include <Wire.h>
#include "xm125.h"
#include "OLED.h"
#include "HeightIndicator.h"
#include "driver/gpio.h"
#include "PID.h"

#define OLED_SDA 17
#define OLED_SCL 18

#define RADAR_SDA 40
#define RADAR_SCL 41

volatile float sharedHeightInches = -1.0f;
portMUX_TYPE heightMux = portMUX_INITIALIZER_UNLOCKED;

TwoWire RadarWire = TwoWire(1);
XM125Radar radar1(0x51, RadarWire);
// XM125Radar radar2(SFE_XM125_I2C_ADDRESS,RadarWire);

// Variables for old filtering [NOT USING]
// float filtered_mm = -1.0;
// float filtered_mm2 = -1.0;

// For median filtering [USING]
// ---------------------
const int num_raws = 3;     // should be odd numbers only
float curr_median = -1.0;
float median_ema_mm = -1.0;
float median_ema_inches = -1.0;
float prev_raws[num_raws];
int raw_index = 0;
int sample_count = 0;
// ---------------------

// For height indicator
// ----------------------
int height_level = -1;
// ----------------------

const float alpha = 0.3;
// const float alpha2 = 0.3;
uint32_t tInit = 0;

TaskHandle_t radarTaskHandle = nullptr;

void i2cScan(TwoWire& wireBus) {
    byte error;
    int found = 0;

    Serial.println("Scanning...");

    for (byte address = 1; address < 127; address++) {
        wireBus.beginTransmission(address);
        error = wireBus.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            found++;
        } else if (error == 4) {
            Serial.print("Unknown error at 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (found == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.printf("Done. Found %d device(s).\n", found);
    }

    Serial.println();
}

float getMedian(float raws[num_raws]) {
    // Copy of array to work on
    float temp[num_raws];

    for (int i = 0; i < num_raws; i++) {
        temp[i] = raws[i];
    }

    // Sort in ascending order
    for (int i = 0; i < (num_raws - 1); i++) {
        for (int j = i + 1; j < num_raws; j++) {
            if (temp[j] < temp[i]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }

    return temp[num_raws / 2]; // the median
}

void radarTask(void* parameter) {
    while (true) {
        XM125Radar::RadarMeasurement m = radar1.measure();

        int32_t raw_mm = m.p0_mm;
        char buf[32];

        /*
        One stage EMA filter

        if (raw_mm >= 0) {
            if (filtered_mm < 0) {
                filtered_mm = raw_mm;
            } else {
                filtered_mm = alpha * raw_mm + (1.0 - alpha) * filtered_mm;
            }
            if(filtered_mm>lowbound && filtered_mm<highbound){
                snprintf(buf, sizeof(buf), "Good|%d", (int32_t)(filtered_mm + 0.5));
            }else if(filtered_mm<=lowbound){
                snprintf(buf, sizeof(buf), "Low |%d", (int32_t)(filtered_mm + 0.5));
            }else if(filtered_mm>=highbound){
                snprintf(buf, sizeof(buf), "High|%d", (int32_t)(filtered_mm + 0.5));
            }
        } else {
            snprintf(buf, sizeof(buf), "NOPEAK");
        }
        */

        // (Median of last num_raws) + (EMA of median filter)
        if (raw_mm >= 0 && raw_mm < 1000) {
            // Circular/ring buffer
            prev_raws[raw_index] = raw_mm;
            raw_index = (raw_index + 1) % num_raws;

            if (sample_count < num_raws) {
                median_ema_mm = raw_mm;
                sample_count++;
            } else {
                curr_median = getMedian(prev_raws);
                median_ema_mm = alpha * curr_median + (1 - alpha) * median_ema_mm;
            }

            // Convert to inches
            median_ema_inches = median_ema_mm / 25.4;
            taskENTER_CRITICAL(&heightMux);
            sharedHeightInches = median_ema_inches;
            taskEXIT_CRITICAL(&heightMux);

            snprintf(buf, sizeof(buf), "%.3f", median_ema_inches /*(int32_t)(median_ema_mm + 0.5)*/);
        } else if (raw_mm >= 1000) {
            snprintf(buf, sizeof(buf), ">= 1000!");
        } else {
            snprintf(buf, sizeof(buf), "NO PEAK");
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        // XM125Radar::RadarMeasurement m2 = radar2.measure();
        // int32_t raw_mm2 = m2.p0_mm;
        // char buf2[32];
        // if (raw_mm2 >= 0) {
        //     if (filtered_mm2 < 0) {
        //         filtered_mm2 = raw_mm2;
        //     } else {
        //         filtered_mm2 = alpha2 * raw_mm2 + (1.0 - alpha2) * filtered_mm2;
        //     }
        //     snprintf(buf2, sizeof(buf2), "%d", (int32_t)(filtered_mm2 + 0.5));
        // } else {
        //     snprintf(buf2, sizeof(buf2), "NOPEAK");
        // }

        uint32_t tNow = millis() - tInit;

        char buffer[64];   // expand size for two radars [WAITING FOR ADDITIONAL RADARS]
        snprintf(buffer, sizeof(buffer), "%s", buf/*, buf2*/);
        OLED_writeText(buffer, 4, u8x8_font_chroma48medium8_r);
        height_level = heightIndicatorUpdate(median_ema_inches);

        Serial.printf("%lu,0x%02X,%lu,%lu,%lu,%ld,%.1f,%.3f,%ld,%lu,%lu,%lu,%lu,%lu,%d,%d\n", m.frame_id, m.i2cAddress, m.loop_start_ms, m.retCode, m.distances, m.p0_mm, median_ema_mm, median_ema_inches, m.p0_strength, m.t_setup, m.t_num, m.t_p0dist, m.t_p0str, m.total_ms, tNow, height_level);

        // For second SparkFun XM125 [NOT AVAILABLE YET]
        // Serial.printf("%lu,0x%02X,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,\n", m2.frame_id, m2.i2cAddress, m2.loop_start_ms, m2.retCode, m2.distances, m2.p0_mm, (int32_t)(filtered_mm2 + 0.5), m2.p0_strength, m2.t_setup, m2.t_num, m2.t_p0dist, m2.t_p0str, m2.total_ms);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void stepperTask(void* parameter) {
    static unsigned long prevMs = millis();

    while (true) {
        unsigned long nowMs = millis();

        if (nowMs - prevMs >= 20UL) {
            double dt = (nowMs - prevMs) / 1000.0;
            prevMs = nowMs;

            float measuredHeight;

            taskENTER_CRITICAL(&heightMux);
            measuredHeight = sharedHeightInches;
            taskEXIT_CRITICAL(&heightMux);

            if (measuredHeight >= 0.0f) {
                double deadband_err = targetHeightInches - measuredHeight;
                const double HEIGHT_DEADBAND = 0.15;

                if (fabs(deadband_err) <= HEIGHT_DEADBAND) {
                    commandedSpeed = 0.0;
                    integral = 0.0;
                } else {
                    double pidPercent = PID(kp, ki, kd, targetHeightInches, measuredHeight, &previousError, &integral, dt);
                    commandedSpeed = (pidPercent / 100.0) * MAX_SPEED;
                    commandedSpeed = applySpeedLimit(commandedSpeed);
                }

                stepper.setSpeed(commandedSpeed);
            } else {
                commandedSpeed = 0.0;
                stepper.setSpeed(0.0);
            }
        }

        stepper.runSpeed();
        taskYIELD();
    }
}
void setup() {
    Serial.begin(115200);
    gpio_pullup_en(GPIO_NUM_4);
    gpio_pullup_en(GPIO_NUM_5);
    delay(2000);

    Wire.begin(OLED_SDA, OLED_SCL);
    RadarWire.begin(RADAR_SDA, RADAR_SCL);
    RadarWire.setClock(400000);

    i2cScan(RadarWire);

    while (!radar1.begin())delay(1000);

    // while (!radar2.begin())delay(1000);

    OLED_init();
    Serial.println("OLED INIT DONE");
    delay(50);

    heightIndicatorInit();
    Serial.println("HEIGHT INDICATOR INIT DONE");
    delay(50);

    tInit = millis();
    Serial.println("RETRIEVED TIME INIT");

    xTaskCreatePinnedToCore(radarTask, "RadarTask", 8192, nullptr, 1, &radarTaskHandle, 1);
    xTaskCreatePinnedToCore(stepperTask, "StepperTask", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}