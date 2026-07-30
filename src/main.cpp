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
#define XM125_RST 4

volatile float sharedHeightInches = -1.0f;
portMUX_TYPE heightMux = portMUX_INITIALIZER_UNLOCKED;

TwoWire RadarWire = TwoWire(1);
XM125Radar radar1(0x51, RadarWire);

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

const float alpha = 0.5;
uint32_t tInit = 0;

TaskHandle_t radarTaskHandle = nullptr;
TaskHandle_t pidTaskHandle = nullptr;
TaskHandle_t stepperTaskHandle = nullptr;

hw_timer_t* stepperTimer = nullptr;
constexpr uint32_t STEPPER_SERVICE_HZ = 3000;

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
    if (found == 0) {Serial.println("No I2C devices found.");
    }else {Serial.printf("Done. Found %d device(s).\n", found);}
    Serial.println();
}

float getMedian(float raws[num_raws]) {
    float temp[num_raws];
    for (int i = 0; i < num_raws; i++) {temp[i] = raws[i];}//cpy array
    for (int i = 0; i < (num_raws - 1); i++) {//le sort
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

            if (pidTaskHandle != nullptr) xTaskNotifyGive(pidTaskHandle);

            snprintf(buf, sizeof(buf), "%.3f", median_ema_inches /*(int32_t)(median_ema_mm + 0.5)*/);
        } else if (raw_mm >= 1000) {
            snprintf(buf, sizeof(buf), ">= 1000!");
        } else {
            snprintf(buf, sizeof(buf), "NO PEAK");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        uint32_t tNow = millis() - tInit;

        char buffer[64];   // expand size for two radars [WAITING FOR ADDITIONAL RADARS]
        snprintf(buffer, sizeof(buffer), "%s", buf/*, buf2*/);
        OLED_writeText(buffer, 4, u8x8_font_chroma48medium8_r);
        height_level = heightIndicatorUpdate(median_ema_inches);

        Serial.printf("%lu,0x%02X,%lu,%lu,%lu,%ld,%.1f,%.3f,%ld,%lu,%lu,%lu,%lu,%lu,%d,%d\n", m.frame_id, m.i2cAddress, m.loop_start_ms, m.retCode, m.distances, m.p0_mm, median_ema_mm, median_ema_inches, m.p0_strength, m.t_setup, m.t_num, m.t_p0dist, m.t_p0str, m.total_ms, tNow, height_level);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pidTask(void* parameter) {
    constexpr double HEIGHT_DEADBAND = 0.15;
    uint32_t previousMeasurementMs = millis();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t nowMs = millis();
        double dt = (nowMs - previousMeasurementMs) / 1000.0;
        previousMeasurementMs = nowMs;

        float measuredHeight;

        taskENTER_CRITICAL(&heightMux);
        measuredHeight = sharedHeightInches;
        taskEXIT_CRITICAL(&heightMux);

        double newSpeed = 0.0;

        if (measuredHeight >= 0.0f && dt > 0.0) {
            double error = targetHeightInches - measuredHeight;

            if (fabs(error) <= HEIGHT_DEADBAND) {
                integral = 0.0;
                previousError = error;
            } else {
                double pidPercent = PID(kp, ki, kd, targetHeightInches, measuredHeight, &previousError, &integral, dt);
                newSpeed = applySpeedLimit((pidPercent / 100.0) * MAX_SPEED);
            }
        } else {
            integral = 0.0;
            previousError = 0.0;
        }

        taskENTER_CRITICAL(&heightMux);
        commandedSpeed = newSpeed;
        taskEXIT_CRITICAL(&heightMux);
    }
}

void ARDUINO_ISR_ATTR stepperTimerISR() {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (stepperTaskHandle != nullptr)vTaskNotifyGiveFromISR(stepperTaskHandle, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void stepperTask(void* parameter) {
    double appliedSpeed = 0.0;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        double requestedSpeed;

        taskENTER_CRITICAL(&heightMux);
        requestedSpeed = commandedSpeed;
        taskEXIT_CRITICAL(&heightMux);

        if (requestedSpeed != appliedSpeed) {
            appliedSpeed = requestedSpeed;
            stepper.setSpeed(appliedSpeed);
        }

        stepper.runSpeed();
    }
}

void setup() {
    Serial.begin(115200);
    gpio_pullup_en((gpio_num_t)RADAR_SDA);
    gpio_pullup_en((gpio_num_t)RADAR_SCL);
    delay(100);

    pinMode(XM125_RST, OUTPUT);
    digitalWrite(XM125_RST, LOW);
    delay(100);
    digitalWrite(XM125_RST, HIGH);
    delay(100);

    Wire.begin(OLED_SDA, OLED_SCL);
    RadarWire.begin(RADAR_SDA, RADAR_SCL);
    RadarWire.setClock(400000);

    i2cScan(RadarWire);

    while (!radar1.begin())delay(1000);

    OLED_init();
    Serial.println("OLED INIT DONE");

    heightIndicatorInit();
    Serial.println("HEIGHT INDICATOR INIT DONE");

    PID_setup();

    tInit = millis();
    Serial.println("RETRIEVED TIME INIT");

    xTaskCreatePinnedToCore(pidTask, "PIDTask", 4096, nullptr, 2, &pidTaskHandle, 0);
    xTaskCreatePinnedToCore(stepperTask, "StepperTask", 4096, nullptr, 3, &stepperTaskHandle, 0);
    xTaskCreatePinnedToCore(radarTask, "RadarTask", 8192, nullptr, 1, &radarTaskHandle, 1);

    stepperTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepperTimer, &stepperTimerISR, true);
    timerAlarmWrite(stepperTimer, 1000000UL / STEPPER_SERVICE_HZ, true);
    timerAlarmEnable(stepperTimer);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}