#include <Arduino.h>
#include <Wire.h>
#include "xm125.h"
#include "OLED.h"
#include "driver/gpio.h"
#include "Ringbuf.h"
#include "PID.h"

#define OLED_SDA 17
#define OLED_SCL 18

#define RADAR_SDA 4
#define RADAR_SCL 5
TwoWire RadarWire = TwoWire(1);
XM125Radar radar1(0x51,RadarWire);
XM125Radar radar2(SFE_XM125_I2C_ADDRESS,RadarWire);

float filtered_mm = -1.0;
float filtered_mm2 = -1.0;
const float alpha = 0.6;
const float alpha2 = 0.3;
uint32_t tInit = 0;

uint16_t lowbound = 350;
uint16_t highbound = 450;

void i2cScan(TwoWire& wireBus){
    byte error;
  int found = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    wireBus.beginTransmission(address);
    error = wireBus.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      found++;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (address < 16) Serial.print("0");
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

void setup() {
    Serial.begin(115200);
    gpio_pullup_en(GPIO_NUM_4);
    gpio_pullup_en(GPIO_NUM_5);
    delay(2000);
    
    Wire.begin(OLED_SDA, OLED_SCL);
    RadarWire.begin(RADAR_SDA, RADAR_SCL);
    RadarWire.setClock(400000);

    i2cScan(RadarWire);

    while (!radar1.begin()) {
        delay(1000);
    }

    // while (!radar2.begin()) {
    //     Serial.println("Radar 2 failed");
    //     delay(1000);
    // }

    OLED_init();
    tInit = millis();

    PID_setup();
}
void loop() {
    XM125Radar::RadarMeasurement m = radar1.measure();
    
    int32_t raw_mm = m.p0_mm;
    char buf[32];

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

    //PID stuff//////////////////////////////////
    static unsigned long prevUs = millis();
    unsigned long nowUs = millis();

    if(nowUs-prevUs >= 20UL){
        double dt = (nowUs-prevUs)/1000.0; //dt in seconds
        prevUs = nowUs;
        double measuredHeight = filtered_mm/25.4;//////here in inches
        double deadband_err = targetHeightInches - measuredHeight;

        const double HEIGHT_DEADBAND = 0.15;//inches
        if (fabs(deadband_err) <= HEIGHT_DEADBAND) {//deadband 
            commandedSpeed = 0.0;
            integral = 0.0;
        }else{
        double pidPercent = PID(kp, ki, kd, targetHeightInches, measuredHeight, &previousError, &integral, dt);
        commandedSpeed = (pidPercent/100.0)*MAX_SPEED;
        commandedSpeed = applySpeedLimit(commandedSpeed);
        }
        stepper.setSpeed(commandedSpeed);
    }
    ////////////////////////////////////////////////

  // Call this as frequently as possible.
  stepper.runSpeed();

    delay(10);

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

    uint32_t tNow = millis()-tInit;

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s", buf/*, buf2*/);
    OLED_writeText(buffer, 4, u8x8_font_chroma48medium8_r);

    Serial.printf("%lu,0x%02X,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%d,\n",
        m.frame_id,
        m.i2cAddress,
        m.loop_start_ms,
        m.retCode,
        m.distances,
        m.p0_mm,
        (int32_t)(filtered_mm + 0.5),
        m.p0_strength,
        m.t_setup,
        m.t_num,
        m.t_p0dist,
        m.t_p0str,
        m.total_ms,
        tNow
    );
    // Serial.printf("%lu,0x%02X,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,\n",
    //     m2.frame_id,
    //     m2.i2cAddress,
    //     m2.loop_start_ms,
    //     m2.retCode,
    //     m2.distances,
    //     m2.p0_mm,
    //     (int32_t)(filtered_mm2 + 0.5),
    //     m2.p0_strength,
    //     m2.t_setup,
    //     m2.t_num,
    //     m2.t_p0dist,
    //     m2.t_p0str,
    //     m2.total_ms
    // );

    delay(20);
}