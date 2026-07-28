#include <HeightIndicator.h>
#include <Arduino.h>

#define LED_RED_HIGH 42
#define LED_YELLOW_HIGH 48
#define LED_GREEN 47
#define LED_YELLOW_LOW 45
#define LED_RED_LOW 46

/*
Initializes the LED indicator (all on)
*/
void heightIndicatorInit() 
{
    pinMode(LED_RED_HIGH, OUTPUT);
    Serial.println("PINMODE: R_H");
    pinMode(LED_YELLOW_HIGH, OUTPUT);
    Serial.println("PINMODE: Y_H");
    pinMode(LED_GREEN, OUTPUT);
    Serial.println("PINMODE: G");
    pinMode(LED_YELLOW_LOW, OUTPUT);
    Serial.println("PINMODE: Y_L");
    pinMode(LED_RED_LOW, OUTPUT);
    Serial.println("PINMODE: R_L");

    digitalWrite(LED_RED_HIGH, HIGH);
    digitalWrite(LED_YELLOW_HIGH, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW_LOW, HIGH);
    digitalWrite(LED_RED_LOW, HIGH);
    Serial.println("HEIGHT INDICATOR DIGITAL WRITE DONE");
}

/*
Changes the LED indicator based on distance read
*/
int heightIndicatorUpdate(float height)
{
    // Write the LEDS
    digitalWrite(LED_RED_HIGH, height > 19.0);
    digitalWrite(LED_YELLOW_HIGH, height <= 19.0 && height > 17.0);
    digitalWrite(LED_GREEN, height <= 17.0 && height >= 15.0);
    digitalWrite(LED_YELLOW_LOW, height < 15.0 && height >= 13.0);
    digitalWrite(LED_RED_LOW, height < 13.0);

    // Return the LED code
    if(height > 19.0)
    {
        return 5;
    }
    else if(height <= 19.0 && height > 17.0)
    {
        return 4;
    }
    else if(height <= 17.0 && height >= 15.0)
    {
        return 3;
    }
    else if(height < 15.0 && height >= 13.0)
    {
        return 2;
    }
    else if(height < 13.0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}