#include <Arduino.h>

class Motor
{

public:
    int speed = 0;
    int ena_pin;
    int in_1;
    int in_2;
    uint8_t led_channel;
    float PID_value = 0;
    int min_threshold = 0;
    int max_threshold = 255;

    boolean reverse = false;
    Motor(int pins[3], uint8_t channel)
    {
        ena_pin = pins[0];
        in_1 = pins[1];
        in_2 = pins[2];
        led_channel = channel;
    }

    void setup()
    {
        ledcAttachPin(ena_pin, led_channel);
    }

    int run(boolean reverse = false, int speed_offset = 0)
    {

        // mapping speed in percentage to actual speed value
        int map_speed = map(speed, 0, 100, min_threshold, max_threshold);

        // Calculating the effective motor speed:
        int motor_speed = map_speed + PID_value;

        if (motor_speed < min_threshold)
        {
            reverse = true;
            motor_speed = (min_threshold - motor_speed) + min_threshold;
        }
        else
        {
            reverse = false;
        }

        motor_speed = constrain(motor_speed, 0, max_threshold);

        Serial.print("motor speed: ");
        Serial.print(motor_speed + speed_offset);
        Serial.print("\t Is reverse: ");
        Serial.print(reverse);

        digitalWrite(in_1, HIGH);
        digitalWrite(in_2, LOW);

        if (reverse)
        {
            digitalWrite(in_2, HIGH);
            digitalWrite(in_1, LOW);
        }
        ledcWrite(led_channel, motor_speed + speed_offset); // Motor Speed
        return motor_speed + speed_offset;
    }

    void stop()
    {
        digitalWrite(in_2, LOW);
        digitalWrite(in_1, LOW);
    }
};