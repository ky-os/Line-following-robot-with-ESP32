class PID
{

public:
    float Kp, Ki, Kd;

    PID(float _Kp, float _Ki, float _Kd)
    {
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
    }

    float calculate(int error)
    {
        P = error;
        I = I + previous_I;
        D = error - previous_error;

        float PID_value = (Kp * P) + (Ki * I) + (Kd * D); // motor must run faster

        previous_I = I;
        previous_error = error;

        return PID_value;
    }

private:
    float P = 0, I = 0, D = 0;
    float previous_error = 0, previous_I = 0;
};