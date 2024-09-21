#include <Arduino.h>
#include <motor_interface.h>

class Motor: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;

    protected:
        void forward(int pwm) override
        {
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, abs(pwm));
        }

    public:
        Motor(float pwm_frequency, int pwm_bits, bool invert, int unused, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }
    
        Motor(float pwm_frequency, int pwm_bits, bool invert, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }

        void brake() override
        {
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, 0);            
        }
};


// class Motor_control
// {
// private:
//     float PWM_FREQUENCY_ = 0;
//     float PWM_BITS_ = 0;
//     int PWM_PIN_ = 0;
//     int CW_PIN_ = 0;
//     int CCW_PIN_ = 0;
//     float INVERT_ = 0;
// public:

//     Motor_control(float pwm_frequency, float pwm_bits, float invert, float in_pwm, int in_cw, int in_ccw);
//     void spin(float pwm_val);
//     void init();
//     void brake();
//     void off();
// };

// Motor_control::Motor_control(float pwm_frequency, float pwm_bits, float invert, float in_pwm, int in_cw, int in_ccw):
//     PWM_FREQUENCY_(pwm_frequency),
//     PWM_BITS_(pwm_bits),
//     INVERT_(invert),
//     PWM_PIN_(in_pwm),
//     CW_PIN_(in_cw),
//     CCW_PIN_(in_ccw)
// {
// }

// void Motor_control::spin(float pwm_val){
//     if(pwm_val > 0){
//         analogWrite(CW_PIN_, fabs(pwm_val));
//         analogWrite(CCW_PIN_, 0);
//     }else if(pwm_val < 0){
//         analogWrite(CW_PIN_, 0);
//         analogWrite(CCW_PIN_, fabs(pwm_val));
//     }else {
//         analogWrite(CW_PIN_, 0);
//         analogWrite(CCW_PIN_,0);
//     }
// }
// void Motor_control::init(){
//     pinMode(CW_PIN_, OUTPUT);
//     pinMode(CCW_PIN_, OUTPUT);
//     analogWriteFrequency(CW_PIN_, PWM_FREQUENCY_);
//     analogWriteResolution(PWM_BITS_);
//     analogWrite(CW_PIN_, 0);
//     analogWrite(CCW_PIN_,0);
// }

// void Motor_control::brake(){
//     analogWrite(CW_PIN_, 0);
//     analogWrite(CCW_PIN_, 0);
// }
