#ifndef _PWM_BASE_HPP_
#define _PWM_BASE_HPP_

class PWM {
    public:
        virtual void setPWM(float duty);
        virtual uint8_t getPWM(void);
        virtual void setFreq(float freq);
        virtual float getFreq(void);
        virtual void setChannel(uint16_t channel);
        virtual i2cflags_t getStatus();

    protected:
        float pwm_frequency;
        uint8_t pwm_duty;
        uint16_t pwm_channel;
};


#endif
