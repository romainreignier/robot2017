#ifndef _PWM_BASE_HPP_
#define _PWM_BASE_HPP_

class PWM {
    public:
        virtual void setPWM(float duty);
        virtual uint8_t getPWM(void);
        virtual void setFreq(uint32_t freq);
        virtual uint32_t getFreq(void);
        virtual void setChannel(uint16_t channel);
        virtual i2cflags_t getStatus();

    protected:
        uint32_t pwm_frequency;
        uint8_t pwm_duty;
        uint16_t pwm_channel;
};


#endif