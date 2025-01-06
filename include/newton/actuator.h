#pragma once

#include <cstdint>
#include "driver/mcpwm_prelude.h"

namespace newton
{
    class Actuator
    {

    public:
        Actuator(const int16_t pin_pwm, const uint16_t min_pulse_width = 1100, const uint16_t max_pulse_width = 1900);
        ~Actuator();

        void set_pulse_width(uint16_t pulse_width);
        void set_ranged(float ranged);

    private:
        void init_mcpwm();

        const char *TAG = "newton::Actuator";

        const uint16_t m_pin_pwm;
        const uint16_t m_min_pulse_width;
        const uint16_t m_max_pulse_width;

        mcpwm_timer_handle_t m_timer;
        mcpwm_oper_handle_t m_operator;
        mcpwm_cmpr_handle_t m_comparator;
        mcpwm_gen_handle_t m_generator;
    };
} // namespace newton