#pragma once

#include <cstdint>
#include "driver/mcpwm_prelude.h"

namespace newton
{
    /// @brief Class for controlling an actuator: will eventually include interfacing with various sensors to have a deeper understanding of the motor's, therefore actuator's, state.
    class Actuator
    {

    public:
        /// @brief Construct a new Actuator object.
        /// @param pin_pwm The PWM pin to control the actuator.
        /// @param min_pulse_width The minimum pulse width in microseconds.
        /// @param max_pulse_width The maximum pulse width in microseconds.
        Actuator(const int16_t pin_pwm, const uint16_t min_pulse_width = 1100, const uint16_t max_pulse_width = 1900);
        ~Actuator();

        /// @brief Set the pulse width of the actuator in microseconds, clamped to the minimum and maximum pulse width.
        /// @param pulse_width The pulse width in microseconds.
        void set_pulse_width(uint16_t pulse_width);

        /// @brief Set the ranged value of the actuator, clamped to -1 and 1. The ranged value is converted to a pulse width.
        /// @param ranged The ranged value [-1, 1].
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