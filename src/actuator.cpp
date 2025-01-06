#include "newton/actuator.h"
#include <esp_log.h>

using namespace newton;

Actuator::Actuator(const int16_t pin_pwm, const uint16_t min_pulse_width, const uint16_t max_pulse_width) : m_pin_pwm(pin_pwm), m_min_pulse_width(min_pulse_width), m_max_pulse_width(max_pulse_width), m_timer(nullptr), m_operator(nullptr), m_comparator(nullptr), m_generator(nullptr)
{
    init_mcpwm();
}

Actuator::~Actuator()
{
    if (m_generator)
    {
        mcpwm_del_generator(m_generator);
    }

    if (m_comparator)
    {
        mcpwm_del_comparator(m_comparator);
    }

    if (m_operator)
    {
        mcpwm_del_operator(m_operator);
    }

    if (m_timer)
    {
        mcpwm_del_timer(m_timer);
    }

    m_generator = nullptr;
    m_comparator = nullptr;
    m_operator = nullptr;
    m_timer = nullptr;
}

void Actuator::set_pulse_width(uint16_t pulse_width)
{
    if (pulse_width < m_min_pulse_width)
    {
        ESP_LOGW(TAG, "Requested pulse width %d is below minimum %d", pulse_width, m_min_pulse_width);

        pulse_width = m_min_pulse_width;
    }

    if (pulse_width > m_max_pulse_width)
    {
        ESP_LOGW(TAG, "Requested pulse width %d is above maximum %d", pulse_width, m_max_pulse_width);

        pulse_width = m_max_pulse_width;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, pulse_width));
}

void Actuator::set_ranged(float ranged)
{
    if (ranged < -1.f)
    {
        ESP_LOGW(TAG, "Requested ranged %f is below minimum -1", ranged);

        ranged = -1.f;
    }

    if (ranged > 1.f)
    {
        ESP_LOGW(TAG, "Requested ranged %f is above maximum 1", ranged);

        ranged = 1.f;
    }

    const uint16_t pulse_width = m_min_pulse_width + (m_max_pulse_width - m_min_pulse_width) * (ranged + 1.f) / 2.f;

    set_pulse_width(pulse_width);
}

void Actuator::init_mcpwm()
{
    ESP_LOGI(TAG, "Initializing MCPWM");

    // Timer & operator

    ESP_LOGI(TAG, " Initializing timer & operator");

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1'000'000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20'000,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &m_timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &m_operator));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m_operator, m_timer));

    // Comparator & generator

    ESP_LOGI(TAG, " Initializing comparator & generator from operator");

    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_operator, &comparator_config, &m_comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = m_pin_pwm,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(m_operator, &generator_config, &m_generator));

    // Initial setpoint (middle of the range)
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, (m_min_pulse_width + m_max_pulse_width) / 2));

    // Generator actions on timer

    ESP_LOGI(TAG, " Setting generator actions on timer");

    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(m_generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(m_generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_comparator, MCPWM_GEN_ACTION_LOW)));

    // Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(m_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(m_timer, MCPWM_TIMER_START_NO_STOP));
}
