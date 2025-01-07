#pragma once

#include "driver/pulse_cnt.h"

namespace newton
{
  /// @brief Class for interfacing with an quadrature, rotary encoder.
  class Encoder
  {
  public:
    /// @brief Construct a new Encoder object.
    /// @param pin_a GPIO pin connected to channel A of the encoder.
    /// @param pin_b GPIO pin connected to channel B of the encoder.
    /// @param ppr Pulses per revolution of the encoder (CPR x 4, for a quadrature encoder).
    Encoder(unsigned int pin_a, unsigned int pin_b, int ppr);
    ~Encoder();

    /// @brief Update the encoder's position and velocity.
    /// @note For best results, call this method at a short & regular interval.
    void update();

    /// @brief Get the total count of the encoder (in counts).
    /// @return The total count of the encoder since boot.
    int get_count() const;

    /// @brief Get the position of the encoder (in revolutions).
    /// @return The position of the encoder since boot.
    float get_position() const;

    /// @brief Get the velocity of the encoder (in revolutions per second).
    /// @return The velocity of the encoder since boot.
    float get_velocity() const;

  private:
    void init_pcnt();
    static bool on_reach(pcnt_unit_handle_t unit,
                         const pcnt_watch_event_data_t *edata, void *user_ctx);

    const unsigned int m_pin_a;
    const unsigned int m_pin_b;
    const int m_cpr;
    int m_total_count;
    float m_position;
    float m_velocity;
    int m_last_count;
    int m_last_time;
    pcnt_unit_handle_t m_pcnt_unit;
  };
}; // namespace newton