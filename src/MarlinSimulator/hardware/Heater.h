#pragma once

#include <cmath>
#include <functional>

#include "Gpio.h"

#include "../virtual_printer.h"

/**
 * data structures for future generalisation
 **/

struct enclosure_data {
  double temperature;
};

struct heater_data {
  double voltage, resistance;
};

struct hotend_data {
  double mass, surface_area, specific_heat;
};

struct adc_data {
  double pullup_resistance;
  uint32_t resolution;
};

class Heater: public VirtualPrinter::Component {
public:

  enum temperature_sensor_mode {
    Normal,
    ForceMin,
    ForceMax,
    ForcePause,
    Fixed       // Report sensor_fixed_temperature instead of the modeled temperature
  };

  Heater(pin_type heater_pin, pin_type adc_pin, heater_data heater, hotend_data hotend, adc_data adc, pin_type fan_pin=-1);
  virtual ~Heater();
  void interrupt(GpioEvent& ev);
  void update();
  void ui_widget();
  virtual void serialize(agent::JsonWriter& writer) const override;
  virtual bool apply(const agent::JsonValue& body, std::string& error) override;

  pin_type heater_pin, adc_pin;

  //heater element
  double heater_volts = 12.0;
  double heater_resistance = 3.6; // 40Watts

  //pwm sim
  uint64_t pwm_period = 0;
  uint64_t pwm_duty = 0;
  uint64_t pwm_hightick = 0;
  uint64_t pwm_lowtick = 0;
  uint64_t pwm_last_update = 0;

  //hotend block
  double hotend_ambient_temperature = 25.0;
  double hotend_temperature = 0.0;
  double hotend_energy = 0.0;
  double hotend_mass = 13.0; // g [approxiamte of 2x2x1 heatblock + nozzle]

  // // intermediates for sphere approximation
  // double hotend_density = 2.7; // gm/cm^3 (Aluminium)
  // double hotend_volume = hotend_mass / hotend_density; // cm^3
  // double hotend_radius = std::cbrt((hotend_volume * 3.0) / (4.0 * M_PI)); // cm
  // double hotend_surface_area = 4.0 * M_PI * hotend_radius * hotend_radius; // cm^2
  // // ****

  double hotend_surface_area = (16.0 + 4.0);// cm^2 [approxiamte of 2x2x1 heatblock + nozzle]
  double hotend_specific_heat = 0.897; // j/g/C (Aluminum)
  double hotend_convection_transfer = 0.001; // 0.001 W/cm^2 . C is an approximate often used for convective heat transfer into slow moving air

  //adc
  double adc_pullup_resistance = 4700;
  uint32_t adc_resolution = 12;
  temperature_sensor_mode m_temperature_sensor_mode {};
  double sensor_fixed_temperature = 25.0;  // Reading in Fixed mode (C)
  bool element_ok = true;                  // false = broken heater element: PWM delivers no power

  /**
   * Optional loads, all off by default so the plain model is unchanged.
   * Matches the terms of Marlin's MPC model so a measured M306 can be replayed:
   *   fan_xfer_coeff       (W/K) extra loss to ambient with the fan at 255 (MPC "F" term)
   *   fan_heater_loss      (W)   heater power lost while the fan runs at 255 (supply sag)
   *   filament_heat_capacity_permm (J/K/mm) energy carried away by extruded filament
   *   sensor_responsiveness (1/s) thermistor lag behind the block (0 = sensor reads the block)
   */
  void set_fan_pin(pin_type pin) { fan_pin = pin; }
  void set_filament_source(std::function<double()> src) { filament_mm = src; filament_last_mm = src ? src() : 0; }
  double fan_fraction() const;

  pin_type fan_pin = -1;
  double fan_xfer_coeff = 0.0;
  double fan_heater_loss = 0.0;
  double filament_heat_capacity_permm = 0.0;
  double sensor_responsiveness = 0.0;
  double sensor_temperature = 25.0;   // Lagged thermistor temperature (= block when responsiveness is 0)
  std::function<double()> filament_mm;
  double filament_last_mm = 0.0;

  // Instantaneous power terms from the last update, for observation (W)
  double power_heater = 0, power_ambient = 0, power_fan = 0, power_filament = 0;

  /**
   * Fan-coupled sensor error (e.g. fan PWM interference on the thermistor line),
   * scaled by fan duty and added to the reported reading only (not the physics):
   *   fan_sensor_offset  (C)  steady error at fan 255
   *   fan_sensor_ripple  (C)  sine amplitude at fan 255, period fan_sensor_ripple_period (s)
   *   fan_sensor_noise   (C)  standard deviation of per-read gaussian noise at fan 255
   */
  double fan_sensor_offset = 0.0;
  double fan_sensor_ripple = 0.0;
  double fan_sensor_ripple_period = 5.0;
  double fan_sensor_noise = 0.0;
  double sensor_error = 0.0;          // Error added to the last reading (C)
  uint64_t rng_state = 28420;         // Deterministic noise (xorshift; <random> clashes with Arduino.h macros)
  double gaussian();
};
