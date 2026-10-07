#include <imgui.h>
#include <algorithm>
#include <iterator>

#include "pinmapping.h"
#include "Heater.h"
#include "pwm_reader.h"
#include "../agent/json.h"

constexpr double absolute_zero_offset = -273.15;
double thermistor_ext_coef[] = {
  7.611428226793945e-04,
  2.011100481838449e-04,
  1.914201231699539e-06,
  1.561937567632929e-08
};

double temperature_to_resistance(double t) {
	double r, u, v, p, q, b, c, d;
	t = t - absolute_zero_offset;
	d = (thermistor_ext_coef[0] - 1.0 / t) / thermistor_ext_coef[3];
	c = thermistor_ext_coef[1] / thermistor_ext_coef[3];
	b = thermistor_ext_coef[2] / thermistor_ext_coef[3];
	q = 2.0 / 27.0 * b * b * b - 1.0 / 3.0 * b * c + d;
	p = c - 1.0 / 3.0 * b * b;
	v = - pow(q / 2.0 + sqrt(q * q / 4.0 + p * p * p / 27.0), 1.0 / 3.0);
	u =   pow(-q / 2.0 + sqrt(q * q / 4.0 + p * p * p / 27.0), 1.0 / 3.0);
	r  = exp(u + v - b / 3.0);
	return r;
}

Heater::Heater(pin_type heater_pin, pin_type adc_pin, heater_data heater, hotend_data hotend, adc_data adc, pin_type fan_pin) : VirtualPrinter::Component("Heater"), heater_pin(heater_pin), adc_pin(analogInputToDigitalPin(adc_pin)) {
  this->fan_pin = fan_pin;
  heater_resistance = heater.resistance;
  heater_volts = heater.voltage;

  hotend_mass = hotend.mass;
  hotend_surface_area = hotend.surface_area;
  hotend_specific_heat = hotend.specific_heat;

  adc_resolution = adc.resolution;
  adc_pullup_resistance = adc.pullup_resistance;

  Gpio::attach(this->adc_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(this->heater_pin, [this](GpioEvent& event){ this->interrupt(event); });
  hotend_energy = hotend_ambient_temperature * (hotend_specific_heat * hotend_mass);
  hotend_temperature = sensor_temperature = hotend_ambient_temperature;
  static uint64_t elementid = 0;
  add_component<PWMReader>("Element" + std::to_string(elementid), heater_pin);
  ++elementid;
}

Heater::~Heater() {
}

void Heater::update() {

}

// Sensor mode names, shared by serialize() and apply(), in enum order
static const char* const sensor_mode_names[] = { "Normal", "ForceMin", "ForceMax", "ForcePause", "Fixed" };

void Heater::serialize(agent::JsonWriter& writer) const {
  writer.begin_object();
  writer.member("temperature", hotend_temperature);
  writer.member("ambient_temperature", hotend_ambient_temperature);
  writer.member("energy", hotend_energy);
  writer.member("sensor_mode", sensor_mode_names[m_temperature_sensor_mode]);
  writer.member("sensor_temperature", sensor_fixed_temperature);
  writer.member("element_ok", element_ok);
  writer.member("heater_pin", int64_t(heater_pin));
  writer.member("adc_pin", int64_t(adc_pin));
  writer.member("pwm_duty", pwm_duty);
  writer.member("pwm_period", pwm_period);
  writer.member("heater_volts", heater_volts);
  writer.member("heater_resistance", heater_resistance);
  // MPC-equivalent model terms and live power balance
  writer.member("heater_power", (heater_volts * heater_volts) / heater_resistance);
  writer.member("heat_capacity", hotend_specific_heat * hotend_mass);
  writer.member("ambient_xfer", hotend_convection_transfer * hotend_surface_area);
  writer.member("fan_xfer", fan_xfer_coeff);
  writer.member("fan_heater_loss", fan_heater_loss);
  writer.member("filament_heat_capacity_permm", filament_heat_capacity_permm);
  writer.member("sensor_responsiveness", sensor_responsiveness);
  writer.member("sensor_reading", sensor_temperature);
  writer.member("fan_pin", int64_t(fan_pin));
  writer.member("fan_fraction", fan_fraction());
  writer.member("power_heater", power_heater);
  writer.member("power_ambient", power_ambient);
  writer.member("power_fan", power_fan);
  writer.member("power_filament", power_filament);
  writer.member("fan_sensor_offset", fan_sensor_offset);
  writer.member("fan_sensor_ripple", fan_sensor_ripple);
  writer.member("fan_sensor_ripple_period", fan_sensor_ripple_period);
  writer.member("fan_sensor_noise", fan_sensor_noise);
  writer.member("sensor_error", sensor_error);
  writer.end_object();
}

void Heater::ui_widget() {
  ImGui::Text("Temperature: %f", hotend_temperature);
  if (sensor_responsiveness > 0) ImGui::Text("Sensor: %f", sensor_temperature);
  if (Gpio::valid_pin(fan_pin)) ImGui::Text("Fan: %.0f%%  Loss: ambient %.2fW fan %.2fW filament %.2fW", fan_fraction() * 100, power_ambient, power_fan, power_filament);
  static const char* items[] = { "Normal", "Force Min", "Force Max", "Force Pause", "Fixed" };
  int mode = m_temperature_sensor_mode;
  if (ImGui::Combo("Sensor Mode", &mode, items, IM_ARRAYSIZE(items))) {
    m_temperature_sensor_mode = temperature_sensor_mode(mode);
  }
  if (m_temperature_sensor_mode == Fixed) {
    float t = sensor_fixed_temperature;
    if (ImGui::InputFloat("Sensor Reading (C)", &t, 1.0f, 10.0f, "%.1f")) sensor_fixed_temperature = t;
  }
  ImGui::Checkbox("Heater Element OK", &element_ok);
}

/**
 * Agent fault injection, by name instead of pin or ADC channel:
 *   {"sensor_mode": "Normal|ForceMin|ForceMax|ForcePause|Fixed"}
 *   {"sensor_temperature": 300}   the Fixed-mode reading; implies Fixed
 *   {"element_ok": false}         broken element: heating power is lost
 *   {"temperature": 60}           set the modeled temperature (e.g. skip a cool-down)
 *   {"ambient_temperature": 30}
 * Thermal model, in the units of Marlin MPC (M306):
 *   {"heater_power": 64}          (W)    MPC P
 *   {"heat_capacity": 19.3}       (J/K)  MPC C
 *   {"ambient_xfer": 0.069}       (W/K)  MPC A
 *   {"fan_xfer": 0.046}           (W/K)  MPC F (extra loss with the fan at full speed)
 *   {"fan_heater_loss": 5}        (W)    heater power stolen by the fan at full speed
 *   {"filament_heat_capacity_permm": 0.0036}  (J/K/mm) MPC H
 *   {"sensor_responsiveness": 0.26}           (1/s)    MPC R (0 = no sensor lag)
 * Fan-coupled sensor error, scaled by fan duty, added to the reading only:
 *   {"fan_sensor_offset": -2}     (C)  steady error at fan 255
 *   {"fan_sensor_ripple": 3, "fan_sensor_ripple_period": 5}  (C, s) sine error
 *   {"fan_sensor_noise": 1}       (C)  gaussian per-read noise (std dev)
 */
bool Heater::apply(const agent::JsonValue& body, std::string& error) {
  bool applied = false;

  std::string mode_name;
  if (body.get_string("sensor_mode", mode_name)) {
    int found = -1;
    for (int i = 0; i < int(std::size(sensor_mode_names)); ++i)
      if (mode_name == sensor_mode_names[i]) found = i;
    if (found < 0) {
      error = "unknown sensor_mode '" + mode_name + "' (Normal, ForceMin, ForceMax, ForcePause, Fixed)";
      return false;
    }
    m_temperature_sensor_mode = temperature_sensor_mode(found);
    applied = true;
  }

  double value;
  if (body.get_double("sensor_temperature", value)) {
    sensor_fixed_temperature = value;
    if (!body.has("sensor_mode")) m_temperature_sensor_mode = Fixed;
    applied = true;
  }

  bool flag;
  if (body.get_bool("element_ok", flag)) { element_ok = flag; applied = true; }

  if (body.get_double("ambient_temperature", value)) { hotend_ambient_temperature = value; applied = true; }

  // Thermal model in MPC units (W, J/K, W/K), e.g. replayed from a measured M306
  if (body.get_double("heater_power", value) && value > 0) { heater_resistance = (heater_volts * heater_volts) / value; applied = true; }
  if (body.get_double("heat_capacity", value) && value > 0) {
    const double t = hotend_energy / (hotend_specific_heat * hotend_mass);
    hotend_mass = value / hotend_specific_heat;
    hotend_energy = t * value;
    applied = true;
  }
  if (body.get_double("ambient_xfer", value)) { hotend_convection_transfer = value / hotend_surface_area; applied = true; }
  if (body.get_double("fan_xfer", value)) { fan_xfer_coeff = value; applied = true; }
  if (body.get_double("fan_heater_loss", value)) { fan_heater_loss = value; applied = true; }
  if (body.get_double("filament_heat_capacity_permm", value)) { filament_heat_capacity_permm = value; applied = true; }
  if (body.get_double("sensor_responsiveness", value)) { sensor_responsiveness = value; applied = true; }
  if (body.get_double("fan_sensor_offset", value)) { fan_sensor_offset = value; applied = true; }
  if (body.get_double("fan_sensor_ripple", value)) { fan_sensor_ripple = value; applied = true; }
  if (body.get_double("fan_sensor_ripple_period", value)) { fan_sensor_ripple_period = value; applied = true; }
  if (body.get_double("fan_sensor_noise", value)) { fan_sensor_noise = value; applied = true; }

  if (body.get_double("temperature", value)) {
    hotend_temperature = sensor_temperature = value;
    hotend_energy = value * (hotend_specific_heat * hotend_mass);
    applied = true;
  }

  if (!applied) error = "no recognised field (sensor_mode, sensor_temperature, element_ok, temperature, ambient_temperature, heater_power, heat_capacity, ambient_xfer, fan_xfer, fan_heater_loss, filament_heat_capacity_permm, sensor_responsiveness, fan_sensor_offset, fan_sensor_ripple, fan_sensor_ripple_period, fan_sensor_noise)";
  return applied;
}

// Standard normal sample (xorshift64* + Box-Muller), reproducible run to run
double Heater::gaussian() {
  auto uniform = [this]() {
    rng_state ^= rng_state >> 12; rng_state ^= rng_state << 25; rng_state ^= rng_state >> 27;
    return ((rng_state * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0) + 1e-12;
  };
  return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2 * M_PI * uniform());
}

// Fan duty 0..1. Hardware PWM stores 0..UINT16_MAX (HAL set_pwm_duty); soft PWM / digital stores 0 or 1.
double Heater::fan_fraction() const {
  if (!Gpio::valid_pin(fan_pin)) return 0.0;
  const uint16_t v = Gpio::get_pin_value(fan_pin);
  return v <= 1 ? double(v) : double(v) / UINT16_MAX;
}

void Heater::interrupt(GpioEvent& ev) {
  // always update the temperature
  double time_delta = Kernel::TimeControl::ticksToNanos(ev.timestamp - pwm_last_update) / (double)Kernel::TimeControl::ONE_BILLION;
  const double heat_capacity = hotend_specific_heat * hotend_mass,   // J/K
               block_temp = hotend_energy / heat_capacity,
               over_ambient = block_temp - hotend_ambient_temperature,
               fan = fan_fraction(),
               heater_on = element_ok ? Gpio::get_pin_value(heater_pin) : 0;

  power_heater = heater_on ? std::max(0.0, (heater_volts * heater_volts) / heater_resistance - fan * fan_heater_loss) : 0.0;
  power_ambient = hotend_convection_transfer * hotend_surface_area * over_ambient;
  power_fan = fan * fan_xfer_coeff * over_ambient;

  // Filament pulled through the nozzle carries heat away (retractions are ignored, as in MPC)
  double filament_energy = 0;
  if (filament_mm) {
    const double mm = filament_mm(), de = mm - filament_last_mm;
    filament_last_mm = mm;
    if (de > 0) filament_energy = de * filament_heat_capacity_permm * over_ambient;
    power_filament = time_delta > 0 ? filament_energy / time_delta : 0;
  }

  hotend_energy += (power_heater - power_ambient - power_fan) * time_delta - filament_energy;
  pwm_last_update = ev.timestamp;
  hotend_temperature = hotend_energy / heat_capacity;

  // First-order thermistor lag behind the block
  if (sensor_responsiveness > 0)
    sensor_temperature += (hotend_temperature - sensor_temperature) * (1.0 - std::exp(-sensor_responsiveness * time_delta));
  else
    sensor_temperature = hotend_temperature;

  if (ev.event == ev.RISE && ev.pin_id == heater_pin) {
    if (pwm_hightick) pwm_period = ev.timestamp - pwm_hightick;
    pwm_hightick = ev.timestamp;

  } else if ( ev.event == ev.FALL && ev.pin_id == heater_pin) {
    pwm_lowtick = ev.timestamp;
    pwm_duty = ev.timestamp - pwm_hightick;
  } else if (ev.event == ev.GET_VALUE && ev.pin_id == adc_pin) {
    double reading = sensor_temperature;
    sensor_error = 0;
    if (fan_sensor_offset != 0 || fan_sensor_ripple != 0 || fan_sensor_noise != 0) {
      const double f = fan_fraction();
      if (f > 0) {
        const double t = Kernel::TimeControl::ticksToNanos(ev.timestamp) / (double)Kernel::TimeControl::ONE_BILLION;
        sensor_error = f * fan_sensor_offset;
        if (fan_sensor_ripple_period > 0) sensor_error += f * fan_sensor_ripple * std::sin(2 * M_PI * t / fan_sensor_ripple_period);
        if (fan_sensor_noise > 0) sensor_error += f * fan_sensor_noise * gaussian();
        reading += sensor_error;
      }
    }
    double thermistor_resistance = temperature_to_resistance(reading);
    uint32_t adc_reading = (uint32_t)((((1U << adc_resolution) -1)  * thermistor_resistance) / (adc_pullup_resistance + thermistor_resistance));
    switch (m_temperature_sensor_mode) {
      case Normal: Gpio::set_pin_value(adc_pin, adc_reading); break;
      case ForceMin: Gpio::set_pin_value(adc_pin, 0); break;
      case ForceMax: Gpio::set_pin_value(adc_pin, UINT16_MAX); break;
      case ForcePause: break;
      case Fixed: {
        const double r = temperature_to_resistance(sensor_fixed_temperature);
        Gpio::set_pin_value(adc_pin, uint32_t((((1U << adc_resolution) - 1) * r) / (adc_pullup_resistance + r)));
      } break;
    }
  }
}
