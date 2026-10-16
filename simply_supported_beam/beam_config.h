#ifndef BEAM_CONFIG_H
#define BEAM_CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

/*
  CBeamConfig
  ------------------------------------------------------------
  Patron derivado conceptualmente de CDoorConfig.

  Responsabilidad:
    - configuracion persistente NVS / Preferences
    - copia RAM de parametros
    - entrada host JSON por Serial
    - pedidos runtime pendientes para que el .ino / FSM decida

  La salida JSON pertenece a CBeamMessaging.

  Regla:
    - NO mueve motor
    - NO lee FC_HOME para tomar decisiones
    - NO lee HX711
    - NO ejecuta homing
    - NO ejecuta calibracion
    - NO cambia st_mode ni estados operativos
    - es la unica clase que accede directamente a Preferences / NVS
*/

// ------------------------------------------------------------
// NVS
// ------------------------------------------------------------
#define BEAM_CFG_NAMESPACE       "beamcfg"
#define BEAM_CFG_MAGIC_NUMBER    220826UL
#define BEAM_CFG_SCHEMA_VERSION  4UL

// ------------------------------------------------------------
// Defaults persistentes
// ------------------------------------------------------------
#define BEAM_STEP_PERIOD_US_DEFAULT         5000UL
#define BEAM_HOME_SLOW_PERIOD_US_DEFAULT   10000UL
#define BEAM_HOME_BACKOFF_STEPS_DEFAULT       20UL
#define BEAM_HOME_DEBOUNCE_MS_DEFAULT         15UL
#define BEAM_HOME_SEEK_MAX_STEPS_DEFAULT    1000UL

// Movimiento absoluto de ensayo.
// 0 en max_position_steps significa: limite aun no configurado.
#define BEAM_MOVE_FAST_PERIOD_US_DEFAULT     5000UL
#define BEAM_MAX_POSITION_STEPS_DEFAULT         0UL

// Calibracion de celdas.
#define BEAM_CELL_CAL_A_DEFAULT  0.0
#define BEAM_CELL_CAL_B_DEFAULT  0.0

// Calibracion motor-viga.
// Solo es valida cuando motor_calibrated == true.
#define BEAM_MOTOR_MAX_FORCE_G_DEFAULT             0.0
#define BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT    0UL
#define BEAM_MOTOR_STEP_500_G_DEFAULT              0UL
#define BEAM_MOTOR_STEP_1000_G_DEFAULT             0UL
#define BEAM_MOTOR_STEP_1500_G_DEFAULT             0UL

// ------------------------------------------------------------
// Defaults runtime NO persistentes
// ------------------------------------------------------------
#define BEAM_LOAD_STREAM_DEFAULT             false
#define BEAM_LOAD_STREAM_PERIOD_MS_DEFAULT   500UL

#define BEAM_HOST_RX_SIZE 256

enum BeamHostRequest : uint8_t {
  BEAM_REQ_NONE = 0,
  BEAM_REQ_HOME,
  BEAM_REQ_STOP,
  BEAM_REQ_ENABLE,
  BEAM_REQ_DISABLE,
  BEAM_REQ_MOVE_ABS_STEPS,
  BEAM_REQ_READ_LOADS,
  BEAM_REQ_APPLY_FORCE,
  BEAM_REQ_MODE_MANUAL,
  BEAM_REQ_MODE_CALIBRATION_CELL,
  BEAM_REQ_MODE_CALIBRATION_MOTOR,
  BEAM_REQ_MODE_EXPERIMENT,
  BEAM_REQ_FACTORY_RESET
};

class CBeamConfig {
public:
  CBeamConfig();

  bool init();
  void set_app_version(const char* version);
  void host_cmd();

  // ----------------------------------------------------------
  // Copia RAM de configuracion persistente
  // ----------------------------------------------------------
  uint32_t get_step_period_us() const;
  uint32_t get_home_slow_period_us() const;
  uint32_t get_home_backoff_steps() const;
  uint32_t get_home_debounce_ms() const;
  uint32_t get_home_seek_max_steps() const;
  uint32_t get_move_fast_period_us() const;
  uint32_t get_max_position_steps() const;

  double get_cell1_a() const;
  double get_cell1_b() const;
  bool is_cell1_calibrated() const;

  double get_cell2_a() const;
  double get_cell2_b() const;
  bool is_cell2_calibrated() const;

  double get_motor_max_force_g() const;
  uint32_t get_motor_contact_offset_steps() const;
  uint32_t get_motor_step_500_g() const;
  uint32_t get_motor_step_1000_g() const;
  uint32_t get_motor_step_1500_g() const;
  bool is_motor_calibrated() const;

  // Runtime, no persistente.
  bool get_load_stream_enabled() const;
  uint32_t get_load_stream_period_ms() const;

  // ----------------------------------------------------------
  // Setters RAM + NVS
  // ----------------------------------------------------------
  bool set_step_period_us(uint32_t value);
  bool set_home_slow_period_us(uint32_t value);
  bool set_home_backoff_steps(uint32_t value);
  bool set_home_debounce_ms(uint32_t value);
  bool set_home_seek_max_steps(uint32_t value);
  bool set_move_fast_period_us(uint32_t value);
  bool set_max_position_steps(uint32_t value);

  bool set_cell1_calibration(double a, double b);
  bool set_cell2_calibration(double a, double b);

  // Guarda el resultado final de la calibracion motor-viga.
  // El offset es la posicion media de contacto medida desde HOME.
  bool set_motor_calibration(double maxForceG,
                             uint32_t contactOffsetSteps,
                             uint32_t step500G,
                             uint32_t step1000G,
                             uint32_t step1500G);

  // ----------------------------------------------------------
  // Pedido runtime pendiente
  // ----------------------------------------------------------
  bool has_request() const;
  BeamHostRequest get_request() const;
  long get_requested_steps() const;
  uint8_t get_requested_cell() const;
  double get_requested_force_g() const;
  void clear_request();

  void factory_reset();

private:
  Preferences prefs;

  const char* app_version;
  bool nvs_ready;

  uint32_t step_period_us;
  uint32_t home_slow_period_us;
  uint32_t home_backoff_steps;
  uint32_t home_debounce_ms;
  uint32_t home_seek_max_steps;
  uint32_t move_fast_period_us;
  uint32_t max_position_steps;

  double cell1_a;
  double cell1_b;
  bool cell1_calibrated;

  double cell2_a;
  double cell2_b;
  bool cell2_calibrated;

  double motor_max_force_g;
  uint32_t motor_contact_offset_steps;
  uint32_t motor_step_500_g;
  uint32_t motor_step_1000_g;
  uint32_t motor_step_1500_g;
  bool motor_calibrated;

  bool load_stream_enabled;
  uint32_t load_stream_period_ms;

  BeamHostRequest pending_request;
  long requested_steps;
  uint8_t requested_cell;
  double requested_force_g;

  char rx_buffer[BEAM_HOST_RX_SIZE];
  size_t rx_len;
  int16_t rx_brace_depth;
  bool rx_in_string;
  bool rx_escape;
  bool rx_discarding;

  void load_defaults();
  void load_runtime_defaults();
  bool load_from_nvs();
  void save_all();

  void reset_rx();
  bool push_rx_char(char c);
  void process_rx_buffer();
  void process_json(JsonDocument& doc);

  void set_pending_request(BeamHostRequest req);
  void set_pending_move_abs_steps(long steps);
  void set_pending_cell_calibration(uint8_t cell);
  void set_pending_apply_force(double forceG);


  bool valid_period_us(uint32_t value) const;
  bool valid_backoff_steps(uint32_t value) const;
  bool valid_debounce_ms(uint32_t value) const;
  bool valid_home_seek_max_steps(uint32_t value) const;
  bool valid_max_position_steps(uint32_t value) const;
  bool valid_load_stream_period_ms(uint32_t value) const;
  bool valid_calibration(double a, double b) const;
  bool valid_motor_calibration(double maxForceG,
                               uint32_t contactOffsetSteps,
                               uint32_t step500G,
                               uint32_t step1000G,
                               uint32_t step1500G) const;
};

#endif // BEAM_CONFIG_H
