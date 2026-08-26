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
    - respuestas host JSON por Serial
    - pedidos runtime pendientes para que la FSM principal decida

  Regla:
    - NO mueve motor
    - NO lee FC_HOME para tomar decisiones
    - NO ejecuta homing
    - NO cambia estados operativos
    - NO hace Serial.begin()
*/

// ------------------------------------------------------------
// NVS
// ------------------------------------------------------------
#define BEAM_CFG_NAMESPACE       "beamcfg"
#define BEAM_CFG_MAGIC_NUMBER    220826UL
#define BEAM_CFG_SCHEMA_VERSION  2UL

// ------------------------------------------------------------
// Defaults
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

// Buffer fijo: sin asignacion dinamica y sin bloquear esperando Serial.
#define BEAM_HOST_RX_SIZE 256

enum BeamHostRequest : uint8_t {
  BEAM_REQ_NONE = 0,
  BEAM_REQ_HOME,
  BEAM_REQ_STOP,
  BEAM_REQ_ENABLE,
  BEAM_REQ_DISABLE,
  BEAM_REQ_MOVE_ABS_STEPS
};

class CBeamConfig {
public:
  CBeamConfig();

  bool init();
  void set_app_version(const char* version);

  // Procesa como maximo un JSON completo por llamada.
  // La recepcion es cooperativa: no espera por bytes que aun no llegaron.
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

  // ----------------------------------------------------------
  // Pedido runtime pendiente
  // ----------------------------------------------------------
  bool has_request() const;
  BeamHostRequest get_request() const;
  long get_requested_steps() const;
  void clear_request();

  // ----------------------------------------------------------
  // Respuestas/eventos: la FSM decide que informar.
  // La clase solamente serializa JSON.
  // ----------------------------------------------------------
  void send_command_result(const char* cmd,
                           const char* result,
                           const char* reason = nullptr);

  void send_event(const char* event,
                  const char* state = nullptr,
                  const char* detail = nullptr);

  void send_event_value(const char* event,
                        const char* state,
                        const char* key,
                        uint32_t value);

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

  BeamHostRequest pending_request;
  long requested_steps;

  // RX JSON cooperativo
  char rx_buffer[BEAM_HOST_RX_SIZE];
  size_t rx_len;
  int16_t rx_brace_depth;
  bool rx_in_string;
  bool rx_escape;
  bool rx_discarding;

  void load_defaults();
  bool load_from_nvs();
  void save_all();

  void reset_rx();
  bool push_rx_char(char c);
  void process_rx_buffer();
  void process_json(JsonDocument& doc);

  void set_pending_request(BeamHostRequest req);
  void set_pending_move_abs_steps(long steps);

  void send_all_params();
  void send_version();
  void send_ack(JsonDocument& doc);
  void send_ok(JsonDocument& doc);
  void send_error(const char* reason);
  void send_json(JsonDocument& doc);
  void send_json_pretty(JsonDocument& doc);

  bool valid_period_us(uint32_t value) const;
  bool valid_backoff_steps(uint32_t value) const;
  bool valid_debounce_ms(uint32_t value) const;
  bool valid_home_seek_max_steps(uint32_t value) const;
  bool valid_max_position_steps(uint32_t value) const;
};

#endif // BEAM_CONFIG_H
