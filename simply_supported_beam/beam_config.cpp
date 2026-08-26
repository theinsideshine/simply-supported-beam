#include "beam_config.h"
#include <string.h>

CBeamConfig::CBeamConfig()
{
  app_version = "unknown";
  nvs_ready = false;
  pending_request = BEAM_REQ_NONE;
  requested_steps = 0;

  load_defaults();
  reset_rx();
}

void CBeamConfig::set_app_version(const char* version)
{
  app_version = (version != nullptr) ? version : "unknown";
}

bool CBeamConfig::init()
{
  load_defaults();

  nvs_ready = prefs.begin(BEAM_CFG_NAMESPACE, false);
  if (!nvs_ready) {
    pending_request = BEAM_REQ_NONE;
    requested_steps = 0;
    reset_rx();
    return false;
  }

  uint32_t magic = prefs.getUInt("magic", 0);
  uint32_t schema = prefs.getUInt("schema", 0);

  if (magic != BEAM_CFG_MAGIC_NUMBER) {
    load_defaults();
    save_all();
  }
  else if (schema == 1UL || schema == BEAM_CFG_SCHEMA_VERSION) {
    // Migracion controlada desde 0.02 / schema 1:
    // conserva los parametros existentes y agrega los nuevos con default.
    load_from_nvs();

    if (schema != BEAM_CFG_SCHEMA_VERSION) {
      save_all();
    }
  }
  else {
    load_defaults();
    save_all();
  }

  pending_request = BEAM_REQ_NONE;
  requested_steps = 0;
  reset_rx();
  return true;
}

// ============================================================
// Defaults / NVS
// ============================================================

void CBeamConfig::load_defaults()
{
  step_period_us = BEAM_STEP_PERIOD_US_DEFAULT;
  home_slow_period_us = BEAM_HOME_SLOW_PERIOD_US_DEFAULT;
  home_backoff_steps = BEAM_HOME_BACKOFF_STEPS_DEFAULT;
  home_debounce_ms = BEAM_HOME_DEBOUNCE_MS_DEFAULT;
  home_seek_max_steps = BEAM_HOME_SEEK_MAX_STEPS_DEFAULT;
  move_fast_period_us = BEAM_MOVE_FAST_PERIOD_US_DEFAULT;
  max_position_steps = BEAM_MAX_POSITION_STEPS_DEFAULT;
}

bool CBeamConfig::load_from_nvs()
{
  if (!nvs_ready) {
    return false;
  }

  step_period_us = prefs.getUInt("step_us", BEAM_STEP_PERIOD_US_DEFAULT);
  home_slow_period_us = prefs.getUInt("slow_us", BEAM_HOME_SLOW_PERIOD_US_DEFAULT);
  home_backoff_steps = prefs.getUInt("backoff", BEAM_HOME_BACKOFF_STEPS_DEFAULT);
  home_debounce_ms = prefs.getUInt("debounce", BEAM_HOME_DEBOUNCE_MS_DEFAULT);

  // 0 se usa como sentinel para migrar NVS antiguas donde seek_max no existia.
  // Un limite de HOME valido nunca es 0.
  home_seek_max_steps = prefs.getUInt("seek_max", 0UL);

  // Nuevos en schema 2. Si no existen, se cargan defaults seguros.
  move_fast_period_us = prefs.getUInt("move_us", BEAM_MOVE_FAST_PERIOD_US_DEFAULT);
  max_position_steps = prefs.getUInt("pos_max", BEAM_MAX_POSITION_STEPS_DEFAULT);

  bool corrected = false;

  if (!valid_period_us(step_period_us)) {
    step_period_us = BEAM_STEP_PERIOD_US_DEFAULT;
    corrected = true;
  }

  if (!valid_period_us(home_slow_period_us)) {
    home_slow_period_us = BEAM_HOME_SLOW_PERIOD_US_DEFAULT;
    corrected = true;
  }

  if (!valid_backoff_steps(home_backoff_steps)) {
    home_backoff_steps = BEAM_HOME_BACKOFF_STEPS_DEFAULT;
    corrected = true;
  }

  if (!valid_debounce_ms(home_debounce_ms)) {
    home_debounce_ms = BEAM_HOME_DEBOUNCE_MS_DEFAULT;
    corrected = true;
  }

  if (!valid_home_seek_max_steps(home_seek_max_steps)) {
    home_seek_max_steps = BEAM_HOME_SEEK_MAX_STEPS_DEFAULT;
    corrected = true;
  }

  if (!valid_period_us(move_fast_period_us)) {
    move_fast_period_us = BEAM_MOVE_FAST_PERIOD_US_DEFAULT;
    corrected = true;
  }

  if (!valid_max_position_steps(max_position_steps)) {
    max_position_steps = BEAM_MAX_POSITION_STEPS_DEFAULT;
    corrected = true;
  }

  if (corrected) {
    save_all();
  }

  return true;
}

void CBeamConfig::save_all()
{
  if (!nvs_ready) {
    return;
  }

  prefs.putUInt("magic", BEAM_CFG_MAGIC_NUMBER);
  prefs.putUInt("schema", BEAM_CFG_SCHEMA_VERSION);
  prefs.putUInt("step_us", step_period_us);
  prefs.putUInt("slow_us", home_slow_period_us);
  prefs.putUInt("backoff", home_backoff_steps);
  prefs.putUInt("debounce", home_debounce_ms);
  prefs.putUInt("seek_max", home_seek_max_steps);
  prefs.putUInt("move_us", move_fast_period_us);
  prefs.putUInt("pos_max", max_position_steps);
}

// ============================================================
// Getters
// ============================================================

uint32_t CBeamConfig::get_step_period_us() const
{
  return step_period_us;
}

uint32_t CBeamConfig::get_home_slow_period_us() const
{
  return home_slow_period_us;
}

uint32_t CBeamConfig::get_home_backoff_steps() const
{
  return home_backoff_steps;
}

uint32_t CBeamConfig::get_home_debounce_ms() const
{
  return home_debounce_ms;
}

uint32_t CBeamConfig::get_home_seek_max_steps() const
{
  return home_seek_max_steps;
}

uint32_t CBeamConfig::get_move_fast_period_us() const
{
  return move_fast_period_us;
}

uint32_t CBeamConfig::get_max_position_steps() const
{
  return max_position_steps;
}

// ============================================================
// Setters RAM + NVS
// ============================================================

bool CBeamConfig::set_step_period_us(uint32_t value)
{
  if (!valid_period_us(value)) {
    return false;
  }

  step_period_us = value;
  if (nvs_ready) {
    prefs.putUInt("step_us", step_period_us);
  }
  return true;
}

bool CBeamConfig::set_home_slow_period_us(uint32_t value)
{
  if (!valid_period_us(value)) {
    return false;
  }

  home_slow_period_us = value;
  if (nvs_ready) {
    prefs.putUInt("slow_us", home_slow_period_us);
  }
  return true;
}

bool CBeamConfig::set_home_backoff_steps(uint32_t value)
{
  if (!valid_backoff_steps(value)) {
    return false;
  }

  home_backoff_steps = value;
  if (nvs_ready) {
    prefs.putUInt("backoff", home_backoff_steps);
  }
  return true;
}

bool CBeamConfig::set_home_debounce_ms(uint32_t value)
{
  if (!valid_debounce_ms(value)) {
    return false;
  }

  home_debounce_ms = value;
  if (nvs_ready) {
    prefs.putUInt("debounce", home_debounce_ms);
  }
  return true;
}

bool CBeamConfig::set_home_seek_max_steps(uint32_t value)
{
  if (!valid_home_seek_max_steps(value)) {
    return false;
  }

  home_seek_max_steps = value;
  if (nvs_ready) {
    prefs.putUInt("seek_max", home_seek_max_steps);
  }
  return true;
}

bool CBeamConfig::set_move_fast_period_us(uint32_t value)
{
  if (!valid_period_us(value)) {
    return false;
  }

  move_fast_period_us = value;
  if (nvs_ready) {
    prefs.putUInt("move_us", move_fast_period_us);
  }
  return true;
}

bool CBeamConfig::set_max_position_steps(uint32_t value)
{
  if (!valid_max_position_steps(value)) {
    return false;
  }

  max_position_steps = value;
  if (nvs_ready) {
    prefs.putUInt("pos_max", max_position_steps);
  }
  return true;
}

// ============================================================
// Requests runtime
// ============================================================

bool CBeamConfig::has_request() const
{
  return pending_request != BEAM_REQ_NONE;
}

BeamHostRequest CBeamConfig::get_request() const
{
  return pending_request;
}

long CBeamConfig::get_requested_steps() const
{
  return requested_steps;
}

void CBeamConfig::clear_request()
{
  pending_request = BEAM_REQ_NONE;
}

void CBeamConfig::set_pending_request(BeamHostRequest req)
{
  pending_request = req;
}

void CBeamConfig::set_pending_move_abs_steps(long steps)
{
  requested_steps = steps;
  pending_request = BEAM_REQ_MOVE_ABS_STEPS;
}

// ============================================================
// Host JSON cooperativo
// ============================================================

void CBeamConfig::reset_rx()
{
  rx_len = 0;
  rx_brace_depth = 0;
  rx_in_string = false;
  rx_escape = false;
  rx_discarding = false;
  rx_buffer[0] = '\0';
}

bool CBeamConfig::push_rx_char(char c)
{
  if (rx_len >= (BEAM_HOST_RX_SIZE - 1)) {
    rx_discarding = true;
    rx_len = 0;
    rx_brace_depth = 0;
    rx_in_string = false;
    rx_escape = false;
    send_error("json_too_long");
    return false;
  }

  rx_buffer[rx_len++] = c;
  rx_buffer[rx_len] = '\0';
  return true;
}

void CBeamConfig::host_cmd()
{
  if (Serial.available() <= 0) {
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (rx_discarding) {
      if (c == '\r' || c == '\n') {
        reset_rx();
      }
      continue;
    }

    if (rx_len == 0) {
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        continue;
      }

      if (c != '{') {
        rx_discarding = true;
        send_error("json_expected");
        continue;
      }
    }

    if (!push_rx_char(c)) {
      continue;
    }

    if (rx_in_string) {
      if (rx_escape) {
        rx_escape = false;
      }
      else if (c == '\\') {
        rx_escape = true;
      }
      else if (c == '"') {
        rx_in_string = false;
      }
    }
    else {
      if (c == '"') {
        rx_in_string = true;
      }
      else if (c == '{') {
        rx_brace_depth++;
      }
      else if (c == '}') {
        rx_brace_depth--;
      }
    }

    if (!rx_in_string && rx_brace_depth == 0 && rx_len > 0) {
      process_rx_buffer();
      reset_rx();

      // Un request por pasada para que la FSM tenga oportunidad de consumirlo
      // antes de recibir otro request runtime.
      return;
    }

    if (rx_brace_depth < 0) {
      send_error("json_parse_error");
      reset_rx();
      return;
    }
  }
}

void CBeamConfig::process_rx_buffer()
{
  StaticJsonDocument<384> doc;
  DeserializationError error = deserializeJson(doc, rx_buffer);

  if (error) {
    send_error("json_parse_error");
    return;
  }

  process_json(doc);
}

void CBeamConfig::process_json(JsonDocument& doc)
{
  if (!doc.is<JsonObject>()) {
    send_error("json_object_expected");
    return;
  }

  bool known_key = false;
  bool invalid_value = false;

  // ----------------------------------------------------------
  // Config persistente
  // ----------------------------------------------------------
  if (doc.containsKey("step_period_us")) {
    uint32_t value = doc["step_period_us"].as<uint32_t>();
    if (set_step_period_us(value)) {
      doc["step_period_us"] = step_period_us;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("home_slow_period_us")) {
    uint32_t value = doc["home_slow_period_us"].as<uint32_t>();
    if (set_home_slow_period_us(value)) {
      doc["home_slow_period_us"] = home_slow_period_us;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("home_backoff_steps")) {
    uint32_t value = doc["home_backoff_steps"].as<uint32_t>();
    if (set_home_backoff_steps(value)) {
      doc["home_backoff_steps"] = home_backoff_steps;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("home_debounce_ms")) {
    uint32_t value = doc["home_debounce_ms"].as<uint32_t>();
    if (set_home_debounce_ms(value)) {
      doc["home_debounce_ms"] = home_debounce_ms;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("home_seek_max_steps")) {
    uint32_t value = doc["home_seek_max_steps"].as<uint32_t>();
    if (set_home_seek_max_steps(value)) {
      doc["home_seek_max_steps"] = home_seek_max_steps;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("move_fast_period_us")) {
    uint32_t value = doc["move_fast_period_us"].as<uint32_t>();
    if (set_move_fast_period_us(value)) {
      doc["move_fast_period_us"] = move_fast_period_us;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (doc.containsKey("max_position_steps")) {
    uint32_t value = doc["max_position_steps"].as<uint32_t>();
    if (set_max_position_steps(value)) {
      doc["max_position_steps"] = max_position_steps;
      known_key = true;
    }
    else {
      invalid_value = true;
    }
  }

  if (invalid_value) {
    send_error("invalid_value");
    return;
  }

  // ----------------------------------------------------------
  // Info
  // ----------------------------------------------------------
  if (doc.containsKey("info")) {
    const char* key = doc["info"] | "";

    if (strcmp(key, "version") == 0) {
      send_version();
      return;
    }

    if (strcmp(key, "all_params") == 0 || strcmp(key, "all-params") == 0) {
      send_all_params();
      return;
    }

    send_error("unknown_info");
    return;
  }

  // ----------------------------------------------------------
  // Runtime: solo deja pending_request.
  // La FSM decide si el movimiento es fisicamente admisible.
  // ----------------------------------------------------------
  if (doc.containsKey("cmd")) {
    const char* key = doc["cmd"] | "";

    if (strcmp(key, "home") == 0) {
      set_pending_request(BEAM_REQ_HOME);
      send_ack(doc);
      return;
    }

    if (strcmp(key, "stop") == 0) {
      set_pending_request(BEAM_REQ_STOP);
      send_ack(doc);
      return;
    }

    if (strcmp(key, "enable") == 0) {
      set_pending_request(BEAM_REQ_ENABLE);
      send_ack(doc);
      return;
    }

    if (strcmp(key, "disable") == 0) {
      set_pending_request(BEAM_REQ_DISABLE);
      send_ack(doc);
      return;
    }

    if (strcmp(key, "move_abs_steps") == 0) {
      if (!doc.containsKey("steps") || !doc["steps"].is<long>()) {
        send_error("invalid_steps");
        return;
      }

      set_pending_move_abs_steps(doc["steps"].as<long>());
      send_ack(doc);
      return;
    }

    if (strcmp(key, "factory_reset") == 0 || strcmp(key, "factory-reset") == 0) {
      factory_reset();
      send_ok(doc);
      return;
    }

    send_error("unknown_cmd");
    return;
  }

  if (known_key) {
    send_ok(doc);
    return;
  }

  send_error("unknown_key");
}

// ============================================================
// JSON responses
// ============================================================

void CBeamConfig::send_all_params()
{
  StaticJsonDocument<512> doc;

  doc["info"] = "all_params";
  doc["result"] = "ok";
  doc["app_version"] = app_version;
  doc["magic"] = BEAM_CFG_MAGIC_NUMBER;
  doc["schema"] = BEAM_CFG_SCHEMA_VERSION;
  doc["step_period_us"] = step_period_us;
  doc["home_slow_period_us"] = home_slow_period_us;
  doc["home_backoff_steps"] = home_backoff_steps;
  doc["home_debounce_ms"] = home_debounce_ms;
  doc["home_seek_max_steps"] = home_seek_max_steps;
  doc["move_fast_period_us"] = move_fast_period_us;
  doc["max_position_steps"] = max_position_steps;
  doc["pending_request"] = (uint8_t)pending_request;
  doc["requested_steps"] = requested_steps;

  // Mismo criterio visual del proyecto de la puerta.
  send_json_pretty(doc);
}

void CBeamConfig::send_version()
{
  StaticJsonDocument<192> doc;
  doc["info"] = "version";
  doc["result"] = "ok";
  doc["app_version"] = app_version;
  send_json(doc);
}

void CBeamConfig::send_ack(JsonDocument& doc)
{
  doc["result"] = "ack";
  send_json(doc);
}

void CBeamConfig::send_ok(JsonDocument& doc)
{
  doc["result"] = "ok";
  send_json(doc);
}

void CBeamConfig::send_error(const char* reason)
{
  StaticJsonDocument<192> doc;
  doc["result"] = "error";
  doc["reason"] = reason;
  send_json(doc);
}

void CBeamConfig::send_command_result(const char* cmd,
                                      const char* result,
                                      const char* reason)
{
  StaticJsonDocument<224> doc;
  doc["cmd"] = cmd;
  doc["result"] = result;

  if (reason != nullptr && reason[0] != '\0') {
    doc["reason"] = reason;
  }

  send_json(doc);
}

void CBeamConfig::send_event(const char* event,
                             const char* state,
                             const char* detail)
{
  StaticJsonDocument<256> doc;
  doc["event"] = event;

  if (state != nullptr && state[0] != '\0') {
    doc["state"] = state;
  }

  if (detail != nullptr && detail[0] != '\0') {
    doc["detail"] = detail;
  }

  send_json(doc);
}

void CBeamConfig::send_event_value(const char* event,
                                   const char* state,
                                   const char* key,
                                   uint32_t value)
{
  StaticJsonDocument<256> doc;
  doc["event"] = event;

  if (state != nullptr && state[0] != '\0') {
    doc["state"] = state;
  }

  if (key != nullptr && key[0] != '\0') {
    doc[key] = value;
  }

  send_json(doc);
}

void CBeamConfig::send_json(JsonDocument& doc)
{
  serializeJson(doc, Serial);
  Serial.println();
  Serial.println();
}

void CBeamConfig::send_json_pretty(JsonDocument& doc)
{
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println();
}

void CBeamConfig::factory_reset()
{
  load_defaults();
  save_all();
  pending_request = BEAM_REQ_NONE;
  requested_steps = 0;
}

// ============================================================
// Validacion minima
// ============================================================

bool CBeamConfig::valid_period_us(uint32_t value) const
{
  return value >= 100UL && value <= 1000000UL;
}

bool CBeamConfig::valid_backoff_steps(uint32_t value) const
{
  return value <= 100000UL;
}

bool CBeamConfig::valid_debounce_ms(uint32_t value) const
{
  return value <= 10000UL;
}

bool CBeamConfig::valid_home_seek_max_steps(uint32_t value) const
{
  return value >= 1UL && value <= 1000000UL;
}

bool CBeamConfig::valid_max_position_steps(uint32_t value) const
{
  // 0 es valido y significa "todavia no configurado".
  return value <= 1000000UL;
}
