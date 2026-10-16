#include "beam_config.h"
#include "beam_messaging.h"
#include <string.h>
#include <math.h>

CBeamConfig::CBeamConfig()
{
  app_version = "unknown";
  nvs_ready = false;
  pending_request = BEAM_REQ_NONE;
  requested_steps = 0;
  requested_cell = 0;
  requested_force_g = 0.0;

  load_defaults();
  load_runtime_defaults();
  reset_rx();
}

void CBeamConfig::set_app_version(const char* version)
{
  app_version = (version != nullptr) ? version : "unknown";
}

bool CBeamConfig::init()
{
  load_defaults();
  load_runtime_defaults();

  nvs_ready = prefs.begin(BEAM_CFG_NAMESPACE, false);
  if (!nvs_ready) {
    pending_request = BEAM_REQ_NONE;
    requested_steps = 0;
    requested_cell = 0;
    requested_force_g = 0.0;
    reset_rx();
    return false;
  }

  uint32_t magic = prefs.getUInt("magic", 0);
  uint32_t schema = prefs.getUInt("schema", 0);

  if (magic != BEAM_CFG_MAGIC_NUMBER) {
    load_defaults();
    save_all();
  }
  else if (schema == 1UL || schema == 2UL || schema == 3UL || schema == BEAM_CFG_SCHEMA_VERSION) {
    // Migracion controlada:
    // schema 1 -> parametros de HOME originales
    // schema 2 -> agrega movimiento absoluto
    // schema 3 -> agrega calibracion de ambas celdas
    // schema 4 -> agrega calibracion motor-viga y offset desde HOME
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
  requested_cell = 0;
  requested_force_g = 0.0;
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

  cell1_a = BEAM_CELL_CAL_A_DEFAULT;
  cell1_b = BEAM_CELL_CAL_B_DEFAULT;
  cell1_calibrated = false;

  cell2_a = BEAM_CELL_CAL_A_DEFAULT;
  cell2_b = BEAM_CELL_CAL_B_DEFAULT;
  cell2_calibrated = false;

  motor_max_force_g = BEAM_MOTOR_MAX_FORCE_G_DEFAULT;
  motor_contact_offset_steps = BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT;
  motor_step_500_g = BEAM_MOTOR_STEP_500_G_DEFAULT;
  motor_step_1000_g = BEAM_MOTOR_STEP_1000_G_DEFAULT;
  motor_step_1500_g = BEAM_MOTOR_STEP_1500_G_DEFAULT;
  motor_calibrated = false;
}

void CBeamConfig::load_runtime_defaults()
{
  load_stream_enabled = BEAM_LOAD_STREAM_DEFAULT;
  load_stream_period_ms = BEAM_LOAD_STREAM_PERIOD_MS_DEFAULT;
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

  move_fast_period_us = prefs.getUInt("move_us", BEAM_MOVE_FAST_PERIOD_US_DEFAULT);
  max_position_steps = prefs.getUInt("pos_max", BEAM_MAX_POSITION_STEPS_DEFAULT);

  // Schema 3. En NVS antiguas estas claves no existen y se toman defaults.
  cell1_a = prefs.getDouble("c1_a", BEAM_CELL_CAL_A_DEFAULT);
  cell1_b = prefs.getDouble("c1_b", BEAM_CELL_CAL_B_DEFAULT);
  cell1_calibrated = prefs.getBool("c1_ok", false);

  cell2_a = prefs.getDouble("c2_a", BEAM_CELL_CAL_A_DEFAULT);
  cell2_b = prefs.getDouble("c2_b", BEAM_CELL_CAL_B_DEFAULT);
  cell2_calibrated = prefs.getBool("c2_ok", false);

  // Schema 4. Claves cortas por limite de longitud de Preferences/NVS.
  motor_max_force_g = prefs.getDouble("m_max_g", BEAM_MOTOR_MAX_FORCE_G_DEFAULT);
  motor_contact_offset_steps = prefs.getUInt("m_contact", BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT);
  motor_step_500_g = prefs.getUInt("m_500", BEAM_MOTOR_STEP_500_G_DEFAULT);
  motor_step_1000_g = prefs.getUInt("m_1000", BEAM_MOTOR_STEP_1000_G_DEFAULT);
  motor_step_1500_g = prefs.getUInt("m_1500", BEAM_MOTOR_STEP_1500_G_DEFAULT);
  motor_calibrated = prefs.getBool("m_ok", false);

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

  if (cell1_calibrated && !valid_calibration(cell1_a, cell1_b)) {
    cell1_a = BEAM_CELL_CAL_A_DEFAULT;
    cell1_b = BEAM_CELL_CAL_B_DEFAULT;
    cell1_calibrated = false;
    corrected = true;
  }

  if (cell2_calibrated && !valid_calibration(cell2_a, cell2_b)) {
    cell2_a = BEAM_CELL_CAL_A_DEFAULT;
    cell2_b = BEAM_CELL_CAL_B_DEFAULT;
    cell2_calibrated = false;
    corrected = true;
  }

  if (motor_calibrated &&
      !valid_motor_calibration(motor_max_force_g,
                               motor_contact_offset_steps,
                               motor_step_500_g,
                               motor_step_1000_g,
                               motor_step_1500_g)) {
    motor_max_force_g = BEAM_MOTOR_MAX_FORCE_G_DEFAULT;
    motor_contact_offset_steps = BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT;
    motor_step_500_g = BEAM_MOTOR_STEP_500_G_DEFAULT;
    motor_step_1000_g = BEAM_MOTOR_STEP_1000_G_DEFAULT;
    motor_step_1500_g = BEAM_MOTOR_STEP_1500_G_DEFAULT;
    motor_calibrated = false;
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

  prefs.putDouble("c1_a", cell1_a);
  prefs.putDouble("c1_b", cell1_b);
  prefs.putBool("c1_ok", cell1_calibrated);

  prefs.putDouble("c2_a", cell2_a);
  prefs.putDouble("c2_b", cell2_b);
  prefs.putBool("c2_ok", cell2_calibrated);

  prefs.putDouble("m_max_g", motor_max_force_g);
  prefs.putUInt("m_contact", motor_contact_offset_steps);
  prefs.putUInt("m_500", motor_step_500_g);
  prefs.putUInt("m_1000", motor_step_1000_g);
  prefs.putUInt("m_1500", motor_step_1500_g);
  prefs.putBool("m_ok", motor_calibrated);
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

double CBeamConfig::get_cell1_a() const
{
  return cell1_a;
}

double CBeamConfig::get_cell1_b() const
{
  return cell1_b;
}

bool CBeamConfig::is_cell1_calibrated() const
{
  return cell1_calibrated;
}

double CBeamConfig::get_cell2_a() const
{
  return cell2_a;
}

double CBeamConfig::get_cell2_b() const
{
  return cell2_b;
}

bool CBeamConfig::is_cell2_calibrated() const
{
  return cell2_calibrated;
}

double CBeamConfig::get_motor_max_force_g() const
{
  return motor_max_force_g;
}

uint32_t CBeamConfig::get_motor_contact_offset_steps() const
{
  return motor_contact_offset_steps;
}

uint32_t CBeamConfig::get_motor_step_500_g() const
{
  return motor_step_500_g;
}

uint32_t CBeamConfig::get_motor_step_1000_g() const
{
  return motor_step_1000_g;
}

uint32_t CBeamConfig::get_motor_step_1500_g() const
{
  return motor_step_1500_g;
}

bool CBeamConfig::is_motor_calibrated() const
{
  return motor_calibrated;
}

bool CBeamConfig::get_load_stream_enabled() const
{
  return load_stream_enabled;
}

uint32_t CBeamConfig::get_load_stream_period_ms() const
{
  return load_stream_period_ms;
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

  // Si el nuevo limite deja fuera una calibracion motor-viga existente,
  // esa calibracion deja de ser confiable.
  if (motor_calibrated &&
      (max_position_steps == 0UL || motor_step_1500_g > max_position_steps)) {
    motor_max_force_g = BEAM_MOTOR_MAX_FORCE_G_DEFAULT;
    motor_contact_offset_steps = BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT;
    motor_step_500_g = BEAM_MOTOR_STEP_500_G_DEFAULT;
    motor_step_1000_g = BEAM_MOTOR_STEP_1000_G_DEFAULT;
    motor_step_1500_g = BEAM_MOTOR_STEP_1500_G_DEFAULT;
    motor_calibrated = false;
  }

  if (nvs_ready) {
    prefs.putUInt("pos_max", max_position_steps);
    prefs.putDouble("m_max_g", motor_max_force_g);
    prefs.putUInt("m_contact", motor_contact_offset_steps);
    prefs.putUInt("m_500", motor_step_500_g);
    prefs.putUInt("m_1000", motor_step_1000_g);
    prefs.putUInt("m_1500", motor_step_1500_g);
    prefs.putBool("m_ok", motor_calibrated);
  }
  return true;
}

bool CBeamConfig::set_cell1_calibration(double a, double b)
{
  if (!valid_calibration(a, b)) {
    return false;
  }

  cell1_a = a;
  cell1_b = b;
  cell1_calibrated = true;

  // Si cambia la calibracion de fuerza, la caracterizacion motor-viga
  // debe repetirse antes de usar sus offsets en un experimento.
  motor_max_force_g = BEAM_MOTOR_MAX_FORCE_G_DEFAULT;
  motor_contact_offset_steps = BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT;
  motor_step_500_g = BEAM_MOTOR_STEP_500_G_DEFAULT;
  motor_step_1000_g = BEAM_MOTOR_STEP_1000_G_DEFAULT;
  motor_step_1500_g = BEAM_MOTOR_STEP_1500_G_DEFAULT;
  motor_calibrated = false;

  if (nvs_ready) {
    prefs.putDouble("c1_a", cell1_a);
    prefs.putDouble("c1_b", cell1_b);
    prefs.putBool("c1_ok", true);
    prefs.putDouble("m_max_g", motor_max_force_g);
    prefs.putUInt("m_contact", motor_contact_offset_steps);
    prefs.putUInt("m_500", motor_step_500_g);
    prefs.putUInt("m_1000", motor_step_1000_g);
    prefs.putUInt("m_1500", motor_step_1500_g);
    prefs.putBool("m_ok", false);
  }

  return true;
}

bool CBeamConfig::set_cell2_calibration(double a, double b)
{
  if (!valid_calibration(a, b)) {
    return false;
  }

  cell2_a = a;
  cell2_b = b;
  cell2_calibrated = true;

  // Si cambia la calibracion de fuerza, la caracterizacion motor-viga
  // debe repetirse antes de usar sus offsets en un experimento.
  motor_max_force_g = BEAM_MOTOR_MAX_FORCE_G_DEFAULT;
  motor_contact_offset_steps = BEAM_MOTOR_CONTACT_OFFSET_STEPS_DEFAULT;
  motor_step_500_g = BEAM_MOTOR_STEP_500_G_DEFAULT;
  motor_step_1000_g = BEAM_MOTOR_STEP_1000_G_DEFAULT;
  motor_step_1500_g = BEAM_MOTOR_STEP_1500_G_DEFAULT;
  motor_calibrated = false;

  if (nvs_ready) {
    prefs.putDouble("c2_a", cell2_a);
    prefs.putDouble("c2_b", cell2_b);
    prefs.putBool("c2_ok", true);
    prefs.putDouble("m_max_g", motor_max_force_g);
    prefs.putUInt("m_contact", motor_contact_offset_steps);
    prefs.putUInt("m_500", motor_step_500_g);
    prefs.putUInt("m_1000", motor_step_1000_g);
    prefs.putUInt("m_1500", motor_step_1500_g);
    prefs.putBool("m_ok", false);
  }

  return true;
}

bool CBeamConfig::set_motor_calibration(double maxForceG,
                                         uint32_t contactOffsetSteps,
                                         uint32_t step500G,
                                         uint32_t step1000G,
                                         uint32_t step1500G)
{
  if (!valid_motor_calibration(maxForceG,
                               contactOffsetSteps,
                               step500G,
                               step1000G,
                               step1500G)) {
    return false;
  }

  motor_max_force_g = maxForceG;
  motor_contact_offset_steps = contactOffsetSteps;
  motor_step_500_g = step500G;
  motor_step_1000_g = step1000G;
  motor_step_1500_g = step1500G;
  motor_calibrated = true;

  if (nvs_ready) {
    prefs.putDouble("m_max_g", motor_max_force_g);
    prefs.putUInt("m_contact", motor_contact_offset_steps);
    prefs.putUInt("m_500", motor_step_500_g);
    prefs.putUInt("m_1000", motor_step_1000_g);
    prefs.putUInt("m_1500", motor_step_1500_g);
    prefs.putBool("m_ok", true);
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

uint8_t CBeamConfig::get_requested_cell() const
{
  return requested_cell;
}

double CBeamConfig::get_requested_force_g() const
{
  return requested_force_g;
}

void CBeamConfig::clear_request()
{
  pending_request = BEAM_REQ_NONE;
  requested_cell = 0;
  requested_force_g = 0.0;
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

void CBeamConfig::set_pending_cell_calibration(uint8_t cell)
{
  requested_cell = cell;
  pending_request = BEAM_REQ_MODE_CALIBRATION_CELL;
}

void CBeamConfig::set_pending_apply_force(double forceG)
{
  requested_force_g = forceG;
  pending_request = BEAM_REQ_APPLY_FORCE;
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
    Messaging.send_error("json_too_long");
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
        Messaging.send_error("json_expected");
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
      Messaging.send_error("json_parse_error");
      reset_rx();
      return;
    }
  }
}

void CBeamConfig::process_rx_buffer()
{
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, rx_buffer);

  if (error) {
    Messaging.send_error("json_parse_error");
    return;
  }

  process_json(doc);
}

void CBeamConfig::process_json(JsonDocument& doc)
{
  if (!doc.is<JsonObject>()) {
    Messaging.send_error("json_object_expected");
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

  // ----------------------------------------------------------
  // Parametros runtime NO persistentes de visualizacion
  // ----------------------------------------------------------
  if (doc.containsKey("load_stream")) {
    if (!doc["load_stream"].is<bool>()) {
      invalid_value = true;
    }
    else {
      load_stream_enabled = doc["load_stream"].as<bool>();
      doc["load_stream"] = load_stream_enabled;
      known_key = true;
    }
  }

  if (doc.containsKey("load_stream_period_ms")) {
    uint32_t value = doc["load_stream_period_ms"].as<uint32_t>();

    if (!valid_load_stream_period_ms(value)) {
      invalid_value = true;
    }
    else {
      load_stream_period_ms = value;
      doc["load_stream_period_ms"] = load_stream_period_ms;
      known_key = true;
    }
  }

  if (invalid_value) {
    Messaging.send_error("invalid_value");
    return;
  }

  // ----------------------------------------------------------
  // Info
  // ----------------------------------------------------------
  if (doc.containsKey("info")) {
    const char* key = doc["info"] | "";

    if (strcmp(key, "version") == 0) {
      Messaging.send_version(app_version);
      return;
    }

    if (strcmp(key, "all_params") == 0 || strcmp(key, "all-params") == 0) {
      Messaging.send_all_params(*this, app_version);
      return;
    }

    Messaging.send_error("unknown_info");
    return;
  }

  // ----------------------------------------------------------
  // Runtime: solo deja pending_request.
  // El .ino / FSM decide si la operacion es fisicamente admisible.
  // ----------------------------------------------------------
  if (doc.containsKey("cmd")) {
    const char* key = doc["cmd"] | "";

    if (strcmp(key, "home") == 0) {
      set_pending_request(BEAM_REQ_HOME);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "stop") == 0) {
      set_pending_request(BEAM_REQ_STOP);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "enable") == 0) {
      set_pending_request(BEAM_REQ_ENABLE);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "disable") == 0) {
      set_pending_request(BEAM_REQ_DISABLE);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "move_abs_steps") == 0) {
      if (!doc.containsKey("steps") || !doc["steps"].is<long>()) {
        Messaging.send_error("invalid_steps");
        return;
      }

      set_pending_move_abs_steps(doc["steps"].as<long>());
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "read_loads") == 0) {
      set_pending_request(BEAM_REQ_READ_LOADS);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "apply_force") == 0) {
      if (!doc.containsKey("force_g")) {
        Messaging.send_error("invalid_force_g");
        return;
      }

      double forceG = doc["force_g"].as<double>();
      if (!isfinite(forceG) || forceG <= 0.0) {
        Messaging.send_error("invalid_force_g");
        return;
      }

      set_pending_apply_force(forceG);
      Messaging.send_ack(doc);
      return;
    }

    if (strcmp(key, "set_mode") == 0) {
      const char* mode = doc["mode"] | "";

      if (strcmp(mode, "manual") == 0) {
        set_pending_request(BEAM_REQ_MODE_MANUAL);
        Messaging.send_ack(doc);
        return;
      }

      if (strcmp(mode, "calibration") == 0) {
        const char* calibration = doc["calibration"] | "";

        if (strcmp(calibration, "motor") == 0) {
          set_pending_request(BEAM_REQ_MODE_CALIBRATION_MOTOR);
          Messaging.send_ack(doc);
          return;
        }

        // Compatibilidad con la mecanica ya validada de calibracion
        // de celdas: el JSON selecciona la celda y luego el usuario
        // continua con confirmaciones de texto "ok".
        if (!doc.containsKey("cell") || !doc["cell"].is<int>()) {
          Messaging.send_error("invalid_calibration");
          return;
        }

        int cell = doc["cell"].as<int>();
        if (cell != 1 && cell != 2) {
          Messaging.send_error("invalid_cell");
          return;
        }

        set_pending_cell_calibration((uint8_t)cell);
        Messaging.send_ack(doc);
        return;
      }

      if (strcmp(mode, "experiment") == 0) {
        set_pending_request(BEAM_REQ_MODE_EXPERIMENT);
        Messaging.send_ack(doc);
        return;
      }

      Messaging.send_error("unknown_mode");
      return;
    }

    if (strcmp(key, "factory_reset") == 0 || strcmp(key, "factory-reset") == 0) {
      set_pending_request(BEAM_REQ_FACTORY_RESET);
      Messaging.send_ack(doc);
      return;
    }

    Messaging.send_error("unknown_cmd");
    return;
  }

  if (known_key) {
    Messaging.send_ok(doc);
    return;
  }

  Messaging.send_error("unknown_key");
}

// ============================================================
// JSON responses
// ============================================================

// Mensajeria JSON delegada a CBeamMessaging.

void CBeamConfig::factory_reset()
{
  load_defaults();
  load_runtime_defaults();
  save_all();
  pending_request = BEAM_REQ_NONE;
  requested_steps = 0;
  requested_cell = 0;
  requested_force_g = 0.0;
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

bool CBeamConfig::valid_load_stream_period_ms(uint32_t value) const
{
  return value >= 100UL && value <= 60000UL;
}

bool CBeamConfig::valid_calibration(double a, double b) const
{
  return isfinite(a) && isfinite(b) && fabs(a) > 1.0e-12;
}


bool CBeamConfig::valid_motor_calibration(double maxForceG,
                                           uint32_t contactOffsetSteps,
                                           uint32_t step500G,
                                           uint32_t step1000G,
                                           uint32_t step1500G) const
{
  if (!isfinite(maxForceG) || maxForceG <= 1500.0) {
    return false;
  }

  if (contactOffsetSteps == 0UL) {
    return false;
  }

  if (!(contactOffsetSteps <= step500G &&
        step500G <= step1000G &&
        step1000G <= step1500G)) {
    return false;
  }

  if (max_position_steps > 0UL && step1500G > max_position_steps) {
    return false;
  }

  return true;
}
