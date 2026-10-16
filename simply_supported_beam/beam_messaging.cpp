#include "beam_messaging.h"
#include "beam_config.h"
#include <math.h>

CBeamMessaging Messaging;

void CBeamMessaging::send_all_params(const CBeamConfig& config,
                                     const char* appVersion)
{
  StaticJsonDocument<1536> doc;

  doc["info"] = "all_params";
  doc["result"] = "ok";
  doc["app_version"] = (appVersion != nullptr) ? appVersion : "unknown";
  doc["magic"] = BEAM_CFG_MAGIC_NUMBER;
  doc["schema"] = BEAM_CFG_SCHEMA_VERSION;
  doc["step_period_us"] = config.get_step_period_us();
  doc["home_slow_period_us"] = config.get_home_slow_period_us();
  doc["home_backoff_steps"] = config.get_home_backoff_steps();
  doc["home_debounce_ms"] = config.get_home_debounce_ms();
  doc["home_seek_max_steps"] = config.get_home_seek_max_steps();
  doc["move_fast_period_us"] = config.get_move_fast_period_us();
  doc["max_position_steps"] = config.get_max_position_steps();

  doc["cell1_a"] = config.get_cell1_a();
  doc["cell1_b"] = config.get_cell1_b();
  doc["cell1_calibrated"] = config.is_cell1_calibrated();

  doc["cell2_a"] = config.get_cell2_a();
  doc["cell2_b"] = config.get_cell2_b();
  doc["cell2_calibrated"] = config.is_cell2_calibrated();

  doc["motor_max_force_g"] = config.get_motor_max_force_g();
  doc["motor_contact_offset_steps"] = config.get_motor_contact_offset_steps();
  doc["motor_step_500_g"] = config.get_motor_step_500_g();
  doc["motor_step_1000_g"] = config.get_motor_step_1000_g();
  doc["motor_step_1500_g"] = config.get_motor_step_1500_g();
  doc["motor_calibrated"] = config.is_motor_calibrated();

  doc["load_stream"] = config.get_load_stream_enabled();
  doc["load_stream_period_ms"] = config.get_load_stream_period_ms();

  doc["pending_request"] = (uint8_t)config.get_request();
  doc["requested_steps"] = config.get_requested_steps();
  doc["requested_cell"] = config.get_requested_cell();
  doc["requested_force_g"] = config.get_requested_force_g();

  send_json_pretty(doc);
}

void CBeamMessaging::send_version(const char* appVersion)
{
  StaticJsonDocument<192> doc;
  doc["info"] = "version";
  doc["result"] = "ok";
  doc["app_version"] = (appVersion != nullptr) ? appVersion : "unknown";
  send_json(doc);
}

void CBeamMessaging::send_ack(JsonDocument& doc)
{
  doc["result"] = "ack";
  send_json(doc);
}

void CBeamMessaging::send_ok(JsonDocument& doc)
{
  doc["result"] = "ok";
  send_json(doc);
}

void CBeamMessaging::send_error(const char* reason)
{
  StaticJsonDocument<192> doc;
  doc["result"] = "error";
  doc["reason"] = reason;
  send_json(doc);
}

void CBeamMessaging::send_command_result(const char* cmd,
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

void CBeamMessaging::send_event(const char* event,
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

void CBeamMessaging::send_event_value(const char* event,
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

void CBeamMessaging::send_loads(const char* source,
                                double r1_g,
                                double r2_g,
                                double sum_g,
                                double r1_n,
                                double r2_n,
                                double sum_n)
{
  StaticJsonDocument<384> doc;

  doc["event"] = "loads";

  if (source != nullptr && source[0] != '\0') {
    doc["source"] = source;
  }

  doc["r1_g"] = r1_g;
  doc["r2_g"] = r2_g;
  doc["sum_g"] = sum_g;
  doc["r1_n"] = r1_n;
  doc["r2_n"] = r2_n;
  doc["sum_n"] = sum_n;

  send_json(doc);
}

void CBeamMessaging::send_motor_calibration_cycle(uint8_t cycle,
                                                   double baselineG,
                                                   uint32_t contactStep,
                                                   uint32_t step500G,
                                                   double force500G,
                                                   uint32_t step1000G,
                                                   double force1000G,
                                                   uint32_t step1500G,
                                                   double force1500G)
{
  StaticJsonDocument<512> doc;

  doc["event"] = "motor_calibration_cycle";
  doc["cycle"] = cycle;
  doc["baseline_g"] = baselineG;
  doc["contact_step"] = contactStep;
  doc["step_500_g"] = step500G;
  doc["force_500_g"] = force500G;
  doc["step_1000_g"] = step1000G;
  doc["force_1000_g"] = force1000G;
  doc["step_1500_g"] = step1500G;
  doc["force_1500_g"] = force1500G;

  send_json(doc);
}

void CBeamMessaging::send_motor_calibration_plan(double gapMm,
                                                  uint32_t estimatedContactSteps,
                                                  uint32_t approachTargetSteps,
                                                  double maxForceG)
{
  StaticJsonDocument<384> doc;

  doc["event"] = "motor_calibration";
  doc["state"] = "plan";
  doc["gap_mm"] = gapMm;
  doc["estimated_contact_steps"] = estimatedContactSteps;
  doc["approach_target_steps"] = approachTargetSteps;
  doc["max_force_g"] = maxForceG;

  send_json(doc);
}

void CBeamMessaging::send_motor_calibration_progress(const char* state,
                                                      uint8_t cycle,
                                                      uint32_t referenceContactStep,
                                                      uint32_t approachTargetStep,
                                                      uint32_t positionSteps,
                                                      double forceG,
                                                      double targetForceG)
{
  StaticJsonDocument<448> doc;

  doc["event"] = "motor_calibration";
  doc["state"] = state;

  if (cycle > 0) {
    doc["cycle"] = cycle;
  }

  if (referenceContactStep > 0UL) {
    doc["reference_contact_step"] = referenceContactStep;
  }

  if (approachTargetStep > 0UL) {
    doc["approach_target_step"] = approachTargetStep;
  }

  doc["position_steps"] = positionSteps;

  if (isfinite(forceG)) {
    doc["force_g"] = forceG;
  }

  if (targetForceG > 0.0) {
    doc["target_g"] = targetForceG;
  }

  send_json(doc);
}

void CBeamMessaging::send_force_control_event(const char* state,
                                               double targetForceG,
                                               double baselineG,
                                               uint32_t estimatedStep,
                                               uint32_t positionSteps,
                                               double forceG)
{
  StaticJsonDocument<448> doc;

  doc["event"] = "force_control";
  doc["state"] = state;
  doc["target_g"] = targetForceG;
  doc["baseline_g"] = baselineG;

  if (estimatedStep > 0UL) {
    doc["estimated_step"] = estimatedStep;
  }

  doc["position_steps"] = positionSteps;

  if (isfinite(forceG)) {
    doc["force_g"] = forceG;
  }

  send_json(doc);
}

void CBeamMessaging::send_force_control_error(const char* reason,
                                               double targetForceG,
                                               double baselineG,
                                               uint32_t positionSteps,
                                               double forceG,
                                               double maxForceG)
{
  StaticJsonDocument<448> doc;

  doc["event"] = "force_control";
  doc["state"] = "error";
  doc["reason"] = reason;
  doc["target_g"] = targetForceG;
  doc["baseline_g"] = baselineG;
  doc["position_steps"] = positionSteps;
  doc["force_g"] = forceG;
  doc["max_force_g"] = maxForceG;

  send_json(doc);
}

void CBeamMessaging::send_motor_calibration_error(const char* detail,
                                                   const char* phase,
                                                   uint32_t positionSteps,
                                                   double forceG,
                                                   double targetForceG,
                                                   double maxForceG)
{
  StaticJsonDocument<384> doc;

  doc["event"] = "motor_calibration";
  doc["state"] = "error";

  if (detail != nullptr && detail[0] != '\0') {
    doc["detail"] = detail;
  }

  if (phase != nullptr && phase[0] != '\0') {
    doc["phase"] = phase;
  }

  doc["position_steps"] = positionSteps;
  doc["force_g"] = forceG;

  if (targetForceG > 0.0) {
    doc["target_g"] = targetForceG;
  }

  doc["max_force_g"] = maxForceG;

  send_json(doc);
}

void CBeamMessaging::send_motor_calibration_summary(const char* state,
                                                     double maxForceG,
                                                     uint32_t contactOffsetSteps,
                                                     uint32_t contactRangeSteps,
                                                     uint32_t step500G,
                                                     uint32_t range500Steps,
                                                     uint32_t step1000G,
                                                     uint32_t range1000Steps,
                                                     uint32_t step1500G,
                                                     uint32_t range1500Steps)
{
  StaticJsonDocument<512> doc;

  doc["event"] = "motor_calibration";
  doc["state"] = state;
  doc["max_force_g"] = maxForceG;
  doc["contact_offset_steps"] = contactOffsetSteps;
  doc["contact_range_steps"] = contactRangeSteps;
  doc["step_500_g"] = step500G;
  doc["range_500_steps"] = range500Steps;
  doc["step_1000_g"] = step1000G;
  doc["range_1000_steps"] = range1000Steps;
  doc["step_1500_g"] = step1500G;
  doc["range_1500_steps"] = range1500Steps;

  send_json(doc);
}

void CBeamMessaging::send_json(JsonDocument& doc)
{
  // Punto unico de salida del protocolo JSON.
  // En una etapa futura este metodo podra emitir tambien por SSE/HTTP
  // sin modificar la logica de control del banco.
  serializeJson(doc, Serial);
  Serial.println();
  Serial.println();
}

void CBeamMessaging::send_json_pretty(JsonDocument& doc)
{
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println();
}
