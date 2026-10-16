#ifndef BEAM_MESSAGING_H
#define BEAM_MESSAGING_H

#include <Arduino.h>
#include <ArduinoJson.h>

class CBeamConfig;

/*
  CBeamMessaging
  ------------------------------------------------------------
  Capa unica de salida de mensajes del firmware.

  Responsabilidad:
    - construir el contrato JSON de salida
    - emitir respuestas, eventos y errores
    - ocultar a la logica funcional el transporte utilizado

  En esta etapa el transporte fisico sigue siendo UART/Serial.
  La logica del banco no debe depender de ese detalle.

  Regla:
    - NO accede a NVS / Preferences
    - NO mueve motor
    - NO lee FC_HOME
    - NO lee HX711
    - NO toma decisiones de control
*/
class CBeamMessaging {
public:
  void send_all_params(const CBeamConfig& config, const char* appVersion);
  void send_version(const char* appVersion);

  void send_ack(JsonDocument& doc);
  void send_ok(JsonDocument& doc);
  void send_error(const char* reason);

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

  void send_loads(const char* source,
                  double r1_g,
                  double r2_g,
                  double sum_g,
                  double r1_n,
                  double r2_n,
                  double sum_n);

  void send_motor_calibration_cycle(uint8_t cycle,
                                    double baselineG,
                                    uint32_t contactStep,
                                    uint32_t step500G,
                                    double force500G,
                                    uint32_t step1000G,
                                    double force1000G,
                                    uint32_t step1500G,
                                    double force1500G);

  void send_motor_calibration_plan(double gapMm,
                                   uint32_t estimatedContactSteps,
                                   uint32_t approachTargetSteps,
                                   double maxForceG);

  void send_motor_calibration_progress(const char* state,
                                       uint8_t cycle,
                                       uint32_t referenceContactStep,
                                       uint32_t approachTargetStep,
                                       uint32_t positionSteps,
                                       double forceG,
                                       double targetForceG);

  void send_force_control_event(const char* state,
                                double targetForceG,
                                double baselineG,
                                uint32_t estimatedStep,
                                uint32_t positionSteps,
                                double forceG);

  void send_force_control_error(const char* reason,
                                double targetForceG,
                                double baselineG,
                                uint32_t positionSteps,
                                double forceG,
                                double maxForceG);

  void send_motor_calibration_error(const char* detail,
                                    const char* phase,
                                    uint32_t positionSteps,
                                    double forceG,
                                    double targetForceG,
                                    double maxForceG);

  void send_motor_calibration_summary(const char* state,
                                      double maxForceG,
                                      uint32_t contactOffsetSteps,
                                      uint32_t contactRangeSteps,
                                      uint32_t step500G,
                                      uint32_t range500Steps,
                                      uint32_t step1000G,
                                      uint32_t range1000Steps,
                                      uint32_t step1500G,
                                      uint32_t range1500Steps);

private:
  void send_json(JsonDocument& doc);
  void send_json_pretty(JsonDocument& doc);
};

// Instancia unica de mensajeria del firmware.
// Hoy emite por Serial; la logica funcional no conoce ese detalle.
extern CBeamMessaging Messaging;

#endif // BEAM_MESSAGING_H
