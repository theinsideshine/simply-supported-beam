/*
  Viga simplemente apoyada
  ESP32-S3 + NEMA17 + A4988 + FC_HOME + 2 x HX711

  Firmware modular 0.10.

  Objetivo de esta version:
    - mantener HOME y movimiento absoluto validados de 0.04
    - mantener interfaz host JSON y configuracion NVS
    - agregar st_mode para separar modos de ejecucion
    - agregar dos celdas de carga + HX711
    - agregar calibracion interactiva 0/500/1000/1500 g
    - persistir coeficientes de calibracion en NVS
    - agregar lectura manual de R1, R2 y suma
    - agregar streaming JSON opcional de reacciones
    - agregar calibracion motor-viga con busqueda de contacto y repetibilidad
    - guardar offset de contacto y posiciones aproximadas 500/1000/1500 g
    - corregir apply_force: aproximacion basada en contacto real calibrado
    - separar salida JSON en CBeamMessaging sin cambiar el protocolo

  Modos:
    - MODE_MANUAL:
        comandos de motor + lectura de celdas + streaming
    - MODE_CALIBRATION:
        calibracion interactiva bloqueante de celdas o motor-viga
    - MODE_EXPERIMENT:
        reservado para la proxima etapa

  Regla de arquitectura:
    - .ino crea objetos y coordina modos / FSM
    - CBeamConfig concentra entrada JSON + NVS + copia RAM
    - CBeamMessaging concentra la salida JSON del protocolo
    - CLoadCell concentra HX711 + matematica + calibracion
    - CLoadCell NO accede a NVS
*/

#include <Arduino.h>
#include <math.h>

#include "beam_system.h"
#include "beam_config.h"
#include "beam_messaging.h"
#include "a4988_stepper.h"
#include "home_switch.h"
#include "load_cell.h"

// ============================================================
// Estados globales del firmware
// Las definiciones de compilacion viven en beam_system.h.
// ============================================================
uint8_t st_loop = ST_INIT;
uint8_t st_mode = MODE_MANUAL;
uint8_t calibrationMode = CAL_NONE;
uint8_t calibrationCell = 0;

uint8_t homeState = HOME_IDLE;
uint32_t homeVerifyStartMs = 0;
long homePhaseSteps = 0;

// Snapshot de parametros al comenzar HOME.
uint32_t homeStepPeriodUs = BEAM_STEP_PERIOD_US_DEFAULT;
uint32_t homeSlowPeriodUs = BEAM_HOME_SLOW_PERIOD_US_DEFAULT;
uint32_t homeBackoffSteps = BEAM_HOME_BACKOFF_STEPS_DEFAULT;
uint32_t homeDebounceMs = BEAM_HOME_DEBOUNCE_MS_DEFAULT;
uint32_t homeSeekMaxSteps = BEAM_HOME_SEEK_MAX_STEPS_DEFAULT;

// ============================================================
// Posicion absoluta
// ============================================================
bool homed = false;
long currentPositionSteps = 0;
long moveTargetSteps = 0;
uint8_t moveDirection = DIR_AWAY;

// ============================================================
// Objetos
// ============================================================
CBeamConfig Config;
CA4988Stepper Motor;
CHomeSwitch HomeSwitch;
CLoadCell LoadCell1(PIN_CELL1_DOUT, PIN_CELL1_SCK);
CLoadCell LoadCell2(PIN_CELL2_DOUT, PIN_CELL2_SCK);

// ============================================================
// Estado / temporizacion auxiliar
// ============================================================
int ultimoEstadoHome = -1;
uint32_t lastLoadStreamMs = 0;

struct MotorCalibrationCycleResult {
  double baseline_g;
  uint32_t contact_step;
  uint32_t step_500_g;
  double force_500_g;
  uint32_t step_1000_g;
  double force_1000_g;
  uint32_t step_1500_g;
  double force_1500_g;
};

// ============================================================
// Helpers generales
// ============================================================
bool isHomeActive()
{
  return st_loop == ST_HOME;
}

bool isMoveActive()
{
  return st_loop == ST_MOVE_ABS;
}

bool isPhysicalOperationActive()
{
  return isHomeActive() || isMoveActive();
}

void homeSetState(uint8_t newState)
{
  homeState = newState;
  homePhaseSteps = 0;
}

bool startMotor(uint8_t directionLevel, uint32_t periodUs)
{
  if (!Motor.start(directionLevel, periodUs)) {
    Messaging.send_event("motor", "error", "invalid_step_period");
    return false;
  }

  return true;
}

void invalidatePosition(const char* reason)
{
  homed = false;

  if (reason != nullptr && reason[0] != '\0') {
    Messaging.send_event("position", "invalid", reason);
  }
}

// ============================================================
// Celdas de carga - adquisicion cooperativa
// ============================================================
void processLoadCells()
{
  LoadCell1.update();
  LoadCell2.update();
}

bool loadsAvailable()
{
  return LoadCell1.isCalibrated() &&
         LoadCell2.isCalibrated() &&
         LoadCell1.hasSample() &&
         LoadCell2.hasSample();
}

bool sendCurrentLoads(const char* source)
{
  if (!loadsAvailable()) {
    return false;
  }

  double r1_g = LoadCell1.getLastGrams();
  double r2_g = LoadCell2.getLastGrams();

  if (!isfinite(r1_g) || !isfinite(r2_g)) {
    return false;
  }

  double sum_g = r1_g + r2_g;
  double r1_n = (r1_g / 1000.0) * LOAD_CELL_GRAVITY;
  double r2_n = (r2_g / 1000.0) * LOAD_CELL_GRAVITY;
  double sum_n = r1_n + r2_n;

  Messaging.send_loads(
    source,
    r1_g,
    r2_g,
    sum_g,
    r1_n,
    r2_n,
    sum_n
  );

  return true;
}

void processLoadStream()
{
  if (!Config.get_load_stream_enabled()) {
    return;
  }

  uint32_t now = millis();
  uint32_t periodMs = Config.get_load_stream_period_ms();

  if ((uint32_t)(now - lastLoadStreamMs) < periodMs) {
    return;
  }

  lastLoadStreamMs = now;
  sendCurrentLoads("stream");
}

// Forward declarations de la FSM de HOME usadas por la calibracion
// bloqueante del conjunto motor-viga.
void startHoming();
void processHoming();

// ============================================================
// Helpers bloqueantes de calibracion motor-viga
// ============================================================
void clearSerialInput()
{
  while (Serial.available()) {
    Serial.read();
  }
}

void waitForOkMotorCalibration()
{
  clearSerialInput();

  while (true) {
    if (Serial.available()) {
      String text = Serial.readStringUntil('\n');
      text.trim();
      text.toLowerCase();

      if (text == "ok") {
        return;
      }

      if (text.length() > 0) {
        Serial.println("Escriba solamente: ok");
      }
    }

    delay(10);
  }
}

bool readPositiveDoubleBlocking(const char* prompt,
                                double minExclusive,
                                double maxInclusive,
                                double& value)
{
  clearSerialInput();

  Serial.println();
  Serial.println(prompt);

  while (true) {
    if (Serial.available()) {
      String text = Serial.readStringUntil('\n');
      text.trim();

      double parsed = text.toFloat();

      if (parsed > minExclusive && parsed <= maxInclusive) {
        value = parsed;
        return true;
      }

      Serial.print("Valor invalido. Rango permitido: > ");
      Serial.print(minExclusive, 1);
      Serial.print(" y <= ");
      Serial.println(maxInclusive, 1);
    }

    delay(10);
  }
}

bool readLoadsAverageBlocking(double& r1G,
                              double& r2G,
                              double& sumG)
{
  if (!LoadCell1.isCalibrated() || !LoadCell2.isCalibrated()) {
    return false;
  }

  if (!LoadCell1.readGramsAverageBlocking(
        r1G,
        MOTOR_CAL_AVG_SAMPLES,
        MOTOR_CAL_READ_TIMEOUT_MS)) {
    return false;
  }

  if (!LoadCell2.readGramsAverageBlocking(
        r2G,
        MOTOR_CAL_AVG_SAMPLES,
        MOTOR_CAL_READ_TIMEOUT_MS)) {
    return false;
  }

  sumG = r1G + r2G;
  return isfinite(sumG);
}

bool runHomeBlocking()
{
  startHoming();

  while (isHomeActive()) {
    processHoming();

    if (Motor.update()) {
      homePhaseSteps++;
    }

    delay(1);
  }

  return homed && currentPositionSteps == 0;
}

bool moveAwayCalibrationBlocking(uint32_t steps)
{
  if (!homed || steps == 0UL) {
    return homed;
  }

  uint32_t maxPosition = Config.get_max_position_steps();
  uint32_t target = (uint32_t)currentPositionSteps + steps;

  if (maxPosition == 0UL || target > maxPosition) {
    return false;
  }

  moveDirection = DIR_AWAY;
  moveTargetSteps = (long)target;
  st_loop = ST_MOVE_ABS;

  if (!startMotor(DIR_AWAY, Config.get_move_fast_period_us())) {
    st_loop = ST_READY;
    return false;
  }

  uint32_t generated = 0;

  while (generated < steps) {
    if (Motor.update()) {
      currentPositionSteps++;
      generated++;
    }

    delay(1);
  }

  Motor.stop();
  st_loop = ST_READY;
  return true;
}

bool checkMotorCalibrationForce(double baselineG,
                                double maxForceG,
                                const char* phase,
                                double targetForceG,
                                double& appliedForceG)
{
  double r1G = 0.0;
  double r2G = 0.0;
  double sumG = 0.0;

  if (!readLoadsAverageBlocking(r1G, r2G, sumG)) {
    Messaging.send_motor_calibration_error(
      "load_read_failed",
      phase,
      (uint32_t)currentPositionSteps,
      0.0,
      targetForceG,
      maxForceG
    );
    return false;
  }

  appliedForceG = sumG - baselineG;

  if (appliedForceG >= maxForceG) {
    Motor.stop();
    Messaging.send_event("safety", "stop", "max_force_reached");
    Messaging.send_motor_calibration_error(
      "max_force_reached",
      phase,
      (uint32_t)currentPositionSteps,
      appliedForceG,
      targetForceG,
      maxForceG
    );
    return false;
  }

  return true;
}

bool approachToStepCalibration(uint32_t targetStep,
                               double baselineG,
                               double maxForceG,
                               const char* phase,
                               bool& contactDetected,
                               uint32_t& contactStep)
{
  contactDetected = false;
  contactStep = 0UL;

  while ((uint32_t)currentPositionSteps < targetStep) {
    uint32_t remaining = targetStep - (uint32_t)currentPositionSteps;
    uint32_t chunk = remaining > (uint32_t)MOTOR_CAL_COARSE_STEP_CHUNK
      ? (uint32_t)MOTOR_CAL_COARSE_STEP_CHUNK
      : remaining;

    if (!moveAwayCalibrationBlocking(chunk)) {
      double appliedForceG = 0.0;
      checkMotorCalibrationForce(
        baselineG,
        maxForceG,
        phase,
        0.0,
        appliedForceG
      );
      Messaging.send_motor_calibration_error(
        "max_position_reached",
        phase,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        0.0,
        maxForceG
      );
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);

    double appliedForceG = 0.0;
    if (!checkMotorCalibrationForce(
          baselineG,
          maxForceG,
          phase,
          0.0,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
      contactDetected = true;
      contactStep = (uint32_t)currentPositionSteps;
      return true;
    }
  }

  return true;
}

bool findContactFineCalibration(double baselineG,
                                double maxForceG,
                                const char* phase,
                                uint32_t& contactStep)
{
  while (true) {
    if (!moveAwayCalibrationBlocking(MOTOR_CAL_FINE_STEP_CHUNK)) {
      double appliedForceG = 0.0;
      checkMotorCalibrationForce(
        baselineG,
        maxForceG,
        phase,
        0.0,
        appliedForceG
      );
      Messaging.send_motor_calibration_error(
        "max_position_reached",
        phase,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        0.0,
        maxForceG
      );
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);

    double appliedForceG = 0.0;
    if (!checkMotorCalibrationForce(
          baselineG,
          maxForceG,
          phase,
          0.0,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
      contactStep = (uint32_t)currentPositionSteps;
      return true;
    }
  }
}

bool seekForceTargetCalibration(const char* phase,
                                uint8_t cycle,
                                double targetForceG,
                                double baselineG,
                                double maxForceG,
                                uint32_t& reachedStep,
                                double& reachedForceG)
{
  Messaging.send_motor_calibration_progress(
    "target_search",
    cycle,
    0UL,
    0UL,
    (uint32_t)currentPositionSteps,
    NAN,
    targetForceG
  );

  while (true) {
    double appliedForceG = 0.0;

    if (!checkMotorCalibrationForce(
          baselineG,
          maxForceG,
          phase,
          targetForceG,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= targetForceG) {
      reachedStep = (uint32_t)currentPositionSteps;
      reachedForceG = appliedForceG;

      Messaging.send_motor_calibration_progress(
        "target_reached",
        cycle,
        0UL,
        0UL,
        reachedStep,
        reachedForceG,
        targetForceG
      );

      return true;
    }

    double remainingG = targetForceG - appliedForceG;
    uint32_t chunk = MOTOR_CAL_FORCE_FINE_STEPS;

    if (remainingG > MOTOR_CAL_FORCE_COARSE_MARGIN_G) {
      chunk = MOTOR_CAL_FORCE_COARSE_STEPS;
    }
    else if (remainingG > MOTOR_CAL_FORCE_MEDIUM_MARGIN_G) {
      chunk = MOTOR_CAL_FORCE_MEDIUM_STEPS;
    }

    uint32_t maxPosition = Config.get_max_position_steps();
    uint32_t nextTarget = (uint32_t)currentPositionSteps + chunk;

    if (maxPosition == 0UL || nextTarget > maxPosition) {
      Messaging.send_motor_calibration_error(
        "max_position_reached",
        phase,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        targetForceG,
        maxForceG
      );
      return false;
    }

    if (!moveAwayCalibrationBlocking(chunk)) {
      Messaging.send_motor_calibration_error(
        "max_position_reached",
        phase,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        targetForceG,
        maxForceG
      );
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);
  }
}

uint32_t interpolateStep(uint32_t x0, uint32_t x1, double fraction)
{
  if (fraction <= 0.0) {
    return x0;
  }

  if (fraction >= 1.0) {
    return x1;
  }

  double value = (double)x0 + ((double)x1 - (double)x0) * fraction;
  return (uint32_t)lround(value);
}

uint32_t estimateStepForForce(double targetForceG)
{
  uint32_t contact = Config.get_motor_contact_offset_steps();
  uint32_t step500 = Config.get_motor_step_500_g();
  uint32_t step1000 = Config.get_motor_step_1000_g();
  uint32_t step1500 = Config.get_motor_step_1500_g();

  if (targetForceG <= MOTOR_CAL_TARGET_500_G) {
    return interpolateStep(
      contact,
      step500,
      targetForceG / MOTOR_CAL_TARGET_500_G
    );
  }

  if (targetForceG <= MOTOR_CAL_TARGET_1000_G) {
    return interpolateStep(
      step500,
      step1000,
      (targetForceG - MOTOR_CAL_TARGET_500_G) /
      (MOTOR_CAL_TARGET_1000_G - MOTOR_CAL_TARGET_500_G)
    );
  }

  return interpolateStep(
    step1000,
    step1500,
    (targetForceG - MOTOR_CAL_TARGET_1000_G) /
    (MOTOR_CAL_TARGET_1500_G - MOTOR_CAL_TARGET_1000_G)
  );
}

bool readAppliedForceBlocking(double baselineG,
                              double targetForceG,
                              double maxForceG,
                              double& appliedForceG)
{
  double r1G = 0.0;
  double r2G = 0.0;
  double sumG = 0.0;

  if (!readLoadsAverageBlocking(r1G, r2G, sumG)) {
    Messaging.send_force_control_error(
      "load_read_failed",
      targetForceG,
      baselineG,
      (uint32_t)currentPositionSteps,
      0.0,
      maxForceG
    );
    return false;
  }

  appliedForceG = sumG - baselineG;

  if (appliedForceG >= maxForceG) {
    Motor.stop();
    Messaging.send_event("safety", "stop", "max_force_reached");
    Messaging.send_force_control_error(
      "max_force_reached",
      targetForceG,
      baselineG,
      (uint32_t)currentPositionSteps,
      appliedForceG,
      maxForceG
    );
    return false;
  }

  return true;
}

bool runApplyForce(double targetForceG)
{
  double maxForceG = Config.get_motor_max_force_g();

  delay(MOTOR_CAL_SETTLE_MS);

  double r1G = 0.0;
  double r2G = 0.0;
  double baselineG = 0.0;

  if (!readLoadsAverageBlocking(r1G, r2G, baselineG)) {
    Messaging.send_force_control_error(
      "baseline_failed",
      targetForceG,
      0.0,
      (uint32_t)currentPositionSteps,
      0.0,
      maxForceG
    );
    return false;
  }

  // La posicion de objetivo estimada se conserva solo como informacion
  // y referencia historica. NO se usa para hacer un salto ciego.
  uint32_t estimatedStep = estimateStepForForce(targetForceG);

  // La aproximacion rapida se hace respecto del contacto REAL aprendido
  // durante la calibracion motor-viga.
  uint32_t referenceContactStep = Config.get_motor_contact_offset_steps();

  if (referenceContactStep == 0UL) {
    Messaging.send_force_control_error(
      "invalid_contact_offset",
      targetForceG,
      baselineG,
      (uint32_t)currentPositionSteps,
      0.0,
      maxForceG
    );
    return false;
  }

  uint32_t approachTarget =
    referenceContactStep > MOTOR_APPLY_PRECONTACT_MARGIN_STEPS
      ? referenceContactStep - MOTOR_APPLY_PRECONTACT_MARGIN_STEPS
      : 0UL;

  Messaging.send_force_control_event(
    "start",
    targetForceG,
    baselineG,
    estimatedStep,
    (uint32_t)currentPositionSteps,
    0.0
  );

  Messaging.send_force_control_event(
    "fast_approach",
    targetForceG,
    baselineG,
    estimatedStep,
    approachTarget,
    0.0
  );

  double appliedForceG = 0.0;
  bool contactDetected = false;

  // ----------------------------------------------------------
  // 1) Aproximacion rapida hasta antes del contacto calibrado.
  //    Se controla fuerza en cada bloque por seguridad.
  // ----------------------------------------------------------
  while ((uint32_t)currentPositionSteps < approachTarget) {
    uint32_t remaining = approachTarget - (uint32_t)currentPositionSteps;
    uint32_t chunk = remaining > (uint32_t)MOTOR_APPLY_APPROACH_CHUNK_STEPS
      ? (uint32_t)MOTOR_APPLY_APPROACH_CHUNK_STEPS
      : remaining;

    uint32_t maxPosition = Config.get_max_position_steps();
    uint32_t nextTarget = (uint32_t)currentPositionSteps + chunk;

    if (maxPosition == 0UL || nextTarget > maxPosition) {
      Messaging.send_force_control_error(
        "max_position_reached",
        targetForceG,
        baselineG,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        maxForceG
      );
      return false;
    }

    if (!moveAwayCalibrationBlocking(chunk)) {
      Messaging.send_force_control_error(
        "max_position_reached",
        targetForceG,
        baselineG,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        maxForceG
      );
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);

    if (!readAppliedForceBlocking(
          baselineG,
          targetForceG,
          maxForceG,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= targetForceG) {
      Messaging.send_force_control_event(
        "target_reached",
        targetForceG,
        baselineG,
        estimatedStep,
        (uint32_t)currentPositionSteps,
        appliedForceG
      );
      return true;
    }

    // Si la mecanica cambio y el contacto aparecio antes de lo esperado,
    // se abandona inmediatamente la aproximacion rapida.
    if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
      contactDetected = true;

      Messaging.send_force_control_event(
        "contact_detected",
        targetForceG,
        baselineG,
        estimatedStep,
        (uint32_t)currentPositionSteps,
        appliedForceG
      );
      break;
    }
  }

  // Si approachTarget era 0 o aun no tenemos una lectura reciente,
  // leer fuerza antes de iniciar la busqueda fina.
  if (!contactDetected) {
    if (!readAppliedForceBlocking(
          baselineG,
          targetForceG,
          maxForceG,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= targetForceG) {
      Messaging.send_force_control_event(
        "target_reached",
        targetForceG,
        baselineG,
        estimatedStep,
        (uint32_t)currentPositionSteps,
        appliedForceG
      );
      return true;
    }

    if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
      contactDetected = true;

      Messaging.send_force_control_event(
        "contact_detected",
        targetForceG,
        baselineG,
        estimatedStep,
        (uint32_t)currentPositionSteps,
        appliedForceG
      );
    }
  }

  // ----------------------------------------------------------
  // 2) Desde el margen previo al contacto, avanzar de a 1 paso
  //    hasta detectar contacto REAL por fuerza.
  // ----------------------------------------------------------
  if (!contactDetected) {
    Messaging.send_force_control_event(
      "fine_contact_search",
      targetForceG,
      baselineG,
      estimatedStep,
      (uint32_t)currentPositionSteps,
      appliedForceG
    );

    while (!contactDetected) {
      uint32_t maxPosition = Config.get_max_position_steps();
      uint32_t nextTarget = (uint32_t)currentPositionSteps + 1UL;

      if (maxPosition == 0UL || nextTarget > maxPosition) {
        Messaging.send_force_control_error(
          "max_position_reached",
          targetForceG,
          baselineG,
          (uint32_t)currentPositionSteps,
          appliedForceG,
          maxForceG
        );
        return false;
      }

      if (!moveAwayCalibrationBlocking(1UL)) {
        Messaging.send_force_control_error(
          "max_position_reached",
          targetForceG,
          baselineG,
          (uint32_t)currentPositionSteps,
          appliedForceG,
          maxForceG
        );
        return false;
      }

      delay(MOTOR_CAL_SETTLE_MS);

      if (!readAppliedForceBlocking(
            baselineG,
            targetForceG,
            maxForceG,
            appliedForceG)) {
        return false;
      }

      if (appliedForceG >= targetForceG) {
        Messaging.send_force_control_event(
          "target_reached",
          targetForceG,
          baselineG,
          estimatedStep,
          (uint32_t)currentPositionSteps,
          appliedForceG
        );
        return true;
      }

      if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
        contactDetected = true;

        Messaging.send_force_control_event(
          "contact_detected",
          targetForceG,
          baselineG,
          estimatedStep,
          (uint32_t)currentPositionSteps,
          appliedForceG
        );
      }
    }
  }

  // ----------------------------------------------------------
  // 3) Contacto confirmado. Desde aca manda exclusivamente la
  //    fuerza medida. Se usan pasos adaptativos segun cercania.
  // ----------------------------------------------------------
  Messaging.send_force_control_event(
    "target_search",
    targetForceG,
    baselineG,
    estimatedStep,
    (uint32_t)currentPositionSteps,
    appliedForceG
  );

  while (true) {
    if (!readAppliedForceBlocking(
          baselineG,
          targetForceG,
          maxForceG,
          appliedForceG)) {
      return false;
    }

    if (appliedForceG >= targetForceG) {
      Messaging.send_force_control_event(
        "target_reached",
        targetForceG,
        baselineG,
        estimatedStep,
        (uint32_t)currentPositionSteps,
        appliedForceG
      );
      return true;
    }

    double remainingG = targetForceG - appliedForceG;
    uint32_t chunk = MOTOR_CAL_FORCE_FINE_STEPS;

    if (remainingG > MOTOR_CAL_FORCE_COARSE_MARGIN_G) {
      chunk = MOTOR_CAL_FORCE_COARSE_STEPS;
    }
    else if (remainingG > MOTOR_CAL_FORCE_MEDIUM_MARGIN_G) {
      chunk = MOTOR_CAL_FORCE_MEDIUM_STEPS;
    }

    uint32_t maxPosition = Config.get_max_position_steps();
    uint32_t nextTarget = (uint32_t)currentPositionSteps + chunk;

    if (maxPosition == 0UL || nextTarget > maxPosition) {
      Messaging.send_force_control_error(
        "max_position_reached",
        targetForceG,
        baselineG,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        maxForceG
      );
      return false;
    }

    if (!moveAwayCalibrationBlocking(chunk)) {
      Messaging.send_force_control_error(
        "max_position_reached",
        targetForceG,
        baselineG,
        (uint32_t)currentPositionSteps,
        appliedForceG,
        maxForceG
      );
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);
  }
}

uint32_t averageSteps(const uint32_t values[], uint8_t count)
{
  uint64_t sum = 0;

  for (uint8_t i = 0; i < count; i++) {
    sum += values[i];
  }

  return (uint32_t)((sum + (count / 2U)) / count);
}

uint32_t rangeSteps(const uint32_t values[], uint8_t count)
{
  uint32_t minValue = values[0];
  uint32_t maxValue = values[0];

  for (uint8_t i = 1; i < count; i++) {
    if (values[i] < minValue) {
      minValue = values[i];
    }

    if (values[i] > maxValue) {
      maxValue = values[i];
    }
  }

  return maxValue - minValue;
}

bool runMotorCalibration()
{
  if (!LoadCell1.isCalibrated() || !LoadCell2.isCalibrated()) {
    Messaging.send_event("motor_calibration", "error", "load_cells_not_calibrated");
    return false;
  }

  if (Config.get_max_position_steps() == 0UL) {
    Messaging.send_event("motor_calibration", "error", "max_position_not_configured");
    return false;
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println("     CALIBRACION MOTOR - VIGA");
  Serial.println("======================================");
  Serial.println();
  Serial.println("La viga debe estar colocada, sin carga externa,");
  Serial.println("y el aplicador debe quedar libre en HOME.");
  Serial.println();

  double gapMm = 0.0;
  double maxForceG = 0.0;

  readPositiveDoubleBlocking(
    "Ingrese la luz aproximada aplicador-viga [mm]:",
    0.0,
    50.0,
    gapMm
  );

  // Debe quedar por encima del ultimo punto normal de ensayo (1500 g).
  readPositiveDoubleBlocking(
    "Ingrese la fuerza MAXIMA de seguridad para esta probeta [g] (debe ser > 1500):",
    MOTOR_CAL_TARGET_1500_G,
    20000.0,
    maxForceG
  );

  Serial.println();
  Serial.print("Luz estimada        : ");
  Serial.print(gapMm, 2);
  Serial.println(" mm");
  Serial.print("Fuerza maxima       : ");
  Serial.print(maxForceG, 1);
  Serial.println(" g");
  Serial.println();
  Serial.println("Se hara una busqueda preliminar y luego 3 repeticiones.");
  Serial.println("Objetivos: 500 / 1000 / 1500 g.");
  Serial.println("Escriba ok para comenzar.");

  waitForOkMotorCalibration();

  // ----------------------------------------------------------
  // Busqueda preliminar: usa el gap ingresado solo para acercarse.
  // ----------------------------------------------------------
  Messaging.send_event("motor_calibration", "start", "rough_contact_search");

  if (!runHomeBlocking()) {
    Messaging.send_event("motor_calibration", "error", "home_failed");
    return false;
  }

  delay(MOTOR_CAL_SETTLE_MS);

  double r1G = 0.0;
  double r2G = 0.0;
  double baselineG = 0.0;

  if (!readLoadsAverageBlocking(r1G, r2G, baselineG)) {
    Messaging.send_event("motor_calibration", "error", "baseline_failed");
    return false;
  }

  long estimatedContact = lround(gapMm * MOTOR_CAL_STEPS_PER_MM_ESTIMATE);
  long roughApproach = estimatedContact - MOTOR_CAL_APPROACH_MARGIN_STEPS;

  if (roughApproach < 0L) {
    roughApproach = 0L;
  }

  uint32_t maxPosition = Config.get_max_position_steps();
  if ((uint32_t)roughApproach > maxPosition) {
    roughApproach = (long)maxPosition;
  }

  Messaging.send_motor_calibration_plan(
    gapMm,
    estimatedContact > 0L ? (uint32_t)estimatedContact : 0UL,
    (uint32_t)roughApproach,
    maxForceG
  );

  bool contactDetected = false;
  uint32_t roughContactStep = 0UL;

  Messaging.send_event_value(
    "motor_calibration",
    "rough_approach",
    "target_steps",
    (uint32_t)roughApproach
  );

  // El gap ingresado se usa para una aproximacion rapida unica hasta
  // unos pocos pasos antes del contacto estimado. Desde ahi la
  // deteccion real de contacto se hace de a un paso por fuerza.
  if ((uint32_t)roughApproach > 0UL) {
    if (!moveAwayCalibrationBlocking((uint32_t)roughApproach)) {
      Messaging.send_motor_calibration_error(
        "max_position_reached",
        "rough_approach",
        (uint32_t)currentPositionSteps,
        0.0,
        0.0,
        maxForceG
      );
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    delay(MOTOR_CAL_SETTLE_MS);

    double appliedForceG = 0.0;
    if (!checkMotorCalibrationForce(
          baselineG,
          maxForceG,
          "rough_approach",
          0.0,
          appliedForceG)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    if (appliedForceG >= MOTOR_CAL_CONTACT_THRESHOLD_G) {
      contactDetected = true;
      roughContactStep = (uint32_t)currentPositionSteps;
    }
  }

  if (!contactDetected) {
    Messaging.send_motor_calibration_progress(
      "fine_contact_search",
      0,
      estimatedContact > 0L ? (uint32_t)estimatedContact : 0UL,
      (uint32_t)roughApproach,
      (uint32_t)currentPositionSteps,
      NAN,
      0.0
    );

    if (!findContactFineCalibration(
          baselineG,
          maxForceG,
          "rough_contact_search",
          roughContactStep)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }
  }

  Messaging.send_event_value(
    "motor_calibration",
    "rough_contact",
    "position_steps",
    roughContactStep
  );

  // Volver a HOME para que las tres repeticiones empiecen desde la misma referencia.
  if (!runHomeBlocking()) {
    Messaging.send_event("motor_calibration", "error", "home_after_rough_search_failed");
    return false;
  }

  MotorCalibrationCycleResult cycles[MOTOR_CAL_REPETITIONS];
  uint32_t contactReferenceStep = roughContactStep;

  // ----------------------------------------------------------
  // Repetibilidad: 3 ciclos completos.
  // ----------------------------------------------------------
  for (uint8_t cycle = 0; cycle < MOTOR_CAL_REPETITIONS; cycle++) {
    Serial.println();
    Serial.print("--- REPETICION ");
    Serial.print(cycle + 1);
    Serial.print(" / ");
    Serial.print(MOTOR_CAL_REPETITIONS);
    Serial.println(" ---");

    delay(MOTOR_CAL_SETTLE_MS);

    if (!readLoadsAverageBlocking(r1G, r2G, baselineG)) {
      Messaging.send_event("motor_calibration", "error", "baseline_failed");
      return false;
    }

    cycles[cycle].baseline_g = baselineG;

    uint32_t preContactTarget = contactReferenceStep > MOTOR_CAL_PRECONTACT_MARGIN_STEPS
      ? contactReferenceStep - MOTOR_CAL_PRECONTACT_MARGIN_STEPS
      : 0UL;

    Messaging.send_motor_calibration_progress(
      "fast_approach",
      cycle + 1,
      contactReferenceStep,
      preContactTarget,
      (uint32_t)currentPositionSteps,
      NAN,
      0.0
    );

    contactDetected = false;
    uint32_t contactStep = 0UL;

    if (!approachToStepCalibration(
          preContactTarget,
          baselineG,
          maxForceG,
          "contact_approach",
          contactDetected,
          contactStep)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    if (!contactDetected) {
      Messaging.send_motor_calibration_progress(
        "fine_contact_search",
        cycle + 1,
        contactReferenceStep,
        preContactTarget,
        (uint32_t)currentPositionSteps,
        NAN,
        0.0
      );

      if (!findContactFineCalibration(
            baselineG,
            maxForceG,
            "contact_search",
            contactStep)) {
        if (homed && currentPositionSteps > 0) {
          runHomeBlocking();
        }
        return false;
      }
    }

    cycles[cycle].contact_step = contactStep;

    double contactForceG = 0.0;
    if (!checkMotorCalibrationForce(
          baselineG,
          maxForceG,
          "contact_detected",
          0.0,
          contactForceG)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    Messaging.send_motor_calibration_progress(
      "contact_detected",
      cycle + 1,
      contactReferenceStep,
      preContactTarget,
      contactStep,
      contactForceG,
      0.0
    );

    // La siguiente repeticion usa el ultimo contacto realmente medido,
    // no la estimacion ingresada ni necesariamente el contacto preliminar.
    contactReferenceStep = contactStep;

    if (!seekForceTargetCalibration(
          "target_500",
          cycle + 1,
          MOTOR_CAL_TARGET_500_G,
          baselineG,
          maxForceG,
          cycles[cycle].step_500_g,
          cycles[cycle].force_500_g)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    if (!seekForceTargetCalibration(
          "target_1000",
          cycle + 1,
          MOTOR_CAL_TARGET_1000_G,
          baselineG,
          maxForceG,
          cycles[cycle].step_1000_g,
          cycles[cycle].force_1000_g)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    if (!seekForceTargetCalibration(
          "target_1500",
          cycle + 1,
          MOTOR_CAL_TARGET_1500_G,
          baselineG,
          maxForceG,
          cycles[cycle].step_1500_g,
          cycles[cycle].force_1500_g)) {
      if (homed && currentPositionSteps > 0) {
        runHomeBlocking();
      }
      return false;
    }

    Messaging.send_motor_calibration_cycle(
      cycle + 1,
      cycles[cycle].baseline_g,
      cycles[cycle].contact_step,
      cycles[cycle].step_500_g,
      cycles[cycle].force_500_g,
      cycles[cycle].step_1000_g,
      cycles[cycle].force_1000_g,
      cycles[cycle].step_1500_g,
      cycles[cycle].force_1500_g
    );

    // Cada ciclo termina descargando y reestableciendo HOME.
    Messaging.send_motor_calibration_progress(
      "return_home",
      cycle + 1,
      contactReferenceStep,
      0UL,
      (uint32_t)currentPositionSteps,
      NAN,
      0.0
    );

    if (!runHomeBlocking()) {
      Messaging.send_event("motor_calibration", "error", "home_between_cycles_failed");
      return false;
    }
  }

  uint32_t contactSteps[MOTOR_CAL_REPETITIONS];
  uint32_t steps500[MOTOR_CAL_REPETITIONS];
  uint32_t steps1000[MOTOR_CAL_REPETITIONS];
  uint32_t steps1500[MOTOR_CAL_REPETITIONS];

  for (uint8_t i = 0; i < MOTOR_CAL_REPETITIONS; i++) {
    contactSteps[i] = cycles[i].contact_step;
    steps500[i] = cycles[i].step_500_g;
    steps1000[i] = cycles[i].step_1000_g;
    steps1500[i] = cycles[i].step_1500_g;
  }

  uint32_t contactOffset = averageSteps(contactSteps, MOTOR_CAL_REPETITIONS);
  uint32_t step500 = averageSteps(steps500, MOTOR_CAL_REPETITIONS);
  uint32_t step1000 = averageSteps(steps1000, MOTOR_CAL_REPETITIONS);
  uint32_t step1500 = averageSteps(steps1500, MOTOR_CAL_REPETITIONS);

  uint32_t contactRange = rangeSteps(contactSteps, MOTOR_CAL_REPETITIONS);
  uint32_t range500 = rangeSteps(steps500, MOTOR_CAL_REPETITIONS);
  uint32_t range1000 = rangeSteps(steps1000, MOTOR_CAL_REPETITIONS);
  uint32_t range1500 = rangeSteps(steps1500, MOTOR_CAL_REPETITIONS);

  if (!Config.set_motor_calibration(
        maxForceG,
        contactOffset,
        step500,
        step1000,
        step1500)) {
    Messaging.send_event("motor_calibration", "error", "persist_failed");
    return false;
  }

  Messaging.send_motor_calibration_summary(
    "ok",
    maxForceG,
    contactOffset,
    contactRange,
    step500,
    range500,
    step1000,
    range1000,
    step1500,
    range1500
  );

  Serial.println();
  Serial.println("CALIBRACION MOTOR-VIGA FINALIZADA.");
  Serial.print("Offset de contacto guardado : ");
  Serial.print(contactOffset);
  Serial.println(" pasos desde HOME");
  Serial.print("Paso medio 500 g            : ");
  Serial.println(step500);
  Serial.print("Paso medio 1000 g           : ");
  Serial.println(step1000);
  Serial.print("Paso medio 1500 g           : ");
  Serial.println(step1500);

  return true;
}

// ============================================================
// HOME
// ============================================================
void homeAbort(const char* reason, bool disableDriver)
{
  Motor.stop();

  if (disableDriver) {
    Motor.disable();
  }

  homeState = HOME_IDLE;
  homePhaseSteps = 0;
  st_loop = ST_READY;
  homed = false;

  Messaging.send_event("home", "aborted", reason);
}

void homeError(const char* reason)
{
  Motor.stop();
  homeState = HOME_IDLE;
  homePhaseSteps = 0;
  st_loop = ST_READY;
  homed = false;

  Messaging.send_event("home", "error", reason);
}

void homeOk()
{
  Motor.stop();
  homeState = HOME_IDLE;
  homePhaseSteps = 0;
  st_loop = ST_READY;

  currentPositionSteps = 0;
  homed = true;

  Messaging.send_event_value("home", "ok", "reference_step", 0UL);
}

void startHoming()
{
  homed = false;

  homeStepPeriodUs = Config.get_step_period_us();
  homeSlowPeriodUs = Config.get_home_slow_period_us();
  homeBackoffSteps = Config.get_home_backoff_steps();
  homeDebounceMs = Config.get_home_debounce_ms();
  homeSeekMaxSteps = Config.get_home_seek_max_steps();

  Messaging.send_event("home", "start");
  Messaging.send_event("home_direction", "toward_switch", DIR_HOME == HIGH ? "DIR=HIGH" : "DIR=LOW");
  Messaging.send_event("home_direction", "away_from_switch", DIR_AWAY == HIGH ? "DIR=HIGH" : "DIR=LOW");
  Messaging.send_event("home_initial_switch", HomeSwitch.stateText());

  st_loop = ST_HOME;

  if (HomeSwitch.isActive()) {
    Messaging.send_event("home", "release_start");
    homeSetState(HOME_RELEASE);

    if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
      homeError("motor_start_failed");
    }
  }
  else {
    Messaging.send_event("home", "seek_fast_start");
    homeSetState(HOME_SEEK_FAST);

    if (!startMotor(DIR_HOME, homeStepPeriodUs)) {
      homeError("motor_start_failed");
    }
  }
}

void processHoming()
{
  if (!isHomeActive()) {
    return;
  }

  int estadoHome = HomeSwitch.readRaw();

  switch (homeState) {

    case HOME_RELEASE:
      if (HomeSwitch.isFree(estadoHome)) {
        Motor.stop();
        homeVerifyStartMs = millis();
        homeSetState(HOME_VERIFY_RELEASE);

        Messaging.send_event_value(
          "home",
          "verify_release",
          "debounce_ms",
          homeDebounceMs
        );
      }
      else if (homePhaseSteps >= HOME_RELEASE_MAX_STEPS) {
        homeError("release_step_limit");
      }
      break;

    case HOME_VERIFY_RELEASE:
      if (!HomeSwitch.isFree(estadoHome)) {
        Messaging.send_event("home", "release_bounce", "continue_release");
        homeSetState(HOME_RELEASE);

        if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      else if ((millis() - homeVerifyStartMs) >= homeDebounceMs) {
        Messaging.send_event_value(
          "home",
          "release_confirmed",
          "backoff_steps",
          homeBackoffSteps
        );

        homeSetState(HOME_BACKOFF);

        if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      break;

    case HOME_BACKOFF:
      if (homePhaseSteps >= (long)homeBackoffSteps) {
        Motor.stop();

        Messaging.send_event_value(
          "home",
          "seek_slow_start",
          "step_period_us",
          homeSlowPeriodUs
        );

        homeSetState(HOME_SEEK_SLOW);

        if (!startMotor(DIR_HOME, homeSlowPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      break;

    case HOME_SEEK_FAST:
      if (HomeSwitch.isActive(estadoHome)) {
        Motor.stop();
        homeVerifyStartMs = millis();
        homeSetState(HOME_VERIFY_FAST);

        Messaging.send_event_value(
          "home",
          "verify_fast_contact",
          "debounce_ms",
          homeDebounceMs
        );
      }
      else if (homePhaseSteps >= (long)homeSeekMaxSteps) {
        homeError("seek_fast_step_limit");
      }
      break;

    case HOME_VERIFY_FAST:
      if (!HomeSwitch.isActive(estadoHome)) {
        Messaging.send_event("home", "fast_contact_bounce", "continue_seek_fast");
        homeSetState(HOME_SEEK_FAST);

        if (!startMotor(DIR_HOME, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      else if ((millis() - homeVerifyStartMs) >= homeDebounceMs) {
        Messaging.send_event("home", "fast_contact_confirmed", "release_switch");
        homeSetState(HOME_RELEASE);

        if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      break;

    case HOME_SEEK_SLOW:
      if (HomeSwitch.isActive(estadoHome)) {
        Motor.stop();
        homeVerifyStartMs = millis();
        homeSetState(HOME_VERIFY_SLOW);

        Messaging.send_event_value(
          "home",
          "verify_slow_contact",
          "debounce_ms",
          homeDebounceMs
        );
      }
      else if (homePhaseSteps >= HOME_SLOW_MAX_STEPS) {
        homeError("seek_slow_step_limit");
      }
      break;

    case HOME_VERIFY_SLOW:
      if (!HomeSwitch.isActive(estadoHome)) {
        Messaging.send_event("home", "slow_contact_bounce", "continue_seek_slow");
        homeSetState(HOME_SEEK_SLOW);

        if (!startMotor(DIR_HOME, homeSlowPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      else if ((millis() - homeVerifyStartMs) >= homeDebounceMs) {
        homeOk();
      }
      break;

    default:
      homeError("invalid_home_state");
      break;
  }
}

// ============================================================
// Movimiento absoluto en pasos
// ============================================================
void moveError(const char* reason, bool invalidateReference)
{
  Motor.stop();
  st_loop = ST_READY;

  Messaging.send_event("move", "error", reason);

  if (invalidateReference) {
    invalidatePosition("home_required");
  }
}

void moveOk()
{
  Motor.stop();
  st_loop = ST_READY;

  Messaging.send_event_value(
    "move",
    "ok",
    "position_steps",
    (uint32_t)currentPositionSteps
  );
}

bool startAbsoluteMove(long targetSteps)
{
  moveTargetSteps = targetSteps;

  if (moveTargetSteps == currentPositionSteps) {
    Messaging.send_event_value(
      "move",
      "ok",
      "position_steps",
      (uint32_t)currentPositionSteps
    );
    return true;
  }

  moveDirection = (moveTargetSteps > currentPositionSteps) ? DIR_AWAY : DIR_HOME;

  Messaging.send_event_value(
    "move",
    "start",
    "target_steps",
    (uint32_t)moveTargetSteps
  );

  if (!startMotor(moveDirection, Config.get_move_fast_period_us())) {
    moveError("motor_start_failed", false);
    return false;
  }

  st_loop = ST_MOVE_ABS;
  return true;
}

void processAbsoluteMoveSafety()
{
  if (!isMoveActive()) {
    return;
  }

  if (moveDirection == DIR_HOME &&
      currentPositionSteps > 0 &&
      HomeSwitch.isActive()) {
    moveError("home_switch_before_zero", true);
  }
}

void processGeneratedMoveStep()
{
  if (!isMoveActive()) {
    return;
  }

  if (moveDirection == DIR_AWAY) {
    currentPositionSteps++;
  }
  else {
    if (currentPositionSteps <= 0) {
      currentPositionSteps = 0;
      moveError("zero_boundary", true);
      return;
    }

    currentPositionSteps--;
  }

  if (currentPositionSteps == moveTargetSteps) {
    moveOk();
  }
}

// ============================================================
// Log FC_HOME
// ============================================================
void processHomeRawLog()
{
  int estadoHome = HomeSwitch.readRaw();

  if (estadoHome == ultimoEstadoHome) {
    return;
  }

  ultimoEstadoHome = estadoHome;
  Messaging.send_event("fc_home_raw", HomeSwitch.stateText(estadoHome));
}

// ============================================================
// Requests host
// ============================================================
const char* requestCmdName(BeamHostRequest req)
{
  switch (req) {
    case BEAM_REQ_HOME:              return "home";
    case BEAM_REQ_STOP:              return "stop";
    case BEAM_REQ_ENABLE:            return "enable";
    case BEAM_REQ_DISABLE:           return "disable";
    case BEAM_REQ_MOVE_ABS_STEPS:    return "move_abs_steps";
    case BEAM_REQ_READ_LOADS:        return "read_loads";
    case BEAM_REQ_APPLY_FORCE:       return "apply_force";
    case BEAM_REQ_MODE_MANUAL:             return "set_mode";
    case BEAM_REQ_MODE_CALIBRATION_CELL:   return "set_mode";
    case BEAM_REQ_MODE_CALIBRATION_MOTOR:  return "set_mode";
    case BEAM_REQ_MODE_EXPERIMENT:         return "set_mode";
    case BEAM_REQ_FACTORY_RESET:     return "factory_reset";
    default:                         return "none";
  }
}

void processPendingRequest()
{
  if (!Config.has_request()) {
    return;
  }

  BeamHostRequest req = Config.get_request();
  long requestedSteps = Config.get_requested_steps();
  uint8_t requestedCell = Config.get_requested_cell();
  double requestedForceG = Config.get_requested_force_g();
  Config.clear_request();

  // Durante HOME / MOVE se permite seguir leyendo las celdas.
  // STOP y DISABLE tambien pueden interrumpir siempre.
  if (isPhysicalOperationActive() &&
      req != BEAM_REQ_STOP &&
      req != BEAM_REQ_DISABLE &&
      req != BEAM_REQ_READ_LOADS) {
    Messaging.send_command_result(
      requestCmdName(req),
      "rejected",
      "operation_in_progress"
    );
    return;
  }

  switch (req) {

    case BEAM_REQ_STOP:
      if (isHomeActive()) {
        homeAbort("stop_command", false);
      }
      else if (isMoveActive()) {
        Motor.stop();
        st_loop = ST_READY;
        Messaging.send_event_value(
          "move",
          "aborted",
          "position_steps",
          (uint32_t)currentPositionSteps
        );
      }
      else {
        Motor.stop();
        st_loop = ST_READY;
      }

      Messaging.send_command_result("stop", "accepted");
      break;

    case BEAM_REQ_ENABLE:
      Motor.enable();
      Messaging.send_command_result("enable", "accepted");
      break;

    case BEAM_REQ_DISABLE:
      if (isHomeActive()) {
        homeAbort("disable_command", true);
      }
      else {
        Motor.disable();
        st_loop = ST_READY;

        if (homed) {
          invalidatePosition("driver_disabled");
        }
      }

      Messaging.send_command_result("disable", "accepted");
      break;

    case BEAM_REQ_HOME:
      startHoming();
      Messaging.send_command_result("home", "accepted");
      break;

    case BEAM_REQ_MOVE_ABS_STEPS:
      if (!homed) {
        Messaging.send_command_result(
          "move_abs_steps",
          "rejected",
          "home_required"
        );
        return;
      }

      if (requestedSteps < 0) {
        Messaging.send_command_result(
          "move_abs_steps",
          "rejected",
          "negative_target"
        );
        return;
      }

      if (requestedSteps > 0 && Config.get_max_position_steps() == 0UL) {
        Messaging.send_command_result(
          "move_abs_steps",
          "rejected",
          "max_position_not_configured"
        );
        return;
      }

      if ((uint32_t)requestedSteps > Config.get_max_position_steps()) {
        Messaging.send_command_result(
          "move_abs_steps",
          "rejected",
          "target_out_of_range"
        );
        return;
      }

      if (!startAbsoluteMove(requestedSteps)) {
        Messaging.send_command_result(
          "move_abs_steps",
          "rejected",
          "motor_start_failed"
        );
        return;
      }

      Messaging.send_command_result("move_abs_steps", "accepted");
      break;

    case BEAM_REQ_READ_LOADS:
      processLoadCells();

      if (!LoadCell1.isCalibrated() || !LoadCell2.isCalibrated()) {
        Messaging.send_command_result(
          "read_loads",
          "rejected",
          "load_cells_not_calibrated"
        );
        return;
      }

      if (!sendCurrentLoads("manual")) {
        Messaging.send_command_result(
          "read_loads",
          "rejected",
          "load_data_not_ready"
        );
        return;
      }

      Messaging.send_command_result("read_loads", "accepted");
      break;

    case BEAM_REQ_APPLY_FORCE:
      if (!LoadCell1.isCalibrated() || !LoadCell2.isCalibrated()) {
        Messaging.send_command_result(
          "apply_force",
          "rejected",
          "load_cells_not_calibrated"
        );
        return;
      }

      if (!Config.is_motor_calibrated()) {
        Messaging.send_command_result(
          "apply_force",
          "rejected",
          "motor_not_calibrated"
        );
        return;
      }

      if (!homed || currentPositionSteps != 0) {
        Messaging.send_command_result(
          "apply_force",
          "rejected",
          "home_position_required"
        );
        return;
      }

      if (requestedForceG < MOTOR_APPLY_MIN_TARGET_G ||
          requestedForceG > MOTOR_APPLY_MAX_TARGET_G) {
        Messaging.send_command_result(
          "apply_force",
          "rejected",
          "force_target_out_of_range"
        );
        return;
      }

      if (requestedForceG >= Config.get_motor_max_force_g()) {
        Messaging.send_command_result(
          "apply_force",
          "rejected",
          "force_target_exceeds_safety_limit"
        );
        return;
      }

      Messaging.send_command_result("apply_force", "accepted");
      runApplyForce(requestedForceG);
      break;

    case BEAM_REQ_MODE_MANUAL:
      st_mode = MODE_MANUAL;
      Messaging.send_command_result("set_mode", "accepted");
      Messaging.send_event("mode", "manual");
      break;

    case BEAM_REQ_MODE_CALIBRATION_CELL:
      if (requestedCell != 1 && requestedCell != 2) {
        Messaging.send_command_result("set_mode", "rejected", "invalid_cell");
        return;
      }

      // No se usa isReady() como condicion de entrada: el HX711 puede estar
      // entre conversiones porque MODE_MANUAL acaba de consumir una muestra.
      // CLoadCell::calibrate() espera hasta 2 s una nueva conversion.

      // El modo calibracion es exclusivo y bloqueante.
      // Se detiene y deshabilita el motor por seguridad.
      Motor.disable();
      st_loop = ST_READY;

      if (homed) {
        invalidatePosition("calibration_mode");
      }

      calibrationMode = CAL_LOAD_CELL;
      calibrationCell = requestedCell;
      st_mode = MODE_CALIBRATION;

      Messaging.send_command_result("set_mode", "accepted");
      Messaging.send_event_value(
        "mode",
        "calibration",
        "cell",
        (uint32_t)calibrationCell
      );
      break;

    case BEAM_REQ_MODE_CALIBRATION_MOTOR:
      if (!LoadCell1.isCalibrated() || !LoadCell2.isCalibrated()) {
        Messaging.send_command_result(
          "set_mode",
          "rejected",
          "load_cells_not_calibrated"
        );
        return;
      }

      if (Config.get_max_position_steps() == 0UL) {
        Messaging.send_command_result(
          "set_mode",
          "rejected",
          "max_position_not_configured"
        );
        return;
      }

      Motor.stop();
      st_loop = ST_READY;
      calibrationMode = CAL_MOTOR;
      calibrationCell = 0;
      st_mode = MODE_CALIBRATION;

      Messaging.send_command_result("set_mode", "accepted");
      Messaging.send_event("mode", "calibration", "motor");
      break;

    case BEAM_REQ_MODE_EXPERIMENT:
      // Reservado para la proxima etapa: motor + fuerza + secuencia.
      Messaging.send_command_result("set_mode", "rejected", "not_implemented");
      break;

    case BEAM_REQ_FACTORY_RESET:
      Motor.disable();
      st_loop = ST_READY;
      homed = false;

      Config.factory_reset();
      LoadCell1.clearCalibration();
      LoadCell2.clearCalibration();

      Messaging.send_command_result("factory_reset", "accepted");
      Messaging.send_event("position", "invalid", "home_required");
      break;

    default:
      break;
  }
}

// ============================================================
// Modos de ejecucion
// ============================================================
void processManualMode()
{
  Config.host_cmd();

  // Adquisicion HX711 cooperativa e independiente del movimiento.
  processLoadCells();

  switch (st_loop) {

    case ST_READY:
      processHomeRawLog();
      processPendingRequest();
      break;

    case ST_MOVE_ABS:
      processHomeRawLog();
      processPendingRequest();
      processAbsoluteMoveSafety();
      break;

    case ST_HOME:
      processHomeRawLog();
      processPendingRequest();
      processHoming();
      break;

    default:
      Motor.stop();
      st_loop = ST_INIT_ERROR;
      return;
  }

  // Un request puede haber cambiado el modo.
  if (st_mode != MODE_MANUAL) {
    return;
  }

  // STEP cooperativo despues de evaluar seguridad.
  if (Motor.update()) {
    if (isHomeActive()) {
      homePhaseSteps++;
    }
    else if (isMoveActive()) {
      processGeneratedMoveStep();
    }
  }

  processLoadStream();
}

void processCalibrationMode()
{
  bool ok = false;

  if (calibrationMode == CAL_LOAD_CELL) {
    if (calibrationCell == 1) {
      Messaging.send_event("calibration", "start", "cell1");

      ok = LoadCell1.calibrate();

      if (ok) {
        ok = Config.set_cell1_calibration(
          LoadCell1.getCalibrationA(),
          LoadCell1.getCalibrationB()
        );
      }
    }
    else if (calibrationCell == 2) {
      Messaging.send_event("calibration", "start", "cell2");

      ok = LoadCell2.calibrate();

      if (ok) {
        ok = Config.set_cell2_calibration(
          LoadCell2.getCalibrationA(),
          LoadCell2.getCalibrationB()
        );
      }
    }

    if (ok) {
      Messaging.send_event(
        "calibration",
        "ok",
        calibrationCell == 1 ? "cell1" : "cell2"
      );
    }
    else {
      Messaging.send_event(
        "calibration",
        "error",
        calibrationCell == 1 ? "cell1" :
        (calibrationCell == 2 ? "cell2" : "invalid_cell")
      );
    }
  }
  else if (calibrationMode == CAL_MOTOR) {
    Messaging.send_event("calibration", "start", "motor");
    ok = runMotorCalibration();

    Messaging.send_event(
      "calibration",
      ok ? "ok" : "error",
      "motor"
    );
  }
  else {
    Messaging.send_event("calibration", "error", "invalid_calibration_mode");
  }

  calibrationMode = CAL_NONE;
  calibrationCell = 0;
  st_mode = MODE_MANUAL;
  lastLoadStreamMs = millis();

  Messaging.send_event("mode", "manual", "calibration_finished");
}

void processExperimentMode()
{
  // Seguridad: este modo existe arquitectonicamente pero todavia no se usa.
  Motor.stop();
  Messaging.send_event("experiment", "error", "not_implemented");
  st_mode = MODE_MANUAL;
  Messaging.send_event("mode", "manual", "experiment_not_implemented");
}

// ============================================================
// Inicializacion
// ============================================================
void processInitialization()
{
  switch (st_loop) {

    case ST_INIT:
      Config.set_app_version(FW_VERSION);
      st_mode = MODE_MANUAL;
      calibrationMode = CAL_NONE;
      st_loop = ST_INIT_MOTOR;
      break;

    case ST_INIT_MOTOR:
      // Dejar ENABLE en estado seguro lo antes posible.
      Motor.init();
      st_loop = ST_INIT_HOME_SWITCH;
      break;

    case ST_INIT_HOME_SWITCH:
      HomeSwitch.init();
      ultimoEstadoHome = HomeSwitch.readRaw();
      st_loop = ST_INIT_CONFIG;
      break;

    case ST_INIT_CONFIG:
      if (Config.init()) {
        st_loop = ST_INIT_LOAD_CELLS;
      }
      else {
        Motor.disable();
        Messaging.send_event("boot", "error", "config_init_failed");
        st_loop = ST_INIT_ERROR;
      }
      break;

    case ST_INIT_LOAD_CELLS:
      LoadCell1.init();
      LoadCell2.init();

      // Tiempo de arranque inicial de los HX711. Solo ocurre en boot.
      delay(200);

      LoadCell1.setCalibration(
        Config.get_cell1_a(),
        Config.get_cell1_b(),
        Config.is_cell1_calibrated()
      );

      LoadCell2.setCalibration(
        Config.get_cell2_a(),
        Config.get_cell2_b(),
        Config.is_cell2_calibrated()
      );

      Messaging.send_event(
        "load_cell_1",
        LoadCell1.isReady() ? "ready" : "not_ready",
        LoadCell1.isCalibrated() ? "calibrated" : "not_calibrated"
      );

      Messaging.send_event(
        "load_cell_2",
        LoadCell2.isReady() ? "ready" : "not_ready",
        LoadCell2.isCalibrated() ? "calibrated" : "not_calibrated"
      );

      Messaging.send_event("boot", "ready", FW_VERSION);
      Messaging.send_event("fc_home_initial", HomeSwitch.stateText(ultimoEstadoHome));
      Messaging.send_event("direction", "home", DIR_HOME == HIGH ? "DIR=HIGH" : "DIR=LOW");
      Messaging.send_event("direction", "away", DIR_AWAY == HIGH ? "DIR=HIGH" : "DIR=LOW");
      Messaging.send_event("position", "invalid", "home_required");
      Messaging.send_event("mode", "manual");

      st_loop = ST_READY;
      break;

    case ST_INIT_ERROR:
      // Motor ya queda en estado seguro.
      break;

    default:
      Motor.disable();
      st_loop = ST_INIT_ERROR;
      break;
  }
}

// ============================================================
// Arduino
// ============================================================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  st_loop = ST_INIT;
  st_mode = MODE_MANUAL;
  calibrationMode = CAL_NONE;
}

void loop()
{
  // Hasta ST_READY solo existe la secuencia de inicializacion.
  if (st_loop == ST_INIT ||
      st_loop == ST_INIT_MOTOR ||
      st_loop == ST_INIT_HOME_SWITCH ||
      st_loop == ST_INIT_CONFIG ||
      st_loop == ST_INIT_LOAD_CELLS ||
      st_loop == ST_INIT_ERROR) {
    processInitialization();
    return;
  }

  switch (st_mode) {

    case MODE_MANUAL:
      processManualMode();
      break;

    case MODE_CALIBRATION:
      processCalibrationMode();
      break;

    case MODE_EXPERIMENT:
      processExperimentMode();
      break;

    default:
      Motor.disable();
      st_mode = MODE_MANUAL;
      st_loop = ST_READY;
      Messaging.send_event("mode", "error", "invalid_mode");
      Messaging.send_event("mode", "manual");
      break;
  }
}
