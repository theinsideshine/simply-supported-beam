/*
  Viga simplemente apoyada
  ESP32-S3 + NEMA17 + A4988 + FC_HOME

  Firmware modular 0.04.

  Objetivo de esta version:
    - mantener HOME validado de 0.02
    - mantener interfaz host JSON y configuracion NVS
    - agregar posicion logica absoluta en pasos respecto de HOME
    - exigir HOME valido antes de cualquier movimiento de ensayo
    - agregar limite maximo absoluto configurable en NVS
    - agregar periodo de avance rapido configurable en NVS
    - mantener ejecucion cooperativa

  Regla de posicion:
    - HOME OK establece posicion logica = 0
    - no existen posiciones negativas
    - todo MOVE absoluto debe cumplir:
        0 <= target_steps <= max_position_steps

  NO implementa todavia:
    - movimiento por vueltas
    - conversion vueltas -> pasos
    - celdas de carga
    - Wi-Fi / Blynk
    - RMT / timers hardware / tareas dedicadas
*/

// C:\Users\ptavolaro\AppData\Local\arduino\sketches

#include <Arduino.h>

#include "beam_config.h"
#include "a4988_stepper.h"
#include "home_switch.h"

// ============================================================
// Version
// ============================================================
#define FW_VERSION "simply_supported_beam 0.04"
#define SERIAL_BAUD 115200

// ============================================================
// Limites de seguridad de homing
// El limite de busqueda rapida se toma de NVS.
// Los limites de release y busqueda lenta siguen compilados por ahora.
// ============================================================
#define HOME_RELEASE_MAX_STEPS   300L
#define HOME_SLOW_MAX_STEPS      300L

// ============================================================
// FSM principal
// ============================================================
#define ST_INIT             0
#define ST_INIT_MOTOR       1
#define ST_INIT_HOME_SWITCH 2
#define ST_INIT_CONFIG      3
#define ST_READY            4
#define ST_MOVE_ABS         5
#define ST_HOME             7
#define ST_INIT_ERROR       8

uint8_t st_loop = ST_INIT;

// ============================================================
// Subestados de HOME - mismos estados funcionales ya validados
// ============================================================
#define HOME_IDLE             0
#define HOME_RELEASE          1
#define HOME_VERIFY_RELEASE   2
#define HOME_SEEK_FAST        3
#define HOME_VERIFY_FAST      4
#define HOME_BACKOFF          5
#define HOME_SEEK_SLOW        6
#define HOME_VERIFY_SLOW      7

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
// La posicion solo es valida despues de HOME OK.
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

// ============================================================
// Estado RAW de FC_HOME para log de cambios
// ============================================================
int ultimoEstadoHome = -1;

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
    Config.send_event("motor", "error", "invalid_step_period");
    return false;
  }

  return true;
}

void invalidatePosition(const char* reason)
{
  homed = false;

  if (reason != nullptr && reason[0] != '\0') {
    Config.send_event("position", "invalid", reason);
  }
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

  Config.send_event("home", "aborted", reason);
}

void homeError(const char* reason)
{
  Motor.stop();
  homeState = HOME_IDLE;
  homePhaseSteps = 0;
  st_loop = ST_READY;
  homed = false;

  Config.send_event("home", "error", reason);
}

void homeOk()
{
  Motor.stop();
  homeState = HOME_IDLE;
  homePhaseSteps = 0;
  st_loop = ST_READY;

  currentPositionSteps = 0;
  homed = true;

  Config.send_event_value("home", "ok", "reference_step", 0UL);
}

void startHoming()
{
  // Desde que comienza un nuevo HOME, la referencia anterior deja de ser valida.
  homed = false;

  homeStepPeriodUs = Config.get_step_period_us();
  homeSlowPeriodUs = Config.get_home_slow_period_us();
  homeBackoffSteps = Config.get_home_backoff_steps();
  homeDebounceMs = Config.get_home_debounce_ms();
  homeSeekMaxSteps = Config.get_home_seek_max_steps();

  Config.send_event("home", "start");
  Config.send_event("home_direction", "toward_switch", DIR_HOME == HIGH ? "DIR=HIGH" : "DIR=LOW");
  Config.send_event("home_direction", "away_from_switch", DIR_AWAY == HIGH ? "DIR=HIGH" : "DIR=LOW");
  Config.send_event("home_initial_switch", HomeSwitch.stateText());

  st_loop = ST_HOME;

  if (HomeSwitch.isActive()) {
    Config.send_event("home", "release_start");
    homeSetState(HOME_RELEASE);

    if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
      homeError("motor_start_failed");
    }
  }
  else {
    Config.send_event("home", "seek_fast_start");
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

        Config.send_event_value(
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
        Config.send_event("home", "release_bounce", "continue_release");
        homeSetState(HOME_RELEASE);

        if (!startMotor(DIR_AWAY, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      else if ((millis() - homeVerifyStartMs) >= homeDebounceMs) {
        Config.send_event_value(
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

        Config.send_event_value(
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

        Config.send_event_value(
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
        Config.send_event("home", "fast_contact_bounce", "continue_seek_fast");
        homeSetState(HOME_SEEK_FAST);

        if (!startMotor(DIR_HOME, homeStepPeriodUs)) {
          homeError("motor_start_failed");
        }
      }
      else if ((millis() - homeVerifyStartMs) >= homeDebounceMs) {
        Config.send_event("home", "fast_contact_confirmed", "release_switch");
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

        Config.send_event_value(
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
        Config.send_event("home", "slow_contact_bounce", "continue_seek_slow");
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

  Config.send_event("move", "error", reason);

  if (invalidateReference) {
    invalidatePosition("home_required");
  }
}

void moveOk()
{
  Motor.stop();
  st_loop = ST_READY;

  Config.send_event_value(
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
    Config.send_event_value(
      "move",
      "ok",
      "position_steps",
      (uint32_t)currentPositionSteps
    );
    return true;
  }

  moveDirection = (moveTargetSteps > currentPositionSteps) ? DIR_AWAY : DIR_HOME;

  Config.send_event_value(
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

  // Si durante un MOVE hacia HOME el switch aparece antes de llegar
  // logicamente a cero, la posicion por pasos ya no es confiable.
  // Se detiene sobre RAW inmediatamente por seguridad y se exige HOME nuevo.
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
    // Salvaguarda logica adicional: nunca permitir posicion negativa.
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
  Config.send_event("fc_home_raw", HomeSwitch.stateText(estadoHome));
}

// ============================================================
// Requests host
// ============================================================
const char* requestCmdName(BeamHostRequest req)
{
  switch (req) {
    case BEAM_REQ_HOME:            return "home";
    case BEAM_REQ_STOP:            return "stop";
    case BEAM_REQ_ENABLE:          return "enable";
    case BEAM_REQ_DISABLE:         return "disable";
    case BEAM_REQ_MOVE_ABS_STEPS:  return "move_abs_steps";
    default:                       return "none";
  }
}

void processPendingRequest()
{
  if (!Config.has_request()) {
    return;
  }

  BeamHostRequest req = Config.get_request();
  long requestedSteps = Config.get_requested_steps();
  Config.clear_request();

  // Una sola operacion fisica por vez.
  // STOP y DISABLE son siempre capaces de interrumpir.
  if (isPhysicalOperationActive() &&
      req != BEAM_REQ_STOP &&
      req != BEAM_REQ_DISABLE) {
    Config.send_command_result(
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
        Config.send_event_value(
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

      Config.send_command_result("stop", "accepted");
      break;

    case BEAM_REQ_ENABLE:
      Motor.enable();
      Config.send_command_result("enable", "accepted");
      break;

    case BEAM_REQ_DISABLE:
      if (isHomeActive()) {
        homeAbort("disable_command", true);
      }
      else {
        Motor.disable();
        st_loop = ST_READY;

        // Con el driver deshabilitado la mecanica puede desplazarse sin
        // que el firmware cuente pasos. La referencia deja de ser confiable.
        if (homed) {
          invalidatePosition("driver_disabled");
        }
      }

      Config.send_command_result("disable", "accepted");
      break;

    case BEAM_REQ_HOME:
      startHoming();
      Config.send_command_result("home", "accepted");
      break;

    case BEAM_REQ_MOVE_ABS_STEPS:
      // Regla principal del ensayo: ningun movimiento sin HOME valido.
      if (!homed) {
        Config.send_command_result(
          "move_abs_steps",
          "rejected",
          "home_required"
        );
        return;
      }

      // El cero es un limite logico absoluto.
      if (requestedSteps < 0) {
        Config.send_command_result(
          "move_abs_steps",
          "rejected",
          "negative_target"
        );
        return;
      }

      // max_position_steps = 0 significa aun no configurado.
      // Se permite target 0 porque nunca aleja el mecanismo de HOME.
      if (requestedSteps > 0 && Config.get_max_position_steps() == 0UL) {
        Config.send_command_result(
          "move_abs_steps",
          "rejected",
          "max_position_not_configured"
        );
        return;
      }

      if ((uint32_t)requestedSteps > Config.get_max_position_steps()) {
        Config.send_command_result(
          "move_abs_steps",
          "rejected",
          "target_out_of_range"
        );
        return;
      }

      if (!startAbsoluteMove(requestedSteps)) {
        Config.send_command_result(
          "move_abs_steps",
          "rejected",
          "motor_start_failed"
        );
        return;
      }

      Config.send_command_result("move_abs_steps", "accepted");
      break;

    default:
      break;
  }
}

// ============================================================
// Arduino
// ============================================================
void setup()
{
  // Intencionalmente minimo, siguiendo la filosofia del Monaco Meter.
  Serial.begin(SERIAL_BAUD);
  st_loop = ST_INIT;
}

void loop()
{
  // El host se habilita una vez terminada la inicializacion.
  if (st_loop >= ST_READY && st_loop != ST_INIT_ERROR) {
    Config.host_cmd();
  }

  // Inicializacion y operacion secuencial por FSM.
  switch (st_loop) {

    case ST_INIT:
      Config.set_app_version(FW_VERSION);
      st_loop = ST_INIT_MOTOR;
      break;

    case ST_INIT_MOTOR:
      // El A4988 es el primer hardware inicializado para dejar ENABLE
      // en estado seguro (deshabilitado) cuanto antes.
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
        Config.send_event("boot", "ready", FW_VERSION);
        Config.send_event("fc_home_initial", HomeSwitch.stateText(ultimoEstadoHome));
        Config.send_event("direction", "home", DIR_HOME == HIGH ? "DIR=HIGH" : "DIR=LOW");
        Config.send_event("direction", "away", DIR_AWAY == HIGH ? "DIR=HIGH" : "DIR=LOW");
        Config.send_event("position", "invalid", "home_required");
        st_loop = ST_READY;
      }
      else {
        Motor.disable();
        Config.send_event("boot", "error", "config_init_failed");
        st_loop = ST_INIT_ERROR;
      }
      break;

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

    case ST_INIT_ERROR:
      // El A4988 ya quedo deshabilitado.
      break;

    default:
      Motor.stop();
      st_loop = ST_INIT_ERROR;
      break;
  }

  // Generacion STEP cooperativa DESPUES de evaluar HOME / seguridad.
  if (Motor.update()) {
    if (isHomeActive()) {
      homePhaseSteps++;
    }
    else if (isMoveActive()) {
      processGeneratedMoveStep();
    }
  }
}
