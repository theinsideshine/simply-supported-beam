#ifndef BEAM_SYSTEM_H
#define BEAM_SYSTEM_H

#include <Arduino.h>

/*
  beam_system.h
  ------------------------------------------------------------
  Definiciones globales de compilacion del firmware.

  Responsabilidad:
    - version / baudrate del firmware
    - asignacion de pines que pertenecen al sistema completo
    - estados globales de la FSM principal
    - modos de ejecucion
    - parametros compilados de las rutinas de calibracion
    - limites compilados de seguridad de HOME

  Regla de arquitectura:
    - los pines internos de una clase permanecen en su propio .h
      (A4988, FC_HOME, etc.)
    - los pines de instancias multiples creadas por el .ino, como las
      dos celdas de carga, se definen aca
    - no contiene logica ni accede a NVS
*/

// ============================================================
// Firmware
// ============================================================
#define FW_VERSION  "simply_supported_beam 0.09"
#define SERIAL_BAUD 115200

// ============================================================
// Celdas de carga / HX711
// ============================================================
// Celda 1 = apoyo fijo
// Celda 2 = apoyo movil
#define PIN_CELL1_DOUT 5
#define PIN_CELL1_SCK  6
#define PIN_CELL2_DOUT 7
#define PIN_CELL2_SCK  8

// ============================================================
// Limites compilados de seguridad de HOME
// ============================================================
#define HOME_RELEASE_MAX_STEPS 300L
#define HOME_SLOW_MAX_STEPS    300L

// ============================================================
// Calibracion motor-viga
// ============================================================
// Relacion mecanica provisional ya medida en el banco.
#define MOTOR_CAL_STEPS_PER_MM_ESTIMATE        25.0

// Se usa la luz ingresada por el operador solo para aproximarse.
// La deteccion real de contacto siempre se hace por fuerza.
#define MOTOR_CAL_APPROACH_MARGIN_STEPS           5L
#define MOTOR_CAL_COARSE_STEP_CHUNK               5L
#define MOTOR_CAL_FINE_STEP_CHUNK                 1L
#define MOTOR_CAL_PRECONTACT_MARGIN_STEPS        10L

// Primer criterio de contacto. Se podra refinar despues de caracterizar
// ruido y repetibilidad con el banco completo.
#define MOTOR_CAL_CONTACT_THRESHOLD_G            20.0

// Promedio / espera entre movimientos discretos y lectura de fuerza.
#define MOTOR_CAL_SETTLE_MS                      300UL
#define MOTOR_CAL_AVG_SAMPLES                      5U
#define MOTOR_CAL_READ_TIMEOUT_MS                2500UL

// Puntos usados para caracterizar repetibilidad del conjunto.
#define MOTOR_CAL_TARGET_500_G                   500.0
#define MOTOR_CAL_TARGET_1000_G                 1000.0
#define MOTOR_CAL_TARGET_1500_G                 1500.0
#define MOTOR_CAL_REPETITIONS                       3U

// Estrategia de aproximacion a cada fuerza objetivo.
#define MOTOR_CAL_FORCE_COARSE_MARGIN_G          200.0
#define MOTOR_CAL_FORCE_MEDIUM_MARGIN_G           60.0
#define MOTOR_CAL_FORCE_COARSE_STEPS                5L
#define MOTOR_CAL_FORCE_MEDIUM_STEPS                2L
#define MOTOR_CAL_FORCE_FINE_STEPS                  1L

// Comando manual apply_force.
// La aproximacion rapida se hace respecto del contacto REAL guardado
// por la calibracion motor-viga, nunca respecto del paso estimado del
// objetivo de fuerza. Desde antes del contacto manda la fuerza medida.
#define MOTOR_APPLY_MIN_TARGET_G                    50.0
#define MOTOR_APPLY_MAX_TARGET_G                  1500.0
#define MOTOR_APPLY_PRECONTACT_MARGIN_STEPS           5L
#define MOTOR_APPLY_APPROACH_CHUNK_STEPS              5L

// ============================================================
// FSM principal de bajo nivel
// ============================================================
enum BeamLoopState : uint8_t {
  ST_INIT = 0,
  ST_INIT_MOTOR,
  ST_INIT_HOME_SWITCH,
  ST_INIT_CONFIG,
  ST_INIT_LOAD_CELLS,
  ST_READY,
  ST_MOVE_ABS,
  ST_HOME,
  ST_INIT_ERROR
};

// ============================================================
// Modos de ejecucion
// ============================================================
enum BeamMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_CALIBRATION,
  MODE_EXPERIMENT
};

// ============================================================
// Tipo de calibracion activo
// ============================================================
enum BeamCalibrationMode : uint8_t {
  CAL_NONE = 0,
  CAL_LOAD_CELL,
  CAL_MOTOR
};

// ============================================================
// Subestados de HOME
// ============================================================
enum BeamHomeState : uint8_t {
  HOME_IDLE = 0,
  HOME_RELEASE,
  HOME_VERIFY_RELEASE,
  HOME_SEEK_FAST,
  HOME_VERIFY_FAST,
  HOME_BACKOFF,
  HOME_SEEK_SLOW,
  HOME_VERIFY_SLOW
};

#endif // BEAM_SYSTEM_H
