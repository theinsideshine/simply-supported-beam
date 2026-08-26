#ifndef A4988_STEPPER_H
#define A4988_STEPPER_H

#include <Arduino.h>

/*
  CA4988Stepper
  ------------------------------------------------------------
  Capa minima de primitivas para A4988.

  Responsabilidad:
    - concentrar pines y logica electrica del A4988
    - inicializar el driver
    - enable / disable
    - direccion
    - start / stop
    - generacion cooperativa de STEP con micros()

  No conoce FC_HOME, no ejecuta homing y no decide secuencias.

  NOTA DE ESTA ETAPA:
    Se mantienen los delayMicroseconds() cortos heredados de la baseline:
      - 10 us despues de cambiar DIR
      - STEP_PULSE_US para el pulso HIGH

    Son bloqueos acotados de pocos microsegundos y se aceptan
    intencionalmente en esta primera version modular. Si mas adelante se
    migra el generador STEP a RMT u otro periferico, esta implementacion
    interna podra reemplazarse sin cambiar la FSM.
*/

// ------------------------------------------------------------
// Hardware A4988 validado
// ------------------------------------------------------------
#define PIN_A4988_EN    2
#define PIN_A4988_STEP  38
#define PIN_A4988_DIR   37

#define A4988_ENABLE    LOW
#define A4988_DISABLE   HIGH

// Sentidos validados con el mecanismo montado.
// HOME debe subir; AWAY debe bajar.
#define DIR_HOME LOW
#define DIR_AWAY HIGH

#define STEP_PULSE_US 4UL

class CA4988Stepper {
public:
  CA4988Stepper();

  void init();

  void enable();
  void disable();
  bool isEnabled() const;

  void setDirection(uint8_t directionLevel);
  uint8_t getDirection() const;

  bool start(uint8_t directionLevel, uint32_t periodUs);
  void stop();

  bool isMoving() const;
  bool isBusy() const;

  uint32_t getPeriodUs() const;

  // Devuelve true solamente cuando en esta llamada se genero un STEP.
  bool update();

private:
  uint32_t current_period_us;
  uint32_t last_step_us;

  uint8_t direction_level;
  bool enabled;
  bool moving;
};

#endif // A4988_STEPPER_H
