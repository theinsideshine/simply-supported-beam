#include "a4988_stepper.h"

CA4988Stepper::CA4988Stepper()
{
  current_period_us = 0;
  last_step_us = 0;

  direction_level = LOW;
  enabled = false;
  moving = false;
}

void CA4988Stepper::init()
{
  pinMode(PIN_A4988_EN, OUTPUT);
  pinMode(PIN_A4988_STEP, OUTPUT);
  pinMode(PIN_A4988_DIR, OUTPUT);

  digitalWrite(PIN_A4988_STEP, LOW);
  digitalWrite(PIN_A4988_DIR, LOW);
  digitalWrite(PIN_A4988_EN, A4988_DISABLE);

  direction_level = LOW;
  enabled = false;
  moving = false;
  current_period_us = 0;
  last_step_us = micros();
}

void CA4988Stepper::enable()
{
  digitalWrite(PIN_A4988_EN, A4988_ENABLE);
  enabled = true;
}

void CA4988Stepper::disable()
{
  stop();
  digitalWrite(PIN_A4988_EN, A4988_DISABLE);
  enabled = false;
}

bool CA4988Stepper::isEnabled() const
{
  return enabled;
}

void CA4988Stepper::setDirection(uint8_t directionLevel)
{
  direction_level = directionLevel;
  digitalWrite(PIN_A4988_DIR, direction_level);
}

uint8_t CA4988Stepper::getDirection() const
{
  return direction_level;
}

bool CA4988Stepper::start(uint8_t directionLevel, uint32_t periodUs)
{
  if (periodUs == 0) {
    stop();
    return false;
  }

  // Misma secuencia electrica de la baseline 0.04:
  // enable -> DIR -> 10 us -> temporizacion STEP.
  enable();
  setDirection(directionLevel);

  // Bloqueo intencional y acotado, aceptado para esta etapa.
  delayMicroseconds(10);

  current_period_us = periodUs;
  last_step_us = micros();
  moving = true;

  return true;
}

void CA4988Stepper::stop()
{
  moving = false;
  digitalWrite(PIN_A4988_STEP, LOW);
}

bool CA4988Stepper::isMoving() const
{
  return moving;
}

bool CA4988Stepper::isBusy() const
{
  return moving;
}

uint32_t CA4988Stepper::getPeriodUs() const
{
  return current_period_us;
}

bool CA4988Stepper::update()
{
  if (!moving || !enabled) {
    return false;
  }

  uint32_t now = micros();

  // Igual que la baseline: como maximo un STEP por pasada de loop().
  // No se intenta recuperar pasos atrasados.
  if ((uint32_t)(now - last_step_us) < current_period_us) {
    return false;
  }

  last_step_us = now;

  digitalWrite(PIN_A4988_STEP, HIGH);

  // Bloqueo intencional y acotado de 4 us en la baseline actual.
  delayMicroseconds(STEP_PULSE_US);

  digitalWrite(PIN_A4988_STEP, LOW);

  return true;
}
