#include "home_switch.h"

CHomeSwitch::CHomeSwitch()
{
}

void CHomeSwitch::init()
{
  pinMode(PIN_FC_HOME, INPUT_PULLUP);
}

int CHomeSwitch::readRaw() const
{
  return digitalRead(PIN_FC_HOME);
}

bool CHomeSwitch::isActive() const
{
  return readRaw() == FC_HOME_ACTIVADO;
}

bool CHomeSwitch::isFree() const
{
  return readRaw() == FC_HOME_LIBRE;
}

bool CHomeSwitch::isActive(int rawState) const
{
  return rawState == FC_HOME_ACTIVADO;
}

bool CHomeSwitch::isFree(int rawState) const
{
  return rawState == FC_HOME_LIBRE;
}

const char* CHomeSwitch::stateText() const
{
  return stateText(readRaw());
}

const char* CHomeSwitch::stateText(int rawState) const
{
  return rawState == FC_HOME_ACTIVADO ? "ACTIVADO" : "LIBRE";
}
