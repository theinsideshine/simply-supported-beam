#ifndef HOME_SWITCH_H
#define HOME_SWITCH_H

#include <Arduino.h>

/*
  CHomeSwitch
  ------------------------------------------------------------
  Capa minima del fin de carrera HOME.

  Responsabilidad:
    - concentrar pin y logica electrica del FC_HOME
    - inicializar INPUT_PULLUP
    - entregar lectura RAW
    - informar ACTIVO / LIBRE

  No hace debounce, no ejecuta homing y no toma decisiones fisicas.
  El debounce y toda la secuencia HOME pertenecen a la FSM principal.
*/

// ------------------------------------------------------------
// Hardware FC_HOME validado
// ------------------------------------------------------------
#define PIN_FC_HOME 13

#define FC_HOME_LIBRE     LOW
#define FC_HOME_ACTIVADO  HIGH

class CHomeSwitch {
public:
  CHomeSwitch();

  void init();

  int readRaw() const;
  bool isActive() const;
  bool isFree() const;

  bool isActive(int rawState) const;
  bool isFree(int rawState) const;

  const char* stateText() const;
  const char* stateText(int rawState) const;
};

#endif // HOME_SWITCH_H
