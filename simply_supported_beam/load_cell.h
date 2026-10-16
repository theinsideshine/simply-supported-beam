#ifndef LOAD_CELL_H
#define LOAD_CELL_H

#include <Arduino.h>
#include "HX711.h"

/*
  CLoadCell
  ------------------------------------------------------------
  Capa de hardware y matematica para una celda de carga + HX711.

  Responsabilidad:
    - inicializar HX711
    - leer RAW
    - promediar lecturas
    - convertir RAW -> gramos / newtons usando:
        peso_g = a * RAW + b
    - ejecutar la calibracion interactiva de 4 puntos
      (0 / 500 / 1000 / 1500 g)

  No conoce NVS / Preferences.
  No mueve motor.
  No decide modos de ejecucion.

  La persistencia de a/b pertenece a CBeamConfig.
*/

#define LOAD_CELL_CAL_POINTS       4
#define LOAD_CELL_CAL_CAPTURE_MS   20000UL
#define LOAD_CELL_GRAVITY          9.80665

class CLoadCell {
public:
  CLoadCell(uint8_t doutPin, uint8_t sckPin);

  void init();

  bool isReady();
  bool isCalibrated() const;

  // Actualizacion cooperativa para MODE_MANUAL / MODE_EXPERIMENT.
  // Si hay una nueva conversion disponible, la consume y actualiza cache.
  bool update();
  bool hasSample() const;
  long getLastRaw() const;
  double getLastGrams() const;
  double getLastNewtons() const;

  long readRaw();
  long readRawAverage(uint8_t samples = 10);

  double readGrams(uint8_t samples = 10);
  double readNewtons(uint8_t samples = 10);

  // Lectura bloqueante robusta para rutinas exclusivas de calibracion.
  // Espera una conversion disponible antes de iniciar el promedio.
  bool readGramsAverageBlocking(double& grams,
                                uint8_t samples = 5,
                                uint32_t timeoutMs = 2500UL);

  void setCalibration(double a, double b, bool valid = true);
  void clearCalibration();

  double getCalibrationA() const;
  double getCalibrationB() const;

  // Calibracion interactiva bloqueante.
  // Se usa exclusivamente dentro de MODE_CALIBRATION.
  // El usuario confirma cada punto escribiendo "ok" por Serial.
  bool calibrate();

private:
  HX711 scale;

  uint8_t pin_dout;
  uint8_t pin_sck;

  double calibration_a;
  double calibration_b;
  bool calibration_valid;

  long last_raw;
  double last_grams;
  bool sample_valid;

  void waitForOk();
  bool captureRaw(unsigned long durationMs,
                  double& average,
                  long& minRaw,
                  long& maxRaw,
                  unsigned long& sampleCount);

  bool calculateCalibration(const double rawCal[LOAD_CELL_CAL_POINTS],
                            double& outA,
                            double& outB) const;
};

#endif // LOAD_CELL_H
