#include "load_cell.h"
#include <math.h>
#include <limits.h>

static const double LOAD_CELL_CAL_MASS_G[LOAD_CELL_CAL_POINTS] = {
  0.0,
  500.0,
  1000.0,
  1500.0
};

CLoadCell::CLoadCell(uint8_t doutPin, uint8_t sckPin)
{
  pin_dout = doutPin;
  pin_sck = sckPin;

  calibration_a = 0.0;
  calibration_b = 0.0;
  calibration_valid = false;

  last_raw = 0;
  last_grams = NAN;
  sample_valid = false;
}

void CLoadCell::init()
{
  scale.begin(pin_dout, pin_sck);
}

bool CLoadCell::isReady()
{
  return scale.is_ready();
}

bool CLoadCell::isCalibrated() const
{
  return calibration_valid;
}

bool CLoadCell::update()
{
  if (!scale.is_ready()) {
    return false;
  }

  last_raw = scale.read();
  sample_valid = true;

  if (calibration_valid) {
    last_grams = calibration_a * (double)last_raw + calibration_b;
  }
  else {
    last_grams = NAN;
  }

  return true;
}

bool CLoadCell::hasSample() const
{
  return sample_valid;
}

long CLoadCell::getLastRaw() const
{
  return last_raw;
}

double CLoadCell::getLastGrams() const
{
  if (!sample_valid || !calibration_valid) {
    return NAN;
  }

  return last_grams;
}

double CLoadCell::getLastNewtons() const
{
  double grams = getLastGrams();

  if (!isfinite(grams)) {
    return NAN;
  }

  return (grams / 1000.0) * LOAD_CELL_GRAVITY;
}

long CLoadCell::readRaw()
{
  return scale.read();
}

long CLoadCell::readRawAverage(uint8_t samples)
{
  if (samples == 0) {
    samples = 1;
  }

  return scale.read_average(samples);
}

double CLoadCell::readGrams(uint8_t samples)
{
  if (!calibration_valid || !scale.is_ready()) {
    return NAN;
  }

  long raw = readRawAverage(samples);
  return calibration_a * (double)raw + calibration_b;
}

double CLoadCell::readNewtons(uint8_t samples)
{
  double grams = readGrams(samples);

  if (!isfinite(grams)) {
    return NAN;
  }

  return (grams / 1000.0) * LOAD_CELL_GRAVITY;
}

bool CLoadCell::readGramsAverageBlocking(double& grams,
                                         uint8_t samples,
                                         uint32_t timeoutMs)
{
  grams = NAN;

  if (!calibration_valid) {
    return false;
  }

  if (samples == 0) {
    samples = 1;
  }

  uint32_t start = millis();
  while (!scale.is_ready()) {
    if ((uint32_t)(millis() - start) >= timeoutMs) {
      return false;
    }

    delay(1);
  }

  long raw = scale.read_average(samples);
  double value = calibration_a * (double)raw + calibration_b;

  if (!isfinite(value)) {
    return false;
  }

  last_raw = raw;
  last_grams = value;
  sample_valid = true;
  grams = value;

  return true;
}

void CLoadCell::setCalibration(double a, double b, bool valid)
{
  calibration_a = a;
  calibration_b = b;
  calibration_valid = valid && isfinite(a) && isfinite(b) && fabs(a) > 1.0e-12;

  if (sample_valid && calibration_valid) {
    last_grams = calibration_a * (double)last_raw + calibration_b;
  }
}

void CLoadCell::clearCalibration()
{
  calibration_a = 0.0;
  calibration_b = 0.0;
  calibration_valid = false;
  last_grams = NAN;
}

double CLoadCell::getCalibrationA() const
{
  return calibration_a;
}

double CLoadCell::getCalibrationB() const
{
  return calibration_b;
}

void CLoadCell::waitForOk()
{
  // Limpiar CR/LF o texto residual antes de esperar la confirmacion.
  while (Serial.available()) {
    Serial.read();
  }

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

bool CLoadCell::captureRaw(unsigned long durationMs,
                           double& average,
                           long& minRaw,
                           long& maxRaw,
                           unsigned long& sampleCount)
{
  unsigned long startTime = millis();
  int64_t sum = 0;

  sampleCount = 0;
  minRaw = LONG_MAX;
  maxRaw = LONG_MIN;

  while ((millis() - startTime) < durationMs) {
    if (scale.is_ready()) {
      long raw = scale.read();

      sum += raw;
      sampleCount++;

      if (raw < minRaw) {
        minRaw = raw;
      }

      if (raw > maxRaw) {
        maxRaw = raw;
      }
    }

    delay(1);
  }

  if (sampleCount == 0) {
    average = 0.0;
    return false;
  }

  average = (double)sum / (double)sampleCount;
  return true;
}

bool CLoadCell::calculateCalibration(const double rawCal[LOAD_CELL_CAL_POINTS],
                                     double& outA,
                                     double& outB) const
{
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXX = 0.0;
  double sumXY = 0.0;

  for (int i = 0; i < LOAD_CELL_CAL_POINTS; i++) {
    double x = rawCal[i];
    double y = LOAD_CELL_CAL_MASS_G[i];

    sumX += x;
    sumY += y;
    sumXX += x * x;
    sumXY += x * y;
  }

  double n = (double)LOAD_CELL_CAL_POINTS;
  double denominator = n * sumXX - sumX * sumX;

  if (!isfinite(denominator) || fabs(denominator) < 1.0e-12) {
    return false;
  }

  outA = (n * sumXY - sumX * sumY) / denominator;
  outB = (sumY - outA * sumX) / n;

  return isfinite(outA) && isfinite(outB) && fabs(outA) > 1.0e-12;
}

bool CLoadCell::calibrate()
{
  // is_ready() refleja si hay una conversion disponible en este instante.
  // Como MODE_MANUAL puede haber consumido la muestra inmediatamente antes
  // de entrar a calibracion, se espera brevemente una nueva conversion.
  uint32_t readyStart = millis();
  while (!scale.is_ready() && (millis() - readyStart) < 2000UL) {
    delay(1);
  }

  if (!scale.is_ready()) {
    Serial.println();
    Serial.println("ERROR: HX711 no disponible.");
    return false;
  }

  double rawCal[LOAD_CELL_CAL_POINTS];

  Serial.println();
  Serial.println("======================================");
  Serial.println("       CALIBRACION HX711");
  Serial.println("======================================");
  Serial.println();
  Serial.println("Se tomaran cuatro puntos:");
  Serial.println("0 g");
  Serial.println("500 g");
  Serial.println("1000 g");
  Serial.println("1500 g");
  Serial.println();
  Serial.println("Cada punto se mide durante 20 segundos.");
  Serial.println();
  Serial.println("IMPORTANTE:");
  Serial.println("El punto 0 g corresponde al conjunto");
  Serial.println("mecanico completo SIN carga externa.");

  for (int i = 0; i < LOAD_CELL_CAL_POINTS; i++) {
    Serial.println();
    Serial.println("--------------------------------------");

    if (LOAD_CELL_CAL_MASS_G[i] == 0.0) {
      Serial.println("Deje el conjunto SIN PESAS.");
      Serial.println("Este sera el punto de 0 g.");
    }
    else {
      Serial.print("Coloque ");
      Serial.print(LOAD_CELL_CAL_MASS_G[i], 0);
      Serial.println(" g.");
    }

    Serial.println();
    Serial.println("Espere unos segundos a que se estabilice.");
    Serial.println("Luego escriba:");
    Serial.println();
    Serial.println("ok");
    Serial.println();

    waitForOk();

    Serial.println();
    Serial.println("Midiendo durante 20 segundos...");

    long minRaw = 0;
    long maxRaw = 0;
    unsigned long sampleCount = 0;

    if (!captureRaw(LOAD_CELL_CAL_CAPTURE_MS,
                    rawCal[i],
                    minRaw,
                    maxRaw,
                    sampleCount)) {
      Serial.println("ERROR: no se recibieron muestras del HX711.");
      return false;
    }

    Serial.println();
    Serial.print("Cantidad muestras : ");
    Serial.println(sampleCount);

    Serial.print("RAW promedio      : ");
    Serial.println(rawCal[i], 2);

    Serial.print("RAW minimo        : ");
    Serial.println(minRaw);

    Serial.print("RAW maximo        : ");
    Serial.println(maxRaw);

    Serial.print("Rango RAW         : ");
    Serial.println(maxRaw - minRaw);

    Serial.println();
    Serial.print("GUARDADO: ");
    Serial.print(LOAD_CELL_CAL_MASS_G[i], 0);
    Serial.print(" g  -> RAW = ");
    Serial.println(rawCal[i], 2);
  }

  double newA = 0.0;
  double newB = 0.0;

  if (!calculateCalibration(rawCal, newA, newB)) {
    Serial.println();
    Serial.println("ERROR: no se pudo calcular la recta de calibracion.");
    return false;
  }

  // La calibracion anterior solo se reemplaza si todo termino correctamente.
  calibration_a = newA;
  calibration_b = newB;
  calibration_valid = true;

  Serial.println();
  Serial.println("======================================");
  Serial.println("       CALIBRACION CALCULADA");
  Serial.println("======================================");
  Serial.println();

  Serial.print("a = ");
  Serial.println(calibration_a, 10);

  Serial.print("b = ");
  Serial.println(calibration_b, 4);

  Serial.println();
  Serial.println("ECUACION:");
  Serial.println();
  Serial.print("Peso[g] = ");
  Serial.print(calibration_a, 10);
  Serial.print(" * RAW ");

  if (calibration_b >= 0.0) {
    Serial.print("+ ");
  }

  Serial.println(calibration_b, 4);

  Serial.println();
  Serial.println("--------------------------------------");
  Serial.println("COMPROBACION");
  Serial.println("--------------------------------------");

  for (int i = 0; i < LOAD_CELL_CAL_POINTS; i++) {
    double calculated = calibration_a * rawCal[i] + calibration_b;
    double error = calculated - LOAD_CELL_CAL_MASS_G[i];

    Serial.print("Patron: ");
    Serial.print(LOAD_CELL_CAL_MASS_G[i], 0);
    Serial.print(" g");

    Serial.print("   Calculado: ");
    Serial.print(calculated, 2);
    Serial.print(" g");

    Serial.print("   Error: ");
    Serial.print(error, 2);
    Serial.println(" g");
  }

  Serial.println("======================================");
  return true;
}
