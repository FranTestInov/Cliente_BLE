#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController() {
  // El constructor puede estar vacío
}

void PIDController::tune(float kp, float ki, float kd, float minOut, float maxOut) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  minOutput = minOut;
  maxOutput = maxOut;
}

void PIDController::reset() {
  integralTerm = 0.0;
  previousProcessVariable = 0.0;
  lastComputeTime = millis();
}

float PIDController::compute(float setpoint, float processVariable) {
  unsigned long now = millis();
  float timeChange = (float)(now - lastComputeTime);

  // Si no ha pasado tiempo, no calculamos nada para evitar división por cero
  if (timeChange <= 0) return maxOutput; // O un valor por defecto

  // --- Cálculo de los 3 Términos ---
  float error = setpoint - processVariable;

  // Término Proporcional (P)
  float p_term = Kp * error;

  // Término Integral (I) - Se acumula con el tiempo
  integralTerm += Ki * error * timeChange;

  // Término Derivativo (D) - Cambio en la variable de proceso
  float d_term = 0.0;
  if (timeChange > 0) {
    d_term = Kd * (processVariable - previousProcessVariable) / timeChange;
  }
  
  // --- Suma de los Términos para obtener la salida "ideal" ---
  float output = p_term + integralTerm - d_term; // Restamos D porque actúa sobre la PV

  // --- Aplicamos los límites (saturación) ---
  float clampedOutput = output;
  if (clampedOutput > maxOutput) {
    clampedOutput = maxOutput;
  } else if (clampedOutput < minOutput) {
    clampedOutput = minOutput;
  }
  
  // --- ¡AQUÍ LA MAGIA! Anti-Windup del Documento 1 ---
  // "Descargamos" el integrador si la salida fue limitada
  integralTerm += (clampedOutput - output);

  // --- Actualizamos las variables para el próximo ciclo ---
  previousProcessVariable = processVariable;
  lastComputeTime = now;

  return clampedOutput;
}