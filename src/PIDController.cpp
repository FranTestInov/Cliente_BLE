#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController()
{
  // El constructor puede estar vacío
}

void PIDController::tune(float kp, float ki, float kd, float minOut, float maxOut)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
  minOutput = minOut;
  maxOutput = maxOut;
}

void PIDController::reset()
{
  lastComputeTime = millis();
  integralTerm = 0.0; // Resetea el termino integral
  previousProcessVariable = 0.0;
  previousOutput = 0.0;
}

float PIDController::compute(float setpoint, float processVariable)
{
  unsigned long now = millis();
  float timeChange = (float)(now - lastComputeTime); // Tiempo de cambio

  float dt_sec = timeChange / 1000.0f;

  // if (dt_sec <= 0.001f)
  // {
  //   Serial.printf("Salgo porque recien se reseteo el PID o dt muy pequeño: %f\n", dt_sec);
  //   return 0.0;
  // }

  // --- Cálculo de los 3 Términos ---
  float error = setpoint - processVariable;

  // Término Proporcional (P)
  float p_term = Kp * error;

  // Término Integral (I) - Se acumula con el tiempo
  integralTerm += Ki * error * dt_sec;

  // Término Derivativo (D) - Cambio en la variable de proceso
  float d_term = 0.0;
  if (dt_sec > 0.0f)
  {
    d_term = Kd * (processVariable - previousProcessVariable) / dt_sec;
  }

  // --- Suma de los Términos para obtener la salida "ideal" ---
  float output = p_term + integralTerm - d_term; // Restamos D porque actúa sobre la PV

  // Serial.printf("output: %f\n", output);

  // --- Aplicamos los límites (saturación - 100 ; +100) ---
  float clampedOutput = output;
  if (clampedOutput > maxOutput)
  {
    clampedOutput = maxOutput;
  }
  else if (clampedOutput < minOutput)
  {
    clampedOutput = minOutput;
  }

  // Serial.printf("clampedOutput: %f", clampedOutput);
  //  "Descargamos" el integrador si la salida fue limitada
  integralTerm += (clampedOutput - output);

  // --- Actualizamos las variables para el próximo ciclo ---
  previousProcessVariable = processVariable;

  if (previousProcessVariable == 0.0)
  {
    previousProcessVariable = processVariable;
  }

  lastComputeTime = now;          // Guarda la ultima ejecución del PID
  previousOutput = clampedOutput; // Guarda la ultima salida del

  return clampedOutput;
}