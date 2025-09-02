/**
 * @file ExecutionManager.cpp
 * @brief Implementación de la clase ExecutionManager para la gestión de procesos de control y válvulas.
 *
 * Este archivo contiene la implementación de la clase ExecutionManager, que se encarga de manejar la máquina de estados
 * principal del sistema, controlar las válvulas y ejecutar los procesos de setpoint, calibración y modo pánico.
 */

#include "ExecutionManager.h"
#include <Arduino.h>

#include "CommunicationManager.h" // Necesitaremos leer los datos del sensor
// Necesitamos una forma de acceder a los datos de los sensores.
// Haremos que el CommunicationManager sea accesible globalmente (una simplificación por ahora).
extern CommunicationManager communicationManager;

/**
 * @brief Constructor de ExecutionManager.
 *
 * Inicializa el estado del sistema en IDLE.
 */
ExecutionManager::ExecutionManager()
{
  currentState = IDLE;
  setpointState = MEASURING;
  pulseState = IDEL;
  setpoint = 0;
};

/**
 * @brief Inicializa los pines de las válvulas y asegura que todas comiencen cerradas.
 */
void ExecutionManager::init()
{
  pinMode(VALVE_CO2_PIN, OUTPUT);
  pinMode(VALVE_AIR_PIN, OUTPUT);
  pinMode(VALVE_VACUUM_PIN, OUTPUT);

  // Asegurarse de que todas las válvulas comiencen cerradas (suponiendo lógica activa en alto)
  digitalWrite(VALVE_CO2_PIN, LOW);
  digitalWrite(VALVE_AIR_PIN, LOW);
  digitalWrite(VALVE_VACUUM_PIN, LOW);

  // --- Sintonizamos el PID con valores iniciales ---
  // Estos valores Kp, Ki, Kd se deben ajustar experimentalmente
  // Salida de 0 a 100 (representando 0% a 100% de tiempo de apertura de válvula)
  pidController.tune(2.0, 0.5, 0.1, 0, 100);

  Serial.println("Execution Manager inicializado.");
}

/**
 * @brief Ejecuta la máquina de estados principal y el ciclo de control.
 */
void ExecutionManager::run()
{
  switch (currentState)
  {
  case IDLE:
    digitalWrite(VALVE_CO2_PIN, LOW);
    digitalWrite(VALVE_AIR_PIN, LOW);
    break;

  case EXECUTING_SETPOINT:
  {
    unsigned long now = millis();

    // --- Sub-Máquina de Estados del Ciclo de Control ---
    switch (setpointState)
    {

    case MEASURING:
    {
      // Estamos esperando a que el gas se estabilice.
      if (now - lastCycleTime >= STABILIZATION_TIME_MS)
      {
        // Se cumplió el tiempo de estabilización, ahora inicia otro proceso de acción.
        setpointState = CALCULATING;
      }
    }
    break;

    case CALCULATING:
    {
      // Leemos el valor estabilizado del sensor.
      float currentCO2 = communicationManager.getLastServerData().co2;

      // Calculamos la nueva salida del PID (0-100) y la guardamos.
      lastPidOutput = pidController.compute(setpoint, currentCO2);

      // Informamos por serial para depuración.
      Serial.printf("Setpoint: %d, Actual: %.0f, PID Salida: %.2f%%\n", setpoint, currentCO2, lastPidOutput);

      // Pasamos a la fase de actuación e iniciamos su temporizador.
      lastCycleTime = now;
      setpointState = ACTUATING;
      break;
    }

    case ACTUATING:
    {
      // Estamos en la ventana de tiempo para actuar sobre las válvulas.

      // Calculamos cuánto tiempo de este ciclo la válvula debe estar abierta.
      long dutyCycleTime = (ACTUATION_TIME_MS * lastPidOutput) / 100;

      if (now - lastCycleTime < dutyCycleTime)
      {
        digitalWrite(VALVE_CO2_PIN, HIGH); // Abrimos válvula de CO2
      }
      else
      {
        digitalWrite(VALVE_CO2_PIN, LOW); // Cerramos válvula de CO2
        float currentCO2 = communicationManager.getLastServerData().co2;
        if (currentCO2 >= setpoint)
        {
          digitalWrite(VALVE_AIR_PIN, HIGH);
        }
      }

      // Verificamos si el ciclo de actuación ha terminado.
      if (now - lastCycleTime >= ACTUATION_TIME_MS)
      {
        // El ciclo de actuación terminó, cerramos la válvula por si acaso
        // y pasamos a la fase de estabilización.
        digitalWrite(VALVE_CO2_PIN, LOW);
        digitalWrite(VALVE_AIR_PIN, LOW);
        lastCycleTime = now;
        setpointState = MEASURING;
      }
      break;
    }
    }
    break;
  }

  // ... (otros casos como PANIC_MODE se mantienen igual) ...
  case EXECUTING_CALIBRATION:
  {
    break;
  }

  case PULSE:
  {
    switch (pulseState)
    {
    case PULSE_START:
    {
      digitalWrite(VALVE_CO2_PIN, HIGH);
      lastCycleTime = millis();
      pulseState = PULSE_END;
    }
    break;
    case PULSE_END:
    {
      if (millis() - lastCycleTime >= PULSE_CO2)
      {
        digitalWrite(VALVE_CO2_PIN, LOW);
        currentState = IDLE;
        pulseState = IDEL;
      }
    }
    break;
    }
  }
  break;
  case PANIC_MODE:
  {
    // Estado seguro
    digitalWrite(VALVE_CO2_PIN, LOW);
    digitalWrite(VALVE_AIR_PIN, HIGH);
    digitalWrite(VALVE_VACUUM_PIN, HIGH);
    break;
  }
  }
}

// --- Implementación de los Comandos ---

/**
 * @brief Inicia el proceso de setpoint para alcanzar una concentración objetivo de CO2.
 *
 * @param targetConcentration Valor objetivo de concentración de CO2 en ppm.
 */
void ExecutionManager::startSetpointProcess(int targetConcentration)
{
  if (currentState == IDLE)
  {
    Serial.printf("Iniciando proceso de Setpoint a %d ppm\n", targetConcentration);
    setpoint = targetConcentration;

    // Preparamos el controlador para un nuevo proceso.
    pidController.reset();     // Reseteamos el PID
    setpointState = MEASURING; // Empezamos en la fase de medición/estabilización.
    lastCycleTime = millis();  // Iniciamos el primer temporizador.
    currentState = EXECUTING_SETPOINT;
  }
  else
  {
    Serial.println("WARN: No se puede iniciar Setpoint, otro proceso está en curso.");
  }
}

/**
 * @brief Inicia el proceso de calibración del sensor.
 */
void ExecutionManager::startCalibrationProcess()
{
  if (currentState == IDLE)
  {
    Serial.println("Iniciando proceso de Calibración de Sensor.");
    currentState = EXECUTING_CALIBRATION;
  }
  else
  {
    Serial.println("WARN: No se puede iniciar Calibración, otro proceso está en curso.");
  }
}

/**
 * @brief Funcion para generar un puslo de 10ms en la valvula de CO2
 * Este metodo se llamara desde CommunicationManager cuando se reciba el comando "PULSE"
 * No interrumpe ningun proceso en curso
 */
void ExecutionManager::startPulseProcess()
{
  if (currentState == IDLE)
  {
    Serial.println("Iniciando proceso de Pulso de 10ms en la valvula de CO2");
    pulseState = PULSE_START;
    lastCycleTime = millis();
    currentState = PULSE;
  }
  else
  {
    Serial.println("WARN: No se puede iniciar Pulso, otro proceso está en curso.");
  }
}

/**
 * @brief Activa el modo pánico, abriendo todas las válvulas.
 *
 * Este estado es terminal y requiere un reinicio para salir.
 */
void ExecutionManager::triggerPanicMode()
{
  Serial.println("!!! MODO PÁNICO ACTIVADO !!!");
  currentState = PANIC_MODE;
}

/**
 * @brief Obtiene el estado actual del sistema.
 *
 * @return SystemState Estado actual del sistema.
 */
SystemState ExecutionManager::getCurrentState()
{
  return currentState;
}