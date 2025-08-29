/**
 * @file ExecutionManager.cpp
 * @brief Implementación de la clase ExecutionManager para la gestión de procesos de control y válvulas.
 * 
 * Este archivo contiene la implementación de la clase ExecutionManager, que se encarga de manejar la máquina de estados
 * principal del sistema, controlar las válvulas y ejecutar los procesos de setpoint, calibración y modo pánico.
 */

#include "ExecutionManager.h"
#include <Arduino.h>

/**
 * @brief Constructor de ExecutionManager.
 * 
 * Inicializa el estado del sistema en IDLE.
 */
ExecutionManager::ExecutionManager() {
  currentState = IDLE;
}

/**
 * @brief Inicializa los pines de las válvulas y asegura que todas comiencen cerradas.
 */
void ExecutionManager::init() {
  pinMode(VALVE_CO2_PIN, OUTPUT);
  pinMode(VALVE_AIR_PIN, OUTPUT);
  pinMode(VALVE_VACUUM_PIN, OUTPUT);

  // Asegurarse de que todas las válvulas comiencen cerradas (suponiendo lógica activa en alto)
  digitalWrite(VALVE_CO2_PIN, LOW);
  digitalWrite(VALVE_AIR_PIN, LOW);
  digitalWrite(VALVE_VACUUM_PIN, LOW);

  Serial.println("Execution Manager inicializado.");
}

/**
 * @brief Ejecuta la máquina de estados principal del sistema.
 * 
 * Según el estado actual, ejecuta la lógica correspondiente para cada proceso.
 */
void ExecutionManager::run() {
  // --- Máquina de Estados Principal ---
  switch (currentState) {
    case IDLE:
      // No hacer nada, esperar a que un comando inicie un proceso.
      break;

    case EXECUTING_SETPOINT:
      // Aquí iría la lógica del PID o el ciclo MIDE -> ACTUA -> COMPARA
      // Por ahora, solo informamos que estamos en este estado.
      // Ejemplo: controlValves();
      break;

    case EXECUTING_CALIBRATION:
      // Aquí iría la secuencia de calibración (abrir válvula de aire, esperar, etc.)
      break;
      
    case PANIC_MODE:
      // Lógica de pánico: Abrir todas las válvulas para despresurizar.
      digitalWrite(VALVE_CO2_PIN, HIGH);
      digitalWrite(VALVE_AIR_PIN, HIGH);
      digitalWrite(VALVE_VACUUM_PIN, HIGH);
      // Nota: Este estado es terminal, requeriría un reinicio para salir.
      break;
  }
}

// --- Implementación de los Comandos ---

/**
 * @brief Inicia el proceso de setpoint para alcanzar una concentración objetivo de CO2.
 * 
 * @param targetConcentration Valor objetivo de concentración de CO2 en ppm.
 */
void ExecutionManager::startSetpointProcess(int targetConcentration) {
  if (currentState == IDLE) {
    Serial.printf("Iniciando proceso de Setpoint a %d ppm\n", targetConcentration);
    currentState = EXECUTING_SETPOINT;
    // Aquí inicializarías las variables para tu PID o controlador.
  } else {
    Serial.println("WARN: No se puede iniciar Setpoint, otro proceso está en curso.");
  }
}

/**
 * @brief Inicia el proceso de calibración del sensor.
 */
void ExecutionManager::startCalibrationProcess() {
  if (currentState == IDLE) {
    Serial.println("Iniciando proceso de Calibración de Sensor.");
    currentState = EXECUTING_CALIBRATION;
  } else {
    Serial.println("WARN: No se puede iniciar Calibración, otro proceso está en curso.");
  }
}

/**
 * @brief Activa el modo pánico, abriendo todas las válvulas.
 * 
 * Este estado es terminal y requiere un reinicio para salir.
 */
void ExecutionManager::triggerPanicMode() {
  Serial.println("!!! MODO PÁNICO ACTIVADO !!!");
  currentState = PANIC_MODE;
}

/**
 * @brief Obtiene el estado actual del sistema.
 * 
 * @return SystemState Estado actual del sistema.
 */
SystemState ExecutionManager::getCurrentState() {
  return currentState;
}