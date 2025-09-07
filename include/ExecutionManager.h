/**
 * @file ExecutionManager.h
 * @brief Definición de la clase ExecutionManager para la gestión de la lógica de control y actuadores.
 *
 * Este archivo define la interfaz de la clase ExecutionManager, que actúa como el "cerebro" del sistema.
 * Implementa una máquina de estados para gestionar los diferentes modos de operación (IDLE, Setpoint, Calibración, Pánico)
 * y controla directamente los actuadores (válvulas).
 */

#ifndef EXECUTION_MANAGER_H
#define EXECUTION_MANAGER_H

#include <Arduino.h>
#include "PIDController.h"

// --- Definición de pines para las válvulas ---
#define VALVE_CO2_PIN 25    ///< Pin para controlar la válvula de CO2.
#define VALVE_AIR_PIN 26    ///< Pin para controlar la válvula de Aire.
#define VALVE_VACUUM_PIN 27 ///< Pin para controlar la válvula de Vacío.

/**
 * @enum SystemState
 * @brief Define los posibles estados de la máquina de estados principal del sistema.
 */
enum SystemState
{
  IDLE,                  ///< El sistema está en espera, sin realizar ninguna acción.
  EXECUTING_SETPOINT,    ///< El sistema está ejecutando un proceso de control para alcanzar un setpoint de CO2.
  SETPOINT_STABLE,       ///< El sistema ha alcanzado y estabilizado el setpoint de CO2.
  EXECUTING_CALIBRATION, ///< El sistema está ejecutando una rutina de calibración.
  PULSE,                 ///< El sistema está generando un pulso en la válvula de CO2.
  PANIC_MODE             ///< Estado de emergencia donde se abren todas las válvulas.
};

/**
 * @enum SetpointSubState
 * @brief Define los estados internos del proceso de control de setpoint.
 */
enum SetpointSubState
{
  MEASURING,   ///< El sistema está esperando que el sensor se estabilice para tomar una nueva medida.
  CALCULATING, ///< El sistema calcula la nueva salida del PID.
  ACTUATING    ///< El sistema aplica la salida del PID a las válvulas durante un ciclo de PWM.
};

/**
 * @enum PulseState
 * @brief Define los estados para generar un puslo de 10 ms
 */

enum PulseState
{
  IDEL,
  PULSE_START,
  PULSE_END
};

/**
 * @class ExecutionManager
 * @brief Gestiona la máquina de estados y el control de las válvulas.
 *
 * Esta clase es el núcleo de la lógica de control. Recibe órdenes para iniciar procesos
 * (como alcanzar un setpoint o calibrar) y gestiona el estado del sistema y los actuadores
 * para llevar a cabo dichas tareas.
 */
class ExecutionManager
{
public:
  // Son todos metodos de acceso publico
  ExecutionManager();
  void init();
  void run();
  void startSetpointProcess(int targetConcentration);
  void startCalibrationProcess();
  void startPulseProcess(int durationMs);
  void triggerPanicMode();
  SystemState getCurrentState(); // Metodo que devuelve un objeto del tipo SystemState

private:
  // Atributos de control
  int setpoint;                                  // Para guardar el setpoint del proceso actual
  PIDController pidController;                   // Para guardar el setpoint del proceso actual
  const float SETPOINT_DEADBAND_PPM = 50.0;      ///< Banda de tolerancia alrededor del setpoint.
  unsigned long stableStartTime;                 ///< Marca de tiempo de cuándo se alcanzó la estabilidad.
  const unsigned long STABLE_TIMEOUT_MS = 60000; ///< 1 minuto para considerar el proceso finalizado.
  unsigned long PULSE_CO2 = 100;                 // Duración del pulso de 10ms en la electrovalvula de CO2

  // Maquinas de estado
  SystemState currentState;       // Almacena el estado actual de la máquina de estados.
  SetpointSubState setpointState; ///< Estado actual del ciclo de control PID.
  PulseState pulseState;          //< Estado actual del ciclo de pulso de 10ms

  // Tiempos
  unsigned long lastCycleTime;                       ///< Marca de tiempo para el inicio de cada fase.
  float lastPidOutput;                               ///< Almacena la última salida del PID para usarla durante la fase de actuación.
  const unsigned long STABILIZATION_TIME_MS = 55000; ///< (T_estabilizacion) Tiempo de espera para que la mezcla se homogeneice (55s).
  const unsigned long ACTUATION_TIME_MS = 200;       ///< (T_ciclo) Duración total del ciclo de actuación de las válvulas (200ms).

  // El tiempo de muestreo (h) será implícitamente la suma de estos dos.
};

#endif // EXECUTION_MANAGER_H