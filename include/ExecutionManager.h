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

// --- Definición de pines para las válvulas ---
#define VALVE_CO2_PIN    25 ///< Pin para controlar la válvula de CO2.
#define VALVE_AIR_PIN    26 ///< Pin para controlar la válvula de Aire.
#define VALVE_VACUUM_PIN 27 ///< Pin para controlar la válvula de Vacío.

/**
 * @enum SystemState
 * @brief Define los posibles estados de la máquina de estados principal del sistema.
 */
enum SystemState {
  IDLE,                  ///< El sistema está en espera, sin realizar ninguna acción.
  EXECUTING_SETPOINT,    ///< El sistema está ejecutando un proceso de control para alcanzar un setpoint de CO2.
  EXECUTING_CALIBRATION, ///< El sistema está ejecutando una rutina de calibración.
  PANIC_MODE             ///< Estado de emergencia donde se abren todas las válvulas.
};

/**
 * @class ExecutionManager
 * @brief Gestiona la máquina de estados y el control de las válvulas.
 * 
 * Esta clase es el núcleo de la lógica de control. Recibe órdenes para iniciar procesos
 * (como alcanzar un setpoint o calibrar) y gestiona el estado del sistema y los actuadores
 * para llevar a cabo dichas tareas.
 */
class ExecutionManager {
public:
  /**
   * @brief Constructor de ExecutionManager.
   * 
   * Inicializa el estado del sistema en IDLE.
   */
  ExecutionManager();

  /**
   * @brief Inicializa el gestor de ejecución.
   * 
   * Configura los pines de las válvulas como salidas y se asegura de que todas estén cerradas.
   */
  void init();

  /**
   * @brief Bucle principal del gestor de ejecución.
   * 
   * Debe ser llamado repetidamente en el loop principal. Ejecuta la lógica correspondiente
   * al estado actual de la máquina de estados.
   */
  void run();

  /**
   * @brief Inicia el proceso para alcanzar una concentración de CO2 objetivo.
   * @param targetConcentration La concentración de CO2 deseada en ppm.
   */
  void startSetpointProcess(int targetConcentration);

  /**
   * @brief Inicia el proceso de calibración.
   */
  void startCalibrationProcess();

  /**
   * @brief Activa el modo pánico, abriendo todas las válvulas.
   * 
   * Este es un estado de seguridad para despresurizar el sistema rápidamente.
   */
  void triggerPanicMode();

  /**
   * @brief Obtiene el estado actual del sistema.
   * @return SystemState El estado actual de la máquina de estados.
   */
  SystemState getCurrentState();

private:
  SystemState currentState; ///< Almacena el estado actual de la máquina de estados.
};

#endif // EXECUTION_MANAGER_H