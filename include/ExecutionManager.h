#ifndef EXECUTION_MANAGER_H
#define EXECUTION_MANAGER_H

// --- Definiciones de Pines para las Válvulas ---
const int VALVE_CO2_PIN = 26;
const int VALVE_AIR_PIN = 27;
const int VALVE_VACUUM_PIN = 14;

// --- Estados Principales del Sistema ---
enum SystemState {
  IDLE,
  EXECUTING_SETPOINT,
  EXECUTING_CALIBRATION,
  PANIC_MODE // Estado de emergencia
};

class ExecutionManager {
public:
  ExecutionManager(); // Constructor
  void init();        // Configuración inicial (pines)
  void run();         // Lógica principal que se ejecuta en cada loop

  // --- Comandos que puede recibir del CommunicationManager ---
  void startSetpointProcess(int targetConcentration);
  void startCalibrationProcess();
  void triggerPanicMode();

  // --- Método para obtener el estado actual ---
  SystemState getCurrentState();

private:
  SystemState currentState; // Variable que guarda el estado actual

  void controlValves(); // Lógica interna para manejar las válvulas
};

#endif // EXECUTION_MANAGER_H