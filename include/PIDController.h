#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define SIMULATION_MODE false // Cambiar a true para activar el modo simulador (sin BLE)


class PIDController {
public:
  PIDController();
  
  // Configura las ganancias y los límites del PID
  void tune(float kp, float ki, float kd, float minOutput, float maxOutput);
  
  // Calcula la salida del PID
  float compute(float setpoint, float processVariable);
  
  // Resetea el estado del controlador (importante al cambiar de modo)
  void reset();

private:
  // --- Ganancias del Controlador ---
  float Kp, Ki, Kd;

  // --- Límites de Salida ---
  float minOutput, maxOutput;

  // --- Variables de Estado ---
  float integralTerm = 0.0;
  float previousProcessVariable = 0.0;
  
  // --- Temporización ---
  unsigned long lastComputeTime = 0;
};

#endif // PID_CONTROLLER_H