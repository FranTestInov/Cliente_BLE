#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include "ExecutionManager.h" // Incluimos para poder interactuar con él

// --- Estructura para almacenar los datos de los sensores ---
struct SensorData {
  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  int co2 = 0;
};

class CommunicationManager {
public:
  // Le pasamos una referencia al ExecutionManager para poder enviarle comandos
  CommunicationManager(ExecutionManager& execManager);
  void init();
  void run();

private:
  // --- Atributos de Comunicación ---
  ExecutionManager& executionManager; // Referencia al cerebro
  SensorData lastSensorData;          // Últimos datos leídos de los sensores
  unsigned long lastSensorReadTime = 0;

  // --- Métodos Privados de Gestión ---
  void handleSerialCommands(); // Procesa comandos entrantes del PC
  void sendStatusToPC();       // Envía datos y estado al PC
  
  // --- Métodos y Atributos de BLE ---
  bool isConnected = false;
  void connectToServer();
  void scanForServer();
  void readAllSensors();
};

#endif // COMMUNICATION_MANAGER_H