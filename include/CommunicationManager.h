/**
 * @file CommunicationManager.h
 * @brief Definición de la clase CommunicationManager para la gestión de comunicación BLE y comandos serie.
 * 
 * Este archivo define la interfaz de la clase CommunicationManager, que encapsula toda la lógica
 * de comunicación del dispositivo cliente (PCB2). Esto incluye el escaneo y conexión a un servidor BLE,
 * la lectura de características remotas (sensores), el manejo de comandos desde el puerto serie (PC)
 * y el envío de datos de estado.
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include "ExecutionManager.h" // Incluimos para poder interactuar con él

/**
 * @struct ServerData
 * @brief Estructura para almacenar los datos de los sensores leídos del servidor BLE.
 * 
 * Agrupa todos los valores de los sensores en un único objeto para facilitar su manejo y transmisión.
 */
 
struct ServerData {
  float temperature = 0.0; ///< Temperatura en grados Celsius.
  float humidity = 0.0;    ///< Humedad relativa en porcentaje (%).
  float pressure = 0.0;    ///< Presión atmosférica en hectopascales (hPa).
  int co2 = 0;             ///< Concentración de CO2 en partes por millón (ppm).
  String systemState = "UNKNOWN"; // NUEVO: Para guardar el estado del PCB1
  String coolerState = "UNKNOWN"; // NUEVO: Para guardar el estado del cooler
};

/**
 * @class CommunicationManager
 * @brief Gestiona la comunicación BLE y la interfaz serie.
 * 
 * Esta clase es responsable de toda la comunicación externa del dispositivo. Actúa como cliente BLE
 * para obtener datos de un servidor y se comunica con un PC a través del puerto serie para recibir
 * comandos y enviar telemetría.
 */
class CommunicationManager {
public:
  /**
   * @brief Constructor de CommunicationManager.
   * @param execManager Referencia al ExecutionManager para poder enviarle comandos y consultar su estado.
   */
  CommunicationManager(ExecutionManager& execManager);

  /**
   * @brief Inicializa el gestor de comunicación.
   * 
   * Configura el puerto serie, inicializa el hardware BLE y comienza el escaneo de servidores.
   */
  void init();

  /**
   * @brief Bucle principal del gestor de comunicación.
   * 
   * Debe ser llamado repetidamente en el loop principal. Se encarga de mantener la conexión BLE,
   * leer sensores, procesar comandos serie y enviar datos de estado.
   */
  void run();

  void toggleCooler();



private:
  // --- Atributos de Comunicación ---
  ExecutionManager& executionManager; ///< Referencia al gestor de ejecución para interactuar con la lógica de control.
  ServerData lastServerData;          ///< Almacena los últimos datos leídos de los sensores.
  unsigned long lastSensorReadTime = 0; ///< Marca de tiempo para controlar la frecuencia de lectura de sensores.

  // --- Métodos Privados de Gestión ---
  /**
   * @brief Procesa comandos entrantes desde el PC a través del puerto serie.
   */
  void handleSerialCommands();

  /**
   * @brief Envía el estado actual del sistema y los datos de los sensores al PC.
   */
  void sendStatusToPC();       
  
  // --- Métodos y Atributos de BLE ---
  bool isConnected = false; ///< Flag que indica si el cliente está conectado al servidor BLE.
  bool isScanning = false; // NUEVO: para controlar si estamos escaneando activamente

  /**
   * @brief Intenta conectar con el servidor BLE una vez encontrado.
   */
  void connectToServer();

  /**
   * @brief Inicia el escaneo de dispositivos BLE para encontrar el servidor.
   */
  void scanForServer();

  /**
   * @brief Lee todas las características de los sensores del servidor BLE.
   */
  void readServerData();
};

#endif // COMMUNICATION_MANAGER_H