/**
 * @file CommunicationManager.cpp
 * @brief Implementación de la clase CommunicationManager para la gestión de comunicación BLE y comandos serie.
 * 
 * Este archivo contiene la implementación de la clase CommunicationManager, que se encarga de gestionar la conexión BLE
 * con el servidor, la lectura de sensores remotos, el manejo de comandos recibidos por el puerto serie y el envío de datos
 * de estado al PC.
 */

#include "CommunicationManager.h"

// --- UUIDs y otras definiciones de BLE ---
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TMP   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_PRES  "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC_UUID_HUM   "d2b2d3e1-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_CO2   "a1b2c3d4-5678-90ab-cdef-1234567890ab"
#define CHARACTERISTIC_UUID_CALIBRATE "12345678-1234-1234-1234-123456789abc"

// --- Variables estáticas para BLE (privadas a este archivo) ---
static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicTemp = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicPres = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicHum  = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicCO2  = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristicCalibrate = nullptr;

static bool deviceFound = false;
static String serverAddress = "";

/**
 * @brief Callback para dispositivos BLE anunciados durante el escaneo.
 * 
 * Esta clase se utiliza para detectar el servidor BLE con el UUID de servicio esperado.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /**
   * @brief Método llamado cuando se encuentra un dispositivo BLE durante el escaneo.
   * @param advertisedDevice Referencia al dispositivo BLE anunciado.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().toString() == SERVICE_UUID) {
      Serial.println("Dispositivo BLE encontrado!");
      serverAddress = String(advertisedDevice.getAddress().toString().c_str());
      deviceFound = true;
      advertisedDevice.getScan()->stop();
    }
  }
};

// --- Implementación de la Clase ---

/**
 * @brief Constructor de CommunicationManager.
 * @param execManager Referencia al ExecutionManager para la gestión de procesos.
 */
CommunicationManager::CommunicationManager(ExecutionManager& execManager) : executionManager(execManager) {}

/**
 * @brief Inicializa el módulo de comunicación.
 * 
 * Configura el puerto serie, inicializa BLE y comienza el escaneo del servidor.
 */
void CommunicationManager::init() {
  Serial.begin(115200); // Usamos una velocidad más alta
  BLEDevice::init("ESP32_BLE_Client");
  scanForServer();
  Serial.println("Communication Manager inicializado.");
}

/**
 * @brief Bucle principal de la comunicación.
 * 
 * Gestiona la conexión BLE, lee sensores remotos, maneja comandos serie y envía el estado al PC.
 */
void CommunicationManager::run() {
  // 1. Gestionar conexión BLE
  if (!isConnected && deviceFound) {
    connectToServer();
  }

  // 2. Leer sensores periódicamente si estamos conectados
  if (isConnected && millis() - lastSensorReadTime > 1000) {
    lastSensorReadTime = millis();
    readAllSensors();
  }

  // 3. Revisar si hay comandos nuevos desde el PC
  handleSerialCommands();

  // 4. Enviar siempre el estado actual y los datos al PC
  sendStatusToPC();
}

/**
 * @brief Maneja los comandos recibidos por el puerto serie.
 * 
 * Interpreta y ejecuta comandos como SET_CO2, CALIBRATE_SENSOR y OPEN_ALL.
 */
void CommunicationManager::handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Comando recibido: ");
    Serial.println(command);

    if (command.startsWith("SET_CO2")) {
      // Extraemos el valor entre paréntesis
      int value = command.substring(command.indexOf('(') + 1, command.indexOf(')')).toInt();
      executionManager.startSetpointProcess(value);

    } else if (command == "CALIBRATE_SENSOR") {
      executionManager.startCalibrationProcess();

    } else if (command == "OPEN_ALL") {
      executionManager.triggerPanicMode();
    
    } else {
      Serial.println("ERROR: Comando no reconocido");
    }
  }
}

/**
 * @brief Envía el estado actual y los datos de sensores al PC por el puerto serie.
 * 
 * El formato es: "STATUS:[estado];TEMP:[val];HUM:[val];PRES:[val];CO2:[val]\n"
 */
void CommunicationManager::sendStatusToPC() {
  // Formato: "STATUS:[estado];TEMP:[val];HUM:[val];PRES:[val];CO2:[val]\n"
  // Este formato es fácil de procesar en la aplicación de Python
  String statusStr = "STATUS:" + String(executionManager.getCurrentState()) + ";";
  statusStr += "TEMP:" + String(lastSensorData.temperature, 2) + ";";
  statusStr += "HUM:" + String(lastSensorData.humidity, 2) + ";";
  statusStr += "PRES:" + String(lastSensorData.pressure, 2) + ";";
  statusStr += "CO2:" + String(lastSensorData.co2);
  
  // Usamos un temporizador para no saturar el puerto serie
  static unsigned long lastStatusSendTime = 0;
  if (millis() - lastStatusSendTime > 500) { // Enviamos estado 2 veces por segundo
    lastStatusSendTime = millis();
    Serial.println(statusStr);
  }
}

// --- Métodos de BLE ---

/**
 * @brief Inicia el escaneo para encontrar el servidor BLE.
 */
void CommunicationManager::scanForServer() {
  Serial.println("Buscando servidor BLE...");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->start(5, false); // Escaneo de 5 segundos
}

/**
 * @brief Conecta al servidor BLE y obtiene las características remotas.
 */
void CommunicationManager::connectToServer() {
  Serial.println("Conectando al servidor BLE...");
  pClient = BLEDevice::createClient();
  
  if (pClient->connect(BLEAddress(serverAddress.c_str()))) {
    Serial.println("Conectado al servidor BLE!");
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService) {
      pRemoteCharacteristicTemp = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_TMP);
      pRemoteCharacteristicPres = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_PRES);
      pRemoteCharacteristicHum  = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_HUM);
      pRemoteCharacteristicCO2  = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CO2);
      pRemoteCharacteristicCalibrate = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CALIBRATE);
      isConnected = true;
    } else {
      Serial.println("ERROR: No se pudo encontrar el servicio en el servidor.");
      pClient->disconnect();
    }
  } else {
    Serial.println("ERROR: No se pudo conectar al servidor BLE.");
  }
}

/**
 * @brief Lee los valores de los sensores remotos a través de BLE.
 * 
 * Actualiza la estructura lastSensorData con los valores leídos.
 */
void CommunicationManager::readAllSensors() {
    if (!isConnected) return;

    if(pRemoteCharacteristicTemp) lastSensorData.temperature = String(pRemoteCharacteristicTemp->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicPres) lastSensorData.pressure = String(pRemoteCharacteristicPres->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicHum) lastSensorData.humidity = String(pRemoteCharacteristicHum->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicCO2) lastSensorData.co2 = String(pRemoteCharacteristicCO2->readValue().c_str()).toInt();
}