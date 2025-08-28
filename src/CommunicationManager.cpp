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

// Callback de BLE (se puede mantener aquí ya que es específico de la implementación)
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
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

CommunicationManager::CommunicationManager(ExecutionManager& execManager) : executionManager(execManager) {}

void CommunicationManager::init() {
  Serial.begin(115200); // Usamos una velocidad más alta
  BLEDevice::init("ESP32_BLE_Client");
  scanForServer();
  Serial.println("Communication Manager inicializado.");
}

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

void CommunicationManager::scanForServer() {
  Serial.println("Buscando servidor BLE...");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->start(5, false); // Escaneo de 5 segundos
}

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

void CommunicationManager::readAllSensors() {
    if (!isConnected) return;

    if(pRemoteCharacteristicTemp) lastSensorData.temperature = String(pRemoteCharacteristicTemp->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicPres) lastSensorData.pressure = String(pRemoteCharacteristicPres->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicHum) lastSensorData.humidity = String(pRemoteCharacteristicHum->readValue().c_str()).toFloat();
    if(pRemoteCharacteristicCO2) lastSensorData.co2 = String(pRemoteCharacteristicCO2->readValue().c_str()).toInt();
}