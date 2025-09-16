/**
 * @file CommunicationManager.cpp
 * @brief Implementación de la clase CommunicationManager para la gestión de comunicación BLE y comandos serie.
 *
 * Este archivo contiene la implementación de la clase CommunicationManager, que se encarga de gestionar la conexión BLE
 * con el servidor, la lectura de sensores remotos, el manejo de comandos recibidos por el puerto serie y el envío de datos
 * de estado al PC.
 */

#include "CommunicationManager.h"
#include "PIDController.h"
#if !defined(SIMULATION_MODE)
// --- UUIDs y otras definiciones de BLE ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TMP "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_PRES "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC_UUID_HUM "d2b2d3e1-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_CO2 "a1b2c3d4-5678-90ab-cdef-1234567890ab"
#define CHARACTERISTIC_UUID_CALIBRATE "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID_SYSTEM_STATE "c1a7d131-15e1-413f-b565-8123c5a31a1e"
#define CHARACTERISTIC_UUID_COOLER_STATE "d2b8d232-26f1-4688-b7f5-ea07361b26a8"

// --- Variables estáticas para BLE (privadas a este archivo) ---
static BLEClient *pClient = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicTemp = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicPres = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicHum = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicCO2 = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicCalibrate = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicSystemState = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristicCoolerState = nullptr;

static bool deviceFound = false;
static String serverAddress = "";

/**
 * @brief Callback para dispositivos BLE anunciados durante el escaneo.
 *
 * Esta clase se utiliza para detectar el servidor BLE con el UUID de servicio esperado.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
   * @brief Método llamado cuando se encuentra un dispositivo BLE durante el escaneo.
   * @param advertisedDevice Referencia al dispositivo BLE anunciado.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().toString() == SERVICE_UUID)
    {
      Serial.println("Dispositivo BLE encontrado!");
      serverAddress = String(advertisedDevice.getAddress().toString().c_str());
      deviceFound = true;
      advertisedDevice.getScan()->stop();
    }
  }
};

#endif

// --- Implementación de la Clase ---

/**
 * @brief Constructor de CommunicationManager.
 * @param execManager Referencia al ExecutionManager para la gestión de procesos.
 */
CommunicationManager::CommunicationManager(ExecutionManager &execManager) : executionManager(execManager) {}

/**
 * @brief Inicializa el módulo de comunicación.
 *
 * Configura el puerto serie, inicializa BLE y comienza el escaneo del servidor.
 */
void CommunicationManager::init()
{
  Serial.begin(115200); // Usamos una velocidad más alta
#if defined(SIMULATION_MODE)
  Serial.println("**********************************");
  Serial.println("* Communication Manager en MODO SIMULACIÓN *");
  Serial.println("**********************************");
  // En modo simulación, inicializamos los valores base.
  lastServerData.co2 = 450;
  lastServerData.temperature = 25.0;
  lastServerData.humidity = 50.0;
  lastServerData.pressure = 1013.25;
#else
  BLEDevice::init("ESP32_BLE_Client");
#endif
  Serial.println("Communication Manager inicializado.");
}

/**
 * @brief Bucle principal de la comunicación.
 *
 * Gestiona la conexión BLE, lee sensores remotos, maneja comandos serie y envía el estado al PC.
 */
void CommunicationManager::run()
{
#if defined(SIMULATION_MODE)
  // --- LÓGICA DE SIMULACIÓN ---
  if (millis() - lastSensorReadTime > 1000)
  { // Actualizamos datos 1 vez por segundo
    lastSensorReadTime = millis();
    simulateServerData();
  }
#else
  // --- LÓGICA REAL CON BLE ---
  if (!isConnected && !deviceFound && !isScanning)
  {
    scanForServer();
  }
  if (!isConnected && deviceFound)
  {
    connectToServer();
  }
  if (isConnected && millis() - lastSensorReadTime > 1000)
  {
    lastSensorReadTime = millis();
    readServerData();
  }
#endif

  // Esta parte es común para ambos modos
  handleSerialCommands();
  sendStatusToPC();
}

/**
 * @brief Envía el comando para alternar el estado del cooler al servidor BLE.
 * @details Escribe un valor genérico en la característica del cooler para
 * solicitar un cambio de estado en el PCB1.
 */
void CommunicationManager::toggleCooler()
{
#if !defined(SIMULATION_MODE)
  if (isConnected && pRemoteCharacteristicCoolerState)
  {
    Serial.println("Enviando comando para alternar el cooler al servidor...");
    pRemoteCharacteristicCoolerState->writeValue("1", 1);
  }
  else
  {
    Serial.println("WARN: No se puede alternar el cooler, no hay conexión BLE.");
  }
#endif
}
/**
 * @brief Envía el comando de calibración 'START_CAL' al servidor BLE (PCB1).
 * @return bool Devuelve `true` si el comando se envió con éxito, `false` en caso contrario.
 */
bool CommunicationManager::sendCalibrationCommand()
{
#if !defined(SIMULATION_MODE)
  if (isConnected && pRemoteCharacteristicCalibrate)
  {
    Serial.println("Enviando comando 'START_CAL' al servidor...");
    return pRemoteCharacteristicCalibrate->writeValue("START_CAL", true);
  }
  else
  {
    Serial.println("WARN: No se puede enviar comando de calibración, no hay conexión BLE.");
    return false;
  }
#else
  // En modo simulación, siempre tenemos éxito.
  return true;
#endif
}

/**
 * @brief Obtiene la última estructura de datos leída del servidor.
 * @return ServerData Una copia de la última estructura de datos recibida.
 */
ServerData CommunicationManager::getLastServerData()
{
  return lastServerData;
}

#if defined(SIMULATION_MODE)
/**
 * @brief Genera datos de sensores simulados que se comportan de forma realista.
 * @details Esta función se llama periódicamente en el modo de simulación para
 * actualizar los valores de los sensores basándose en el estado actual del
 * ExecutionManager, creando curvas y comportamientos lógicos para la GUI.
 */
void CommunicationManager::simulateServerData()
{
  SystemState currentState = executionManager.getCurrentState();
  int setpoint = executionManager.getSetpoint(); // Necesitarás añadir esta función a ExecutionManager

  // Simular Temperatura, Humedad y Presión con una leve fluctuación
  lastServerData.temperature = 25.0 + (sin(millis() / 5000.0) * 0.5); // Oscila +/- 0.5 grados
  lastServerData.humidity = 50.0 + (cos(millis() / 7000.0) * 2.0);    // Oscila +/- 2%
  lastServerData.pressure = 1013.25 + (sin(millis() / 3000.0) * 1.0); // Oscila +/- 1 hPa

  // Simular el CO2 basado en el estado
  switch (currentState)
  {
  case EXECUTING_SETPOINT:
    // Hacemos que el CO2 se acerque suavemente al setpoint
    lastServerData.co2 += (setpoint - lastServerData.co2) * 0.1;
    lastServerData.systemState = "READY";
    break;
  case PURGING_WITH_AIR:
    // Hacemos que el CO2 baje suavemente a 450 ppm
    lastServerData.co2 += (450 - lastServerData.co2) * 0.1;
    lastServerData.systemState = "READY";
    break;
  case EXECUTING_CALIBRATION:
    lastServerData.systemState = "CALIBRATING";
    break;
  case IDLE:
  case SETPOINT_STABLE:
  default:
    // En reposo, se mantiene cerca de 450 ppm
    if (lastServerData.co2 < 445)
      lastServerData.co2 = 450;
    if (lastServerData.co2 > 455)
      lastServerData.co2 = 450;
    lastServerData.systemState = "READY";
    break;
  }

  // Simular el estado del cooler (puedes hacerlo más complejo si quieres)
  lastServerData.coolerState = "ON";
}
#else

// --- Métodos de BLE ---

/**
 * @brief Inicia el escaneo para encontrar el servidor BLE.
 */
void CommunicationManager::scanForServer()
{
  Serial.println("Buscando servidor BLE...");
  BLEScan *pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->start(5, true); // Escaneo de 5 segundos
  isScanning = true;
}

/**
 * @brief Conecta al servidor BLE y obtiene las características remotas.
 */
void CommunicationManager::connectToServer()
{
  BLEDevice::getScan()->stop();
  isScanning = false;

  Serial.println("Conectando al servidor BLE...");
  pClient = BLEDevice::createClient();

  if (pClient->connect(BLEAddress(serverAddress.c_str())))
  {
    Serial.println("Conectado al servidor BLE!");
    BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService)
    {
      pRemoteCharacteristicTemp = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_TMP);
      pRemoteCharacteristicPres = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_PRES);
      pRemoteCharacteristicHum = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_HUM);
      pRemoteCharacteristicCO2 = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CO2);
      pRemoteCharacteristicCalibrate = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CALIBRATE);
      pRemoteCharacteristicSystemState = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_SYSTEM_STATE);
      pRemoteCharacteristicCoolerState = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_COOLER_STATE);
      isConnected = true;
    }
    else
    {
      Serial.println("ERROR: No se pudo encontrar el servicio en el servidor.");
      deviceFound = false; // Permitimos que el ciclo de escaneo comience de nuevo.
    }
  }
  else
  {
    Serial.println("ERROR: No se pudo conectar al servidor BLE.");
  }
}

/**
 * @brief Lee los valores de los sensores remotos a través de BLE.
 *
 * Actualiza la estructura lastSensorData con los valores leídos.
 */
void CommunicationManager::readServerData()
{
  if (!isConnected)
    return;

  if (pRemoteCharacteristicTemp)
    lastServerData.temperature = String(pRemoteCharacteristicTemp->readValue().c_str()).toFloat();
  if (pRemoteCharacteristicPres)
    lastServerData.pressure = String(pRemoteCharacteristicPres->readValue().c_str()).toFloat();
  if (pRemoteCharacteristicHum)
    lastServerData.humidity = String(pRemoteCharacteristicHum->readValue().c_str()).toFloat();
  if (pRemoteCharacteristicCO2)
    lastServerData.co2 = String(pRemoteCharacteristicCO2->readValue().c_str()).toInt();
  if (pRemoteCharacteristicSystemState)
  {
    // Paso 1: Guardamos el resultado de readValue() en una variable explícita.
    std::string stateValue = pRemoteCharacteristicSystemState->readValue();
    // Paso 2: Ahora convertimos esa variable a String de Arduino.
    lastServerData.systemState = String(stateValue.c_str());
  }
  if (pRemoteCharacteristicCoolerState)
  {
    // Hacemos lo mismo para el estado del cooler.
    std::string coolerValue = pRemoteCharacteristicCoolerState->readValue();
    lastServerData.coolerState = String(coolerValue.c_str());
  }
}

void CommunicationManager::toggleCooler()
{
  if (isConnected && pRemoteCharacteristicCoolerState)
  {
    Serial.println("Enviando comando para alternar el cooler al servidor...");
    // Cualquier escritura al servidor alternará el estado del cooler.
    // Enviamos "1" como un valor genérico.
    pRemoteCharacteristicCoolerState->writeValue("1", 1);
  }
  else
  {
    Serial.println("WARN: No se puede alternar el cooler, no hay conexión BLE.");
  }
}

/**
 * @brief Obtiene la última estructura de datos leída del servidor.
 * @details Este método público permite a otras clases (como ExecutionManager)
 * acceder de forma segura a los datos de los sensores y de estado del PCB1.
 * @return ServerData Una copia de la última estructura de datos recibida.
 */
ServerData CommunicationManager::getLastServerData()
{
  return lastServerData;
}

/**
 * @brief Envía el comando de calibración 'START_CAL' al servidor BLE (PCB1).
 * @details Verifica si la conexión está activa y la característica de calibración es válida
 * antes de escribir el comando.
 * @return bool Devuelve `true` si el comando se envió con éxito, `false` en caso contrario.
 */
bool CommunicationManager::sendCalibrationCommand()
{
  if (isConnected && pRemoteCharacteristicCalibrate)
  {
    Serial.println("Enviando comando 'START_CAL' al servidor...");
    // Corregido para usar el comando que el servidor espera.
    pRemoteCharacteristicCalibrate->writeValue("START_CAL", true);
    return true;
  }
  else
  {
    Serial.println("WARN: No se puede enviar comando de calibración, no hay conexión BLE.");
    return false;
  }
}

#endif
/**
 * @brief Maneja los comandos recibidos por el puerto serie.
 *
 * Interpreta y ejecuta comandos como SET_CO2, CALIBRATE_SENSOR y OPEN_ALL.
 */
void CommunicationManager::handleSerialCommands()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.print("Comando recibido: ");
    Serial.println(command);

    if (command.startsWith("SET_CO2"))
    {
      // Extraemos el valor entre paréntesis
      int value = command.substring(command.indexOf('(') + 1, command.indexOf(')')).toInt();
      executionManager.startSetpointProcess(value);
    }
    else if (command == "CALIBRATE_SENSOR")
    {
      executionManager.startCalibrationProcess();
    }
    // Aca
    else if (command.startsWith("PULSE"))
    {
      int value = command.substring(command.indexOf('(') + 1, command.indexOf(')')).toInt();
      executionManager.startPulseProcess(value);
    }
    else if (command == "TOGGLE_COOLER")
    {
      toggleCooler();
    }
    else if (command == "OPEN_ALL")
    {
      executionManager.triggerPanicMode();
    }
    else
    {
      Serial.println("ERROR: Comando no reconocido");
    }
  }
}

/**
 * @brief Envía el estado actual y los datos de sensores al PC por el puerto serie.
 *
 * El formato es: "STATUS:[estado];TEMP:[val];HUM:[val];PRES:[val];CO2:[val]\n"
 */
void CommunicationManager::sendStatusToPC()
{
  // Formato: "STATUS:[estado];TEMP:[val];HUM:[val];PRES:[val];CO2:[val]\n"
  // Este formato es fácil de procesar en la aplicación de Python
  String statusStr = "PCB2_STATE:" + String(executionManager.getCurrentState()) + ";";
  statusStr += "TEMP:" + String(lastServerData.temperature, 2) + ";";
  statusStr += "HUM:" + String(lastServerData.humidity, 2) + ";";
  statusStr += "PRES:" + String(lastServerData.pressure, 2) + ";";
  statusStr += "CO2:" + String(lastServerData.co2) + ";";
  statusStr += "PCB1_STATE:" + lastServerData.systemState + ";";
  statusStr += "COOLER:" + lastServerData.coolerState;

  // Usamos un temporizador para no saturar el puerto serie
  static unsigned long lastStatusSendTime = 0;
  if (millis() - lastStatusSendTime > serialdatatimer)
  { // Enviamos estado 2 veces por segundo
    lastStatusSendTime = millis();
    Serial.println(statusStr);
  }
}
