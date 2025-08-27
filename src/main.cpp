#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// UUIDs del servicio y las características del servidor BLE
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TMP   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_PRES  "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC_UUID_HUM   "d2b2d3e1-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_CO2   "a1b2c3d4-5678-90ab-cdef-1234567890ab"
#define CHARACTERISTIC_UUID_CALIBRATE "12345678-1234-1234-1234-123456789abc"

BLEClient* pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristicTemp = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristicPres = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristicHum  = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristicCO2  = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristicCalibrate = nullptr;

// --- Variables globales ---
bool enMenu = true;
bool enSensores = false;
bool deviceFound = false;
bool connected = false;
bool calibrationChecked = false;
bool awaitingCalibrationResponse = false;
bool calibrationWaiting = false;
int flag=1;
const int RELAY_PIN = 26; // cualquiera de los recomendados GPIO 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33

unsigned long ultimaLectura = 0;
const unsigned long intervalo = 1000;
unsigned long calibrationStartTime = 0;
const unsigned long calibrationDelay = 20UL * 60UL * 1000UL; // 20 minutos

String serverAddress = "";

// Callback para manejar el escaneo de dispositivos BLE
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().toString() == SERVICE_UUID) {
      Serial.println("Dispositivo encontrado!");
      serverAddress = String(advertisedDevice.getAddress().toString().c_str());
      deviceFound = true;
      advertisedDevice.getScan()->stop();
    }
  }
};

// --- Función para mostrar menú ---
void mostrarMenu() {
  Serial.println("\n===== MENU PRINCIPAL =====");
  Serial.println("1 - Lectura de sensores");
  Serial.println("2 - Calibrar sensor CO₂");
  Serial.println("3 - Encender reles por 100ms");
  Serial.println("Q - Salir de programa (o volver al menu desde sensores)");
  Serial.println("==========================");
  Serial.print("Seleccione opcion: ");
}

void setup() {
  Serial.begin(9600);

  Serial.println("Iniciando cliente BLE...");
  BLEDevice::init("ESP32_BLE_Client");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->start(5);
  
  pinMode(RELAY_PIN, OUTPUT); // declaro a GPIO26 como salida
  digitalWrite(RELAY_PIN, HIGH); // si el módulo es activo en bajo, esto deja el relé apagado
}

void loop() {
  // Intentamos conectar si encontramos el dispositivo
  if (!connected && deviceFound) {
    Serial.println("Conectando al servidor BLE...");
    pClient = BLEDevice::createClient();
    if (pClient->connect(BLEAddress(serverAddress.c_str()))) {
      Serial.println("Conectado al servidor BLE!");
      connected = true;

      BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
      if (pRemoteService) {
        Serial.println("Servicio encontrado!");
        pRemoteCharacteristicTemp = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_TMP);
        pRemoteCharacteristicPres = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_PRES);
        pRemoteCharacteristicHum  = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_HUM);
        pRemoteCharacteristicCO2  = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CO2);
        pRemoteCharacteristicCalibrate  = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_CALIBRATE);
      } else {
        Serial.println("No se pudo encontrar el servicio.");
        pClient->disconnect();
        connected = false;
      }
    } else {
      Serial.println("No se pudo conectar al servidor BLE.");
    }
  }
  if(flag==1){
    mostrarMenu();
    flag=0;
  }
  // ====================== MANEJO DE SERIAL ======================
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    // Si estamos esperando respuesta de calibración
    if (awaitingCalibrationResponse) {
      if (input == "S") {
        Serial.println("Enviando solicitud de calibración al servidor...");
        if (pRemoteCharacteristicCalibrate)
          pRemoteCharacteristicCalibrate->writeValue("D");

        calibrationWaiting = true;
        calibrationStartTime = millis();
        Serial.println("Esperando 20 minutos para que se complete la calibración...");
        awaitingCalibrationResponse = false;
      }
      else if (input == "N") {
        Serial.println("Calibración omitida.");
        awaitingCalibrationResponse = false;
        calibrationChecked = false;
      }
      else {
        Serial.println("Opción inválida. Responda S o N.");
        Serial.println("¿Desea calibrar el sensor CO₂? (S/N): ");
      }
    }
    // Si estamos en el menú principal
    else if (enMenu) {
      if (input == "1") {
        enMenu = false;
        enSensores = true;
        Serial.println("Entrando en modo lectura de sensores...");
        ultimaLectura = millis();
      }
      else if (input == "2") {
        if (connected && !calibrationChecked) {
          Serial.println("¿Desea calibrar el sensor CO₂? (S/N): ");
          calibrationChecked = true;
          awaitingCalibrationResponse = true;
        } else {
          Serial.println("Sensor no conectado o calibración ya realizada.");
        }
      }
      else if (input == "3") {
        Serial.println("Activando reles...");
        digitalWrite(RELAY_PIN, LOW); // enciende el relé por 50ms
        delay(100);
        digitalWrite(RELAY_PIN, HIGH); // apago el relé
        mostrarMenu();
      }
      else if (input == "Q") {
        Serial.println("Saliendo del programa...");
        enMenu = true;
        mostrarMenu();
      }
      else {
        Serial.println("Opción inválida.");
        mostrarMenu();
      }
    }
    // Si estamos en modo sensores
    else if (enSensores) {
      if (input == "Q") {
        enSensores = false;
        enMenu = true;
        Serial.println("\nVolviendo al MENU principal...");
        mostrarMenu();
      }
    }
  }

  // ====================== MANEJO DE CALIBRACIÓN ======================
  if (calibrationWaiting) {
    unsigned long currentTime = millis();
    if (currentTime - calibrationStartTime >= calibrationDelay) {
      Serial.println("Calibración finalizada. Continuando con la lectura normal.");
      calibrationWaiting = false;
    } else {
      static unsigned long lastMsgTime = 0;
      if (currentTime - lastMsgTime > 60000) {
        Serial.println("→ Calibrando... aún en espera.");
        lastMsgTime = currentTime;
      }
      return; // Bloqueamos lecturas de sensores mientras calibramos
    }
  }

  // ====================== LECTURA DE SENSORES ======================
  if (enSensores && connected && !awaitingCalibrationResponse) {
    if (millis() - ultimaLectura >= intervalo) {
      ultimaLectura = millis();

      if (pRemoteCharacteristicTemp) {
        String valueTemp = String(pRemoteCharacteristicTemp->readValue().c_str());
        Serial.print("Temperatura: ");
        Serial.println(valueTemp + "°C");
      }

      if (pRemoteCharacteristicPres) {
        String valuePres = String(pRemoteCharacteristicPres->readValue().c_str());
        Serial.print("Presión: ");
        Serial.println(valuePres + " hPa");
      }

      if (pRemoteCharacteristicHum) {
        String valueHum = String(pRemoteCharacteristicHum->readValue().c_str());
        Serial.print("Humedad: ");
        Serial.println(valueHum + " %");
      }

      if (pRemoteCharacteristicCO2) {
        String valueCO2 = String(pRemoteCharacteristicCO2->readValue().c_str());
        Serial.print("CO₂: ");
        Serial.println(valueCO2 + " ppm");
      }
    }
  }
}
