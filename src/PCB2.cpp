#include <Arduino.h>
#include "CommunicationManager.h"
#include "ExecutionManager.h"

// --- OBJETOS GLOBALES DE LOS Módulos ---
ExecutionManager executionManager;
CommunicationManager communicationManager(executionManager); // Le pasamos el executionManager

void setup() {
  // Espera a que el puerto serial se conecte para no perder mensajes iniciales
  //while (!Serial); 
  delay(2000);

  // Inicializamos cada uno de nuestros managers
  executionManager.init();
  communicationManager.init();
  
  Serial.println("Sistema PCB2 inicializado y listo.");
}

void loop() {
  // Simplemente ejecutamos los bucles de nuestros managers
  communicationManager.run();
  executionManager.run();
}