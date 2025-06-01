// Inclure les bibliothèques
#include <Arduino.h> // Pour les fonctions de base d'Arduino

// --- Définition des broches de l'Arduino Nano IoT 33 ---

// Broches pour les LEDs des feux de véhicules (Rouge, Orange, Vert)
// Chaque ligne représente une zone de circulation (Z0, Z1, Z2, Z3)
const int carLEDs[4][3] = {
  {2, 3, 4},   // Zone 0: Rouge(D2), Orange(D3), Vert(D4)
  {5, 6, 7},   // Zone 1: Rouge(D5), Orange(D6), Vert(D7)
  {8, 9, 10},  // Zone 2: Rouge(D8), Orange(D9), Vert(D10)
  {11, 12, 13} // Zone 3: Rouge(D11), Orange(D12), Vert(D13)
};

// Broches pour les LEDs des feux piétons (Rouge, Vert)
// Chaque ligne représente une zone piétonne (P0, P1, P2, P3)
const int pedLEDs[4][2] = {
  {A0, A1}, // Zone 0: Rouge(A0), Vert(A1) (D14, D15)
  {A2, A3}, // Zone 1: Rouge(A2), Vert(A3) (D16, D17)
  {A4, A5}, // Zone 2: Rouge(A4), Vert(A5) (D18, D19)
  {A6, A7}  // Zone 3: Rouge(A6), Vert(A7) (D20, D21)
};

// Broches UART pour la communication avec l'ESP32
// D0 (RX) et D1 (TX) sont utilisés par Serial1 sur l'Arduino Nano IoT 33
const int ARDUINO_TX = 1; // D1 (Arduino Nano TX) -> ESP32 RX2 (GPIO16)
const int ARDUINO_RX = 0; // D0 (Arduino Nano RX) -> ESP32 TX2 (GPIO17)

// --- Variables de contrôle des feux ---
enum LightState { RED, ORANGE, GREEN };
enum Mode { AUTO, MANUAL, AMBULANCE_EMERGENCY };

Mode currentMode = AUTO; // Démarrage en mode automatique
int currentPhase = 0;    // Index de la phase actuelle (0 à 3 pour les 4 zones principales)
unsigned long lastPhaseChangeTime = 0; // Temps du dernier changement de phase
int manualControlZone = -1; // -1 pour aucune zone en contrôle manuel direct

// Durées des phases en millisecondes
const long GREEN_LIGHT_DURATION = 8000;  // 8 secondes
const long ORANGE_LIGHT_DURATION = 2000;  // 2 secondes
const long RED_LIGHT_DURATION = 10000;    // Rouge = Vert + Orange

// Durée de la phase d'urgence ambulance (chaque zone verte pendant ce temps)
const long AMBULANCE_GREEN_DURATION = 10000; // 10 secondes pour chaque zone verte en urgence

// Variables pour le débogage (peut être désactivé en production)
unsigned long lastDebugPrintTime = 0;
const long debugPrintInterval = 1000; // Afficher le statut toutes les 1 seconde

// --- Fonctions utilitaires ---

// Fonction pour éteindre toutes les LEDs d'une zone
void turnOffAllLEDs(int zoneIndex) {
  for (int i = 0; i < 3; i++) { // Véhicules
    digitalWrite(carLEDs[zoneIndex][i], LOW);
  }
  for (int i = 0; i < 2; i++) { // Piétons
    digitalWrite(pedLEDs[zoneIndex][i], LOW);
  }
}

// Fonction pour allumer une LED spécifique
void setLED(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

// Fonction pour mettre à jour l'état des feux de circulation pour une zone véhicule
void setCarLights(int zoneIndex, LightState state) {
  turnOffAllLEDs(zoneIndex); // Éteindre toutes les LEDs de la zone d'abord
  switch (state) {
    case RED:
      setLED(carLEDs[zoneIndex][0], HIGH); // Rouge
      setLED(pedLEDs[zoneIndex][1], HIGH); // Piéton Vert
      break;
    case ORANGE:
      setLED(carLEDs[zoneIndex][1], HIGH); // Orange
      setLED(pedLEDs[zoneIndex][0], HIGH); // Piéton Rouge
      break;
    case GREEN:
      setLED(carLEDs[zoneIndex][2], HIGH); // Vert
      setLED(pedLEDs[zoneIndex][0], HIGH); // Piéton Rouge
      break;
  }
}

// Fonction pour définir un état de feu piéton spécifique (utilisé pour les zones croisées)
void setPedestrianLight(int zoneIndex, LightState state) {
    setLED(pedLEDs[zoneIndex][0], LOW); // Éteint Piéton Rouge
    setLED(pedLEDs[zoneIndex][1], LOW); // Éteint Piéton Vert

    if (state == GREEN) {
        setLED(pedLEDs[zoneIndex][1], HIGH); // Allume Piéton Vert
    } else { // RED ou ORANGE -> Piéton Rouge
        setLED(pedLEDs[zoneIndex][0], HIGH); // Allume Piéton Rouge
    }
}

// Fonction pour envoyer le statut actuel de toutes les LEDs à l'ESP32
void sendLEDStatusToESP32() {
  String statusMessage = "LED_STATUS:";

  // Statut des feux de véhicules
  for (int i = 0; i < 4; i++) {
    statusMessage += "Z" + String(i) + "_";
    if (digitalRead(carLEDs[i][0]) == HIGH) statusMessage += "R";
    else if (digitalRead(carLEDs[i][1]) == HIGH) statusMessage += "O";
    else if (digitalRead(carLEDs[i][2]) == HIGH) statusMessage += "G";
    else statusMessage += "OFF"; // Si aucune LED véhicule n'est allumée
    if (i < 3) statusMessage += ",";
  }
  statusMessage += ";";

  // Statut des feux piétons
  for (int i = 0; i < 4; i++) {
    statusMessage += "P" + String(i) + "_";
    if (digitalRead(pedLEDs[i][0]) == HIGH) statusMessage += "R";
    else if (digitalRead(pedLEDs[i][1]) == HIGH) statusMessage += "G";
    else statusMessage += "OFF"; // Si aucune LED piéton n'est allumée
    if (i < 3) statusMessage += ",";
  }

  Serial1.println(statusMessage); // Envoyer via UART à l'ESP32
  Serial.print("[UART OUT] LED Status sent to ESP32: ");
  Serial.println(statusMessage);
}

// NOUVEAU : Fonction pour envoyer le compte à rebours à l'ESP32
void sendCountdownToESP32(int seconds) {
    String countdownMessage = "COUNTDOWN:" + String(seconds);
    Serial1.println(countdownMessage);
    Serial.print("[UART OUT] Countdown sent to ESP32: ");
    Serial.println(countdownMessage);
}


// --- Fonctions de gestion des commandes de l'ESP32 ---

void processIncomingCommand(String command) {
  Serial.print("[UART IN] Commande reçue de l'ESP32: ");
  Serial.println(command);

  // Gérer le mode automatique
  if (command == "AUTO") {
    currentMode = AUTO;
    currentPhase = 0; // Réinitialiser la phase
    lastPhaseChangeTime = millis();
    Serial.println("Mode: AUTOMATIQUE activé.");
  }
  // Gérer le mode manuel
  else if (command == "MANUAL") {
    currentMode = MANUAL;
    // Éteindre toutes les LEDs pour commencer le contrôle manuel
    for (int i = 0; i < 4; i++) {
      turnOffAllLEDs(i);
    }
    Serial.println("Mode: MANUEL activé. Toutes les LEDs éteintes.");
  }
  // Gérer les commandes de contrôle manuel par zone et couleur
  else if (command.startsWith("SET_TRAFFIC_LIGHTS:")) {
    if (currentMode == MANUAL) {
        // Ex: SET_TRAFFIC_LIGHTS:Z0_R,Z1_G,P2_R
        String lightStates = command.substring(19); // "Z0_R,Z1_G,P2_R"
        int startIndex = 0;
        int endIndex = lightStates.indexOf(',');

        while (startIndex != -1) {
            String part;
            if (endIndex == -1) {
                part = lightStates.substring(startIndex);
            } else {
                part = lightStates.substring(startIndex, endIndex);
            }
            
            if (part.startsWith("Z")) { // Véhicule
                int zone = part.substring(1, 2).toInt();
                char color = part.charAt(3);
                
                if (zone >= 0 && zone < 4) {
                    turnOffAllLEDs(zone); // Éteindre la zone avant de changer
                    if (color == 'R') setCarLights(zone, RED);
                    else if (color == 'O') setCarLights(zone, ORANGE);
                    else if (color == 'G') setCarLights(zone, GREEN);
                    else if (color == 'X') turnOffAllLEDs(zone); // 'X' pour éteindre
                    Serial.print("Zone "); Serial.print(zone); Serial.print(" set to "); Serial.println(color);
                }
            } else if (part.startsWith("P")) { // Piéton
                int zone = part.substring(1, 2).toInt();
                char color = part.charAt(3);
                if (zone >= 0 && zone < 4) {
                    setLED(pedLEDs[zone][0], LOW); // Éteindre rouge
                    setLED(pedLEDs[zone][1], LOW); // Éteindre vert
                    if (color == 'R') setPedestrianLight(zone, RED);
                    else if (color == 'G') setPedestrianLight(zone, GREEN);
                    else if (color == 'X') { // 'X' pour éteindre
                        setLED(pedLEDs[zone][0], LOW); 
                        setLED(pedLEDs[zone][1], LOW);
                    }
                    Serial.print("Ped Zone "); Serial.print(zone); Serial.print(" set to "); Serial.println(color);
                }
            }

            startIndex = endIndex + 1;
            if (startIndex >= lightStates.length()) break;
            endIndex = lightStates.indexOf(',', startIndex);
        }
    } else {
        Serial.println("Commande SET_TRAFFIC_LIGHTS ignorée car pas en mode manuel.");
    }
  }
  // Gérer l'urgence ambulance
  else if (command == "AMBULANCE_EMERGENCY_ON" || command.startsWith("AMBULANCE_EMERGENCY_ON:")) {
    currentMode = AMBULANCE_EMERGENCY;
    // Déterminer la zone de l'ambulance si spécifiée
    int ambulanceZone = -1;
    if (command.startsWith("AMBULANCE_EMERGENCY_ON:")) {
      ambulanceZone = command.substring(command.lastIndexOf(':') + 1).toInt();
      Serial.print("Urgence Ambulance activée pour la zone: "); Serial.println(ambulanceZone);
    } else {
      Serial.println("Urgence Ambulance activée.");
    }

    // Éteindre tous les feux pour commencer
    for (int i = 0; i < 4; i++) {
      turnOffAllLEDs(i);
    }

    // Mettre la zone de l'ambulance en vert, les autres en rouge
    for (int i = 0; i < 4; i++) {
      if (i == ambulanceZone) {
        setCarLights(i, GREEN);
      } else {
        setCarLights(i, RED);
      }
    }
    lastPhaseChangeTime = millis(); // Pour la durée de l'urgence
  }
  else if (command == "EMERGENCY_OFF") {
    Serial.println("Fin de l'urgence. Retour au mode AUTOMATIQUE.");
    currentMode = AUTO;
    currentPhase = 0; // Redémarrer la séquence normale
    lastPhaseChangeTime = millis();
    // Revenir à l'état initial du mode automatique (ex: Z0/Z2 Vert, Z1/Z3 Rouge)
    setCarLights(0, GREEN); // Zone 0 Vert
    setCarLights(2, GREEN); // Zone 2 Vert
    setCarLights(1, RED);   // Zone 1 Rouge
    setCarLights(3, RED);   // Zone 3 Rouge
  }
  // Gérer la commande CROSS:X pour les piétons (simule un bouton piéton)
  else if (command.startsWith("CROSS:")) {
    int zoneToCross = command.substring(6).toInt();
    if (zoneToCross >= 0 && zoneToCross < 4) {
      Serial.print("Commande CROSS recu pour la zone: ");
      Serial.println(zoneToCross);
      // Ici, vous pouvez ajouter une logique pour gérer le passage piéton.
      // Dans un système réel, cela interromprait la séquence et donnerait le vert aux piétons.
      // Pour cet exemple, nous allons juste afficher un message.
      Serial.print("La zone "); Serial.print(zoneToCross); Serial.println(" demande un passage piéton.");
      // Idéalement, cela devrait déclencher une séquence spécifique gérée par l'Arduino.
      // Pour l'instant, on se contente de l'afficher.
    }
  }
  else {
    Serial.print("Commande inconnue ou non prise en charge: ");
    Serial.println(command);
  }
  sendLEDStatusToESP32(); // Toujours envoyer le statut après une commande
}


// --- Séquence du mode automatique ---
void runAutoMode() {
  unsigned long currentTime = millis();
  long elapsedTime = currentTime - lastPhaseChangeTime;
  long remainingTime = 0; // Temps restant dans la phase actuelle

  switch (currentPhase) {
    case 0: // Zones 0 et 2 Vert, Zones 1 et 3 Rouge
      setCarLights(0, GREEN); setCarLights(2, GREEN);
      setCarLights(1, RED);   setCarLights(3, RED);
      
      remainingTime = (GREEN_LIGHT_DURATION - elapsedTime) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      sendCountdownToESP32(remainingTime); // Envoyer le compte à rebours (Vert 0, Vert 2, Rouge 1, Rouge 3)

      if (elapsedTime >= GREEN_LIGHT_DURATION) {
        currentPhase = 1; // Passage à l'Orange
        lastPhaseChangeTime = currentTime;
      }
      break;

    case 1: // Zones 0 et 2 Orange, Zones 1 et 3 Rouge
      setCarLights(0, ORANGE); setCarLights(2, ORANGE);
      setCarLights(1, RED);    setCarLights(3, RED); // Restent rouges
      
      remainingTime = (ORANGE_LIGHT_DURATION - elapsedTime) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      sendCountdownToESP32(remainingTime); // Envoyer le compte à rebours (Orange 0, Orange 2)

      if (elapsedTime >= ORANGE_LIGHT_DURATION) {
        currentPhase = 2; // Passage aux feux croisés
        lastPhaseChangeTime = currentTime;
      }
      break;

    case 2: // Zones 0 et 2 Rouge, Zones 1 et 3 Vert
      setCarLights(0, RED);   setCarLights(2, RED);
      setCarLights(1, GREEN); setCarLights(3, GREEN);
      
      remainingTime = (GREEN_LIGHT_DURATION - elapsedTime) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      sendCountdownToESP32(remainingTime); // Envoyer le compte à rebours (Vert 1, Vert 3, Rouge 0, Rouge 2)

      if (elapsedTime >= GREEN_LIGHT_DURATION) {
        currentPhase = 3; // Passage à l'Orange pour les zones croisées
        lastPhaseChangeTime = currentTime;
      }
      break;

    case 3: // Zones 0 et 2 Rouge, Zones 1 et 3 Orange
      setCarLights(0, RED);    setCarLights(2, RED); // Restent rouges
      setCarLights(1, ORANGE); setCarLights(3, ORANGE);
      
      remainingTime = (ORANGE_LIGHT_DURATION - elapsedTime) / 1000;
      if (remainingTime < 0) remainingTime = 0;
      sendCountdownToESP32(remainingTime); // Envoyer le compte à rebours (Orange 1, Orange 3)

      if (elapsedTime >= ORANGE_LIGHT_DURATION) {
        currentPhase = 0; // Retour à la phase initiale
        lastPhaseChangeTime = currentTime;
      }
      break;
  }
}

// Fonction pour gérer le mode urgence ambulance
void runAmbulanceEmergencyMode() {
  unsigned long currentTime = millis();
  if (currentTime - lastPhaseChangeTime >= AMBULANCE_GREEN_DURATION) {
    // Si la durée pour la zone verte est passée, basculer les autres feux au rouge et réinitialiser la durée.
    // Cette logique suppose que l'ESP32 envoie la commande "AMBULANCE_EMERGENCY_ON:X" pour une zone spécifique.
    // Si l'ESP32 ne spécifie pas de zone, cette partie doit être adaptée.
    // Ici, nous maintenons l'état défini par l'ESP32 jusqu'à "EMERGENCY_OFF".
    // Le buzzer est géré par l'ESP32 directement dans ce mode.
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200); // Pour le moniteur série de l'IDE
  Serial.println("--- Démarrage de l'Arduino Nano IoT 33 ---");

  Serial1.begin(9600); // Initialiser la communication UART avec l'ESP32
  Serial.println("UART1 initialisé pour communication avec ESP32.");

  // Initialiser les broches des LEDs comme OUTPUT
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) { // Feux véhicules (R, O, V)
      pinMode(carLEDs[i][j], OUTPUT);
      digitalWrite(carLEDs[i][j], LOW); // Éteindre toutes les LEDs au démarrage
    }
    for (int j = 0; j < 2; j++) { // Feux piétons (R, V)
      pinMode(pedLEDs[i][j], OUTPUT);
      digitalWrite(pedLEDs[i][j], LOW); // Éteindre toutes les LEDs au démarrage
    }
  }

  // Démarrer la première phase du mode automatique
  lastPhaseChangeTime = millis();
  setCarLights(0, GREEN); // Zone 0 (et 2 implicite) vert
  setCarLights(2, GREEN);
  setCarLights(1, RED);   // Zone 1 (et 3 implicite) rouge
  setCarLights(3, RED);
  sendLEDStatusToESP32(); // Envoyer le statut initial

  Serial.println("Setup terminé. Démarrage de la boucle principale.");
}

// --- Loop ---
void loop() {
  // Gérer les commandes entrantes de l'ESP32
  while (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim(); // Supprimer les espaces blancs
    if (command.length() > 0) {
      processIncomingCommand(command);
    }
  }

  // Exécuter la logique du mode actuel
  if (currentMode == AUTO) {
    runAutoMode();
  } else if (currentMode == AMBULANCE_EMERGENCY) {
    runAmbulanceEmergencyMode();
  }
  // En mode MANUAL, les LEDs sont contrôlées par les commandes MQTT via processIncomingCommand

  // Envoyer régulièrement le statut des LEDs à l'ESP32 (pour le débogage et la mise à jour de l'app)
  unsigned long currentTime = millis();
  if (currentTime - lastDebugPrintTime >= debugPrintInterval) {
    sendLEDStatusToESP32();
    lastDebugPrintTime = currentTime;
  }
}