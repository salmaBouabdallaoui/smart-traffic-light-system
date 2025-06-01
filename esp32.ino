// Inclure les bibliothèques nécessaires
#include <WiFi.h> // Pour la connectivité Wi-Fi
#include <PubSubClient.h> // Pour la communication MQTT
#include <LiquidCrystal_I2C.h> // Pour l'écran LCD I2C
#include <time.h> // Pour la gestion du temps (NTP)

// --- Configuration Wi-Fi ---
const char* ssid = "TP-LINK_3177B4"; 
const char* password = "46858498";

// --- Configuration MQTT ---

const char* mqtt_server = "broker.hivemq.com"; 
const int mqtt_port = 1883; 
const char* mqtt_client_id = "ESP32_TrafficLight_Systemsalmaesto"; 

// Topics MQTT
const char* TOPIC_BUTTON_PRESS = "traffic/buttonssalma"; // Pour envoyer les presses de bouton
const char* TOPIC_SOUND_SENSOR = "traffic/soundsalma"; // Pour envoyer l'état du capteur de son
const char* TOPIC_LCD_DISPLAY = "traffic/lcd_messagesalma"; // Pour recevoir les messages à afficher sur le LCD
const char* TOPIC_7SEG_DISPLAY_OUT = "traffic/countdownsalma"; // (Now unused by ESP32, as Nano will send to ESP32 via UART, not MQTT)
const char* TOPIC_CONTROL_COMMANDS = "traffic/controlsalma"; // Pour recevoir les commandes de contrôle de l'application web
const char* TOPIC_LED_STATUS = "traffic/lights/current_statussalma"; // Pour publier le statut des LEDs de l'Arduino Nano

// --- Définition des broches ESP32 ---

// Broches pour les boutons poussoirs (avec résistances pull-down externes)
const int buttonPins[] = {12, 25, 26, 27}; // GPIO12, GPIO25, GPIO26, GPIO27
const int NUM_BUTTONS = sizeof(buttonPins) / sizeof(buttonPins[0]);
unsigned long lastButtonPressTime[NUM_BUTTONS] = {0}; // Pour le débounce des boutons
const long debounceDelay = 50; // Délai de débounce en ms

// Broche pour le capteur de son (maintenant en mode analogique)
const int soundSensorPin = 34; // GPIO34 (Broche ADC disponible)
// Le seuil pour considérer un son comme "bruyant".
// Vous devrez ajuster cette valeur après avoir observé les lectures brutes sur le moniteur série.
const int soundThreshold = 500; // Exemple: une valeur entre 0 et 4095 (4095 étant le maximum pour l'ESP32)

// Broches pour l'afficheur LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2); // Ajustez l'adresse I2C (0x27) si nécessaire, commun 0x3F ou 0x27

// Broches pour le registre à décalage 74HC595 (pour le seul afficheur 7 segments)
// Connecté à l'ESP32 car l'Arduino Nano n'a plus de broches libres.
const int dataPin1 = 13; // DS (Data) - GPIO13
const int latchPin1 = 18; // ST_CP (Latch) - GPIO18
const int clockPin1 = 14; // SH_CP (Clock) - GPIO14

// Broches pour la communication UART avec l'Arduino Nano IoT 33
const int RXD2 = 16; // GPIO16 (ESP32 RX2) -> Arduino Nano TX (D1)
const int TXD2 = 17; // GPIO17 (ESP32 TX2) -> Arduino Nano RX (D0)

// Broche pour le buzzer
const int buzzerPin = 33; // GPIO33

// Variables pour le clignotement du buzzer (avertissement piéton)
bool buzzerWarningActive = false;
unsigned long lastBuzzerToggleTime = 0;
const long buzzerToggleInterval = 250; // Clignote toutes les 250 ms (fréquence de 2 Hz)

// Variables de gestion de l'état du système pour l'affichage LCD
bool isAmbulanceEmergencyActive = false; // Vrai si une urgence ambulance est en cours
unsigned long lastCustomLcdMessageTime = 0; // Dernier temps où un message LCD personnalisé a été affiché
const long customLcdMessageDuration = 5000; // Durée d'affichage d'un message LCD personnalisé (5 secondes)
bool hasCustomLcdMessage = false; // Vrai si un message LCD personnalisé est actuellement actif
bool isDisplayingDate = false; // Garde la trace si la date est actuellement affichée

// Objet Serial pour UART2
HardwareSerial SerialPort(2); // Utilise UART2 de l'ESP32

// Tableau de mapping des chiffres pour l'afficheur 7 segments (commun anode)
// Chaque byte représente l'état des segments (dp, g, f, e, d, c, b, a)
// 0 = allumé, 1 = éteint pour commun anode
// L'ordre des bits dans le byte est important et doit correspondre à votre câblage et à MSBFIRST/LSBFIRST.
// !! ATTENTION !! Si votre afficheur est à CATHODE COMMUNE, vous devrez INVERSER toutes ces valeurs.
// Par exemple, pour un afficheur CATHODE COMMUNE, le 0b11000000 (0) deviendrait 0b00111111 (0x3F)
byte seven_seg_digits[] = {
  0b11000000, // 0 (0xC0) - segments a,b,c,d,e,f ON, g OFF, dp OFF
  0b11111001, // 1 (0xF9) - segments b,c ON, others OFF
  0b10100100, // 2 (0xA4) - segments a,b,d,e,g ON, c,f OFF
  0b10110000, // 3 (0xB0) - segments a,b,c,d,g ON, e,f OFF
  0b10011001, // 4 (0x99) - segments b,c,f,g ON, a,d,e OFF
  0b10010010, // 5 (0x92) - segments a,c,d,f,g ON, b,e OFF
  0b10000010, // 6 (0x82) - segments a,c,d,e,f,g ON, b OFF
  0b11111000, // 7 (0xF8) - segments a,b,c ON, others OFF
  0b10000000, // 8 (0x80) - all segments ON (except dp)
  0b10010000, // 9 (0x90) - segments a,b,c,d,f,g ON, e OFF
  0b11111111  // BLANK (0xFF) - all segments OFF
};


// Objets Wi-Fi et MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// --- Fonctions de gestion du temps (NTP) ---
// GMT+1 pour le Maroc (fuseau horaire standard). Pendant le Ramadan, le Maroc passe temporairement à GMT.
// Pour la plupart de l'année, GMT+1 est correct.
const long gmtOffset_sec = 3600; // 1 heure * 3600 secondes/heure = 3600
const int daylightOffset_sec = 0; // Pas d'ajustement DST (le Maroc est en GMT+1 la plupart du temps)

void initTime() {
  Serial.println("Synchronisation de l'heure avec les serveurs NTP...");
  // Utiliser des serveurs NTP plus fiables
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.google.com", "time.cloudflare.com");

  struct tm timeinfo;
  int retry_count = 0;
  // Attendre que l'heure soit synchronisée (max 20 tentatives)
  while (!getLocalTime(&timeinfo) && retry_count < 20) {
    delay(500);
    Serial.print(".");
    retry_count++;
  }
  Serial.println();

  if (!getLocalTime(&timeinfo)) {
    Serial.println("Échec de l'obtention de l'heure depuis NTP. Vérifiez la connexion WiFi et NTP.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Echec Synchro");
    lcd.setCursor(0,1);
    lcd.print("Heure!");
    // Pas de delay(2000) ici pour ne pas bloquer le setup
    return;
  }
  Serial.println("Heure synchronisée !");
  char time_buff[30];
  strftime(time_buff, sizeof(time_buff), "%A, %d %B %Y %H:%M:%S", &timeinfo);
  Serial.println(time_buff);
}

// Fonction pour afficher la date actuelle sur le LCD
void displayCurrentDate() {
  struct tm timeinfo;
  // Tenter d'obtenir l'heure, si échec, indiquer "Date inconnue"
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Erreur: Impossible d'obtenir la date pour l'affichage LCD.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Date inconnue");
    return;
  }
  char dateBuffer[17]; // Format "JJ/MM/AAAA" (10 caractères) + null terminator
  strftime(dateBuffer, sizeof(dateBuffer), "%d/%m/%Y", &timeinfo); // DD/MM/YYYY
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Date du jour:");
  lcd.setCursor(0, 1);
  lcd.print(dateBuffer);
  isDisplayingDate = true;
  Serial.println("[LCD] Affichage de la date actuelle.");
}

// --- Fonctions ---

// Fonction pour se connecter au Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connexion à ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); // Définir le mode Station
  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries++;
    if (retries > 60) { // Attendre max 30 secondes
      Serial.println("\nÉchec de la connexion WiFi. Redémarrage...");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Echec WiFi!");
      lcd.setCursor(0,1);
      lcd.print("Redemarrage...");
      delay(2000);
      ESP.restart(); // Redémarrer l'ESP32 si la connexion échoue
    }
  }

  Serial.println("");
  Serial.println("Connexion Wi-Fi établie");
  Serial.print("Adresse IP: ");
  Serial.println(WiFi.localIP());

  // Une fois connecté au Wi-Fi, initialiser le temps via NTP
  initTime();
}

// Fonction pour envoyer une commande à l'Arduino Nano via UART
void sendCommandToArduino(String command) {
  SerialPort.println(command); // Envoyer la commande via UART
  Serial.print("[UART OUT] Commande envoyée à Arduino: ");
  Serial.println(command);
}

// Fonction de callback MQTT (quand un message est reçu)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT IN] Message MQTT reçu sur le topic: ");
  Serial.print(topic);
  Serial.print(" -> ");
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // --- Gestion des commandes de contrôle (urgences et autres) ---
  if (String(topic) == TOPIC_CONTROL_COMMANDS) {
    if (message == "AMBULANCE_EMERGENCY_ON" || message.startsWith("AMBULANCE_EMERGENCY_ON:")) {
      sendCommandToArduino(message); // Transférer la commande à l'Arduino Nano
      isAmbulanceEmergencyActive = true; // Activer le drapeau d'urgence ambulance
      digitalWrite(buzzerPin, HIGH); // Activer le buzzer en continu
      buzzerWarningActive = false; // Désactiver le clignotement piéton si ambulance active
      Serial.println("[Buzzer] Urgence Ambulance: ON (continu)");

      // Afficher le message d'urgence sur le LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("URGENCE AMBULANCE!");
      lcd.setCursor(0, 1);
      lcd.print("DEGAGEZ LA VOIE!");
      hasCustomLcdMessage = false; // L'urgence écrase tout message personnalisé chronométré
      isDisplayingDate = false; // Plus besoin d'afficher la date
      Serial.println("[LCD] Message d'urgence ambulance affiché.");

    } else if (message == "AMBULANCE_EMERGENCY_OFF") {
      sendCommandToArduino("EMERGENCY_OFF"); // Transférer la commande au Nano
      isAmbulanceEmergencyActive = false; // Désactiver le drapeau d'urgence ambulance
      digitalWrite(buzzerPin, LOW); // Désactiver le buzzer
      buzzerWarningActive = false; // S'assurer que le clignotement est désactivé
      Serial.println("[Buzzer] Urgence Ambulance: OFF");

      // Afficher le message de fin d'urgence sur le LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Fin d'urgence.");
      lcd.setCursor(0, 1);
      lcd.print("Trafic normal.");
      lastCustomLcdMessageTime = millis(); // Enregistrer le temps pour afficher ce message pendant une durée
      hasCustomLcdMessage = true; // Le considérer comme un message personnalisé temporaire
      isDisplayingDate = false; // Pas la date pour l'instant
      Serial.println("[LCD] Message de fin d'urgence affiché.");

    } else if (message == "BUZZER_PED_A_WARN_ON") {
        if (!isAmbulanceEmergencyActive) { // N'activer l'avertissement piéton que si pas d'urgence ambulance
            buzzerWarningActive = true;
            Serial.println("[Buzzer] Avertissement Piéton: ON (clignotant)");
        } else {
            Serial.println("[Buzzer] Avertissement Piéton ignoré (Urgence Ambulance active)");
        }
    } else if (message == "BUZZER_PED_A_WARN_OFF") {
        buzzerWarningActive = false;
        if (!isAmbulanceEmergencyActive) { // N'éteindre que si pas d'urgence ambulance (pour ne pas couper le buzzer d'urgence)
            digitalWrite(buzzerPin, LOW);
            Serial.println("[Buzzer] Avertissement Piéton: OFF");
        } else {
            Serial.println("[Buzzer] Avertissement Piéton OFF ignoré (Urgence Ambulance active)");
        }
    }
    else {
      // Pour toutes les autres commandes de contrôle (auto, manual, CROSS:X, A:red, SET_TRAFFIC_LIGHTS: etc.)
      // Elles sont transmises directement à l'Arduino Nano pour traitement.
      sendCommandToArduino(message);
    }
  }
  // --- Gestion des messages LCD personnalisés ---
  else if (String(topic) == TOPIC_LCD_DISPLAY) {
    if (!isAmbulanceEmergencyActive) { // Ne pas écraser l'affichage d'urgence ambulance
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(message.substring(0, 16));
      if (message.length() > 16) {
        lcd.setCursor(0, 1);
        lcd.print(message.substring(16));
      }
      lastCustomLcdMessageTime = millis(); // Enregistrer le temps pour l'affichage temporaire
      hasCustomLcdMessage = true; // Activer le drapeau de message personnalisé
      isDisplayingDate = false; // Plus besoin d'afficher la date
      Serial.println("[LCD] Message personnalisé affiché.");
    } else {
      Serial.println("[LCD] Message personnalisé ignoré (Urgence Ambulance active).");
    }
  }
  // Note: TOPIC_7SEG_DISPLAY_OUT n'est plus utilisé par l'ESP32 pour l'affichage 7-segment,
  // car l'Arduino Nano enverra directement la valeur via UART.
  // Note: TOPIC_LED_STATUS n'est pas géré ici car l'ESP32 PUBLIE sur ce topic, il ne s'y ABONNE PAS.
}

// Fonction pour se reconnecter au broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentative de connexion MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connecté");
      // S'abonner aux topics de contrôle que l'ESP32 doit gérer ou transférer
      client.subscribe(TOPIC_LCD_DISPLAY);
      client.subscribe(TOPIC_CONTROL_COMMANDS); // L'ESP32 s'abonne à ce topic pour les commandes des feux
      Serial.println("[MQTT] Abonné aux topics de contrôle.");
    } else {
      Serial.print("échec, rc=");
      Serial.print(client.state());
      Serial.println(" nouvelle tentative dans 5 secondes");
      delay(5000); // Attendre 5 secondes avant de réessayer
    }
  }
}

// Fonction pour afficher un nombre sur le seul afficheur 7 segments
// Le nombre affiché sera l'unité du nombre reçu.
void display7Segment(int number) {
  // Gérer les nombres en dehors de la plage 0-9
  if (number < 0) {
    Serial.println("[7-Seg] display7Segment: Nombre négatif, affichage BLANK.");
    shiftOut(dataPin1, clockPin1, MSBFIRST, seven_seg_digits[10]); // Afficher BLANK
    return;
  }
  
  // N'afficher que le chiffre des unités sur le seul afficheur
  // Ex: si number = 15, units = 5. Si number = 2, units = 2.
  int units = number % 10; 

  // Assurez-vous que l'index est valide pour le tableau seven_seg_digits
  if (units >= 0 && units <= 9) {
    digitalWrite(latchPin1, LOW); // Pull latch LOW to start data transfer
    shiftOut(dataPin1, clockPin1, MSBFIRST, seven_seg_digits[units]); // Send data for units
    digitalWrite(latchPin1, HIGH); // Pull latch HIGH to display data
    Serial.print("[7-Seg] Affichage de l'unité: "); Serial.println(units);
  } else {
    // Si la valeur est inattendue, afficher un blank
    Serial.println("[7-Seg] display7Segment: Valeur d'unité invalide, affichage BLANK.");
    digitalWrite(latchPin1, LOW);
    shiftOut(dataPin1, clockPin1, MSBFIRST, seven_seg_digits[10]); // Afficher BLANK
    digitalWrite(latchPin1, HIGH);
  }
}


// --- Setup ---
void setup() {
  Serial.begin(115200); 
  Serial.println("--- Démarrage du Système ESP32 ---");

  // Initialiser la communication UART avec l'Arduino Nano IoT 33
  SerialPort.begin(9600, SERIAL_8N1, RXD2, TXD2); 
  Serial.println("UART2 initialisé pour communication avec Arduino Nano.");

  // Initialiser les broches des boutons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT); // Boutons avec pull-down externe
  }

  // Initialiser la broche du capteur de son
  pinMode(soundSensorPin, INPUT);

  // Initialiser les broches du registre à décalage (pour le seul afficheur 7 segments)
  pinMode(dataPin1, OUTPUT);
  pinMode(latchPin1, OUTPUT);
  pinMode(clockPin1, OUTPUT);
  // Assurer que le 74HC595 est bien configuré (MR à HIGH, OE à LOW)
  // Ces broches sont généralement câblées directement si non contrôlées dynamiquement.
  // Si câblées sur des GPIOs, décommenter et configurer ici:
  // pinMode(MR_PIN, OUTPUT); digitalWrite(MR_PIN, HIGH);
  // pinMode(OE_PIN, OUTPUT); digitalWrite(OE_PIN, LOW);


  // Initialiser la broche du buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW); // S'assurer que le buzzer est éteint au démarrage

  // Initialiser l'écran LCD I2C
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Systeme Feux");
  lcd.setCursor(0, 1);
  lcd.print("Intelligents");
  delay(2000); // Court délai pour que le message soit visible
  lcd.clear();

  // Configurer le Wi-Fi et synchroniser l'heure via NTP
  setup_wifi();

  // Configurer le client MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.println("Setup terminé. Démarrage de la boucle principale.");

  // Test du 7 segments au démarrage (pour le débogage)
  Serial.println("[7-Seg] Test d'affichage '8' pendant 2 secondes...");
  display7Segment(8);
  delay(2000);
  Serial.println("[7-Seg] Test d'affichage '0' pendant 2 secondes...");
  display7Segment(0);
  delay(2000);
  Serial.println("[7-Seg] Test de fin.");
}

// --- Loop ---
void loop() {
  // Vérifier la connexion MQTT et se reconnecter si nécessaire
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Traiter les messages MQTT entrants et maintenir la connexion

  // --- Lecture et publication des messages UART (du Nano vers MQTT) ---
  while (SerialPort.available()) { 
    String uartMessage = SerialPort.readStringUntil('\n'); 
    uartMessage.trim(); // Supprimer les espaces blancs
    Serial.print("[UART IN] Message reçu de Arduino: ");
    Serial.println(uartMessage);

    // Si le message est le statut des LEDs, le publier sur MQTT
    if (uartMessage.startsWith("LED_STATUS:")) {
      if (client.connected()) {
        client.publish(TOPIC_LED_STATUS, uartMessage.c_str());
        Serial.print("[MQTT OUT] Publié le statut des LEDs sur ");
        Serial.print(TOPIC_LED_STATUS);
        Serial.print(": ");
        Serial.println(uartMessage);
      } else {
        Serial.println("[MQTT OUT] Erreur: Client MQTT non connecté, impossible de publier le statut des LEDs.");
      }
    } 
    // NOUVEAU: Si le message est un compte à rebours de l'Arduino Nano
    else if (uartMessage.startsWith("COUNTDOWN:")) {
      int countdownValue = uartMessage.substring(10).toInt(); 
      Serial.print("[UART IN] Compte à rebours reçu: ");
      Serial.println(countdownValue);
      // La fonction display7Segment gérera l'affichage de l'unité
      display7Segment(countdownValue); 
    }
    else {
      // Si ce n'est pas un message de statut LED ou de compte à rebours
      Serial.print("[UART IN] Message UART non géré par l'ESP32: ");
      Serial.println(uartMessage);
    }
  }

  // --- Lecture des boutons ---
  for (int i = 0; i < NUM_BUTTONS; i++) {
    int buttonState = digitalRead(buttonPins[i]);
    // Détection de l'appui (passage de LOW à HIGH) et débounce
    if (buttonState == HIGH && (millis() - lastButtonPressTime[i]) > debounceDelay) {
      Serial.print("[Bouton] Bouton ");
      Serial.print(i + 1);
      Serial.println(" pressé !");
      String payload = "{\"button\": " + String(i + 1) + ", \"state\": \"pressed\"}";
      if (client.connected()) {
        client.publish(TOPIC_BUTTON_PRESS, payload.c_str()); // Publier sur MQTT
      } else {
        Serial.println("[MQTT OUT] Erreur: Client MQTT non connecté, impossible de publier le bouton.");
      }
      lastButtonPressTime[i] = millis();
    }
  }

  // --- Lecture du capteur de son (analogique) ---
  static unsigned long lastSoundReadTime = 0;
  const long soundReadInterval = 1000; // Lire le capteur toutes les 1 seconde

  if (millis() - lastSoundReadTime >= soundReadInterval) {
    lastSoundReadTime = millis();
    int soundValue = analogRead(soundSensorPin);
    Serial.print("[Capteur Son] Valeur analogique (GPIO34): "); 
    Serial.println(soundValue); 
    
    static bool lastSoundDetected = false;
    bool currentSoundDetected = (soundValue > soundThreshold);

    // La publication n'a lieu QUE si l'état de détection du son CHANGE
    if (currentSoundDetected != lastSoundDetected) {
      if (client.connected()) {
        if (currentSoundDetected) {
          Serial.println("[Capteur Son] Son bruyant détecté !");
          String payload = "{\"sound_detected\": true}";
          client.publish(TOPIC_SOUND_SENSOR, payload.c_str());
        } else {
          Serial.println("[Capteur Son] Pas de son bruyant.");
          String payload = "{\"sound_detected\": false}";
          client.publish(TOPIC_SOUND_SENSOR, payload.c_str());
        }
      } else {
        Serial.println("[MQTT OUT] Erreur: Client MQTT non connecté, impossible de publier le son.");
      }
      lastSoundDetected = currentSoundDetected;
    }
  }

  // --- Gestion du Buzzer ---
  if (isAmbulanceEmergencyActive) {
      // Le buzzer est déjà géré comme continu (HIGH) dans le callback d'urgence ambulance
      // Il reste allumé en continu tant que l'urgence est active
  } else if (buzzerWarningActive) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastBuzzerToggleTime >= buzzerToggleInterval) {
          lastBuzzerToggleTime = currentMillis;
          // Basculer l'état du buzzer
          int currentBuzzerState = digitalRead(buzzerPin);
          digitalWrite(buzzerPin, !currentBuzzerState); // Inverser l'état
          // Serial.print("[Buzzer] Clignotement: "); Serial.println(!currentBuzzerState ? "ON" : "OFF");
      }
  } else {
      // Ni urgence, ni avertissement piéton, s'assurer que le buzzer est éteint
      if (digitalRead(buzzerPin) == HIGH) { // Éviter les écritures inutiles
          digitalWrite(buzzerPin, LOW);
          Serial.println("[Buzzer] Éteint.");
      }
  }

  // --- Gestion de l'affichage LCD ---
  static unsigned long lastLcdDateUpdate = 0;
  const long lcdDateUpdateInterval = 5000; // Mettre à jour la date toutes les 5 secondes

  if (isAmbulanceEmergencyActive) {
      // L'écran LCD affiche le message d'urgence (géré dans le callback), ne rien faire ici
      // S'assurer qu'il ne tente pas d'afficher autre chose
      if (hasCustomLcdMessage || isDisplayingDate) {
        hasCustomLcdMessage = false;
        isDisplayingDate = false;
      }
  } else if (hasCustomLcdMessage && (millis() - lastCustomLcdMessageTime < customLcdMessageDuration)) {
      // Un message MQTT personnalisé est toujours actif sur l'écran LCD, ne rien faire
  } else {
      // Pas d'urgence, pas de message personnalisé actif, afficher la date ou rafraîchir
      if (hasCustomLcdMessage) { // Si un message personnalisé vient de "expirer"
          hasCustomLcdMessage = false; // Réinitialiser le drapeau
          displayCurrentDate(); // Afficher la date immédiatement
          lastLcdDateUpdate = millis(); // Réinitialiser le timer
          Serial.println("[LCD] Retour à l'affichage de la date (message personnalisé expiré).");
      } else if (millis() - lastLcdDateUpdate >= lcdDateUpdateInterval || !isDisplayingDate) {
          // Si l'intervalle est passé OU si la date n'est pas encore affichée
          displayCurrentDate(); // Mettre à jour la date après l'intervalle
          lastLcdDateUpdate = millis();
          Serial.println("[LCD] Date mise à jour / Affichage initial.");
      }
  }
}