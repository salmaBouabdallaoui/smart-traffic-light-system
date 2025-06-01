# 🚦 Smart Traffic Light System (Système de Feux Tricolores Intelligents)

💡 **Projet de Stage** : Optimisation du Trafic Urbain et Sécurité Routière pour la Smart City


---

## ✨ Fonctionnalités Clés

- **Gestion Adaptative du Trafic**  
  Analyse en temps réel de la densité du trafic via une caméra (**YOLOv8**) pour ajuster dynamiquement les cycles des feux.

- **Priorisation des Véhicules d'Urgence**  
  Détection automatique des ambulances (via capteur de son et vision par ordinateur) pour libérer les intersections instantanément.

- **Interaction Piétonne Intelligente**  
  Boutons poussoirs dédiés permettant aux piétons de demander le passage, avec ajustement rapide du délai des feux.

- **Monitoring et Alertes Locales**  
  Affichage d’informations cruciales et de messages d’urgence sur un écran LCD, et indication des temps d’attente via des afficheurs 7-segments.  
  Inclut un mode nuit pour l’efficacité énergétique.

- **Détection et Enregistrement des Violations**  
  Identification des véhicules qui brûlent les feux rouges avec tentative de capture des plaques d’immatriculation.

- **Supervision et Contrôle à Distance**  
  Une application web développée avec **Flask** offre un tableau de bord en temps réel pour visualiser le statut des feux, les données de trafic (graphiques d’embouteillage) et les violations détectées.  
  Permet également un contrôle administratif manuel.

---

## ⚙️ Architecture Technologique

- **Système Embarqué (Hardware)**  
  - **ESP32** : Gère les capteurs (boutons, son), les afficheurs locaux (LCD, 7-segments), et la communication Wi-Fi/MQTT.  
  - **Arduino Nano IoT 33** : Contrôle précis des LEDs des feux tricolores.  
  - **Communication Interne** : Liaison série (**UART**) entre l’ESP32 et l’Arduino Nano.

- **Vision par Ordinateur (IA)**  
  - Utilisation de la caméra d’un téléphone portable comme source vidéo.  
  - Application **IVCam** pour streamer le flux vidéo du téléphone vers le PC.  
  - Modèle **YOLOv8** pour la détection en temps réel de véhicules, ambulances et piétons.

- **Communication IoT**  
  - Le protocole **MQTT** (Message Queuing Telemetry Transport) assure l’échange de données en temps réel entre tous les composants (embarqué, IA, web app).

- **Intelligence Artificielle (Logique)**  
  - Module Python traitant les données visuelles (YOLOv8) et sonores pour prendre des décisions intelligentes de contrôle des feux.

- **Application Web**  
  - Développée avec **Flask** (Python), elle sert d’interface utilisateur, de centre de supervision et de plateforme d’analyse des données.

---

## 🚀 Comment Démarrer (Setup local)

```bash
# Clonez le dépôt
git clone https://github.com/salmaBouabdallaoui/smart-traffic-light-system.git
cd smart-traffic-light-system
