🚦 Smart Traffic Light System (Système de Feux Tricolores Intelligents)
💡 Projet de Stage : Optimisation du Trafic Urbain et Sécurité Routière pour la Smart City
(Replace this image link with a diagram of your system's architecture, or a photo of your maquette if you have one. You can create a docs folder in your repo for images.)

Ce projet de stage développe un système de feux tricolores intelligents et adaptatifs conçu pour moderniser la gestion du trafic urbain dans le cadre d'une Smart City. En se concentrant sur un carrefour simulé (à Taourirt), notre solution intègre des technologies de pointe pour fluidifier la circulation, assurer la sécurité et optimiser la réponse aux événements en temps réel.

✨ Fonctionnalités Clés
Gestion Adaptative du Trafic : Analyse en temps réel de la densité du trafic via une caméra (YOLOv8) pour ajuster dynamiquement les cycles des feux.
Priorisation des Véhicules d'Urgence : Détection automatique des ambulances (via capteur de son et vision par ordinateur) pour libérer les intersections instantanément.
Interaction Piétonne Intelligente : Boutons poussoirs dédiés permettant aux piétons de demander le passage, avec ajustement rapide du délai des feux.
Monitoring et Alertes Locales : Affichage d'informations cruciales et de messages d'urgence sur un écran LCD, et indication des temps d'attente via des afficheurs 7-segments. Inclut un mode nuit pour l'efficacité énergétique.
Détection et Enregistrement des Violations : Identification des véhicules qui brûlent les feux rouges avec tentative de capture des matricules.
Supervision et Contrôle à Distance : Une application web développée avec Flask offre un tableau de bord en temps réel pour visualiser le statut des feux, les données de trafic (graphiques d'embouteillage), et les violations détectées. Permet également un contrôle administratif manuel du système.
⚙️ Architecture Technologique
Le système repose sur une architecture robuste et interconnectée :

Système Embarqué (Hardware) :
ESP32 : Gère les capteurs (boutons, son), les afficheurs locaux (LCD, 7-segments), et la communication Wi-Fi/MQTT.
Arduino Nano IoT 33 : Contrôle précis des LEDs des feux tricolores.
Communication Interne : Liaison série (UART) entre l'ESP32 et l'Arduino Nano pour un échange d'ordres rapide.
Vision par Ordinateur (IA) :
Utilisation de la caméra d'un téléphone portable comme source vidéo.
Application IVCam pour streamer le flux vidéo du téléphone vers le PC.
Modèle YOLOv8 pour la détection en temps réel de véhicules, ambulances et piétons.
Communication IoT :
Le protocole MQTT (Message Queuing Telemetry Transport) assure l'échange de données en temps réel entre tous les composants (embarqué, IA, web app).
Intelligence Artificielle (Logique) :
Module Python traitant les données visuelles (YOLOv8) et sonores pour prendre des décisions intelligentes de contrôle des feux.
Application Web :
Développée avec Flask en Python, elle sert d'interface utilisateur, de centre de commande/supervision et de plateforme d'analyse des données de trafic et des violations.
🚀 Comment Démarrer (Setup local)
(This section is crucial for a GitHub repo. Provide brief, actionable steps. If you have a docs folder, you can link to more detailed instructions there.)

Cloner le dépôt :
Bash

git clone https://github.com/salmaBouabdallaoui/smart-traffic-light-system.git
cd your-repo-name
Configuration Matérielle :
Assemblez les composants sur votre maquette comme décrit dans la section docs/hardware_setup.md (link to a more detailed doc).
Assurez-vous que l'application IVCam est installée sur votre téléphone et PC, et que la connexion IP est établie.
Firmware des Microcontrôleurs :
Ouvrez les fichiers .ino pour l'ESP32 et l'Arduino Nano dans Arduino IDE.
Configurez vos identifiants Wi-Fi et l'adresse du broker MQTT dans les codes respectifs.
Téléversez les firmwares sur vos cartes.
Backend IA & Application Web (Python) :
Créez un environnement virtuel Python : python -m venv venv
Activez-le : source venv/bin/activate (Linux/macOS) ou venv\Scripts\activate (Windows)
Installez les dépendances : pip install -r requirements.txt (Make sure you have a requirements.txt file listing Flask, PyMQTT, OpenCV, etc.)
Lancez le broker MQTT (si local).
Lancez le script Python de l'IA et l'application Flask.
🤝 Contribution
Ce projet a été développé dans le cadre de notre stage. Pour toute question ou suggestion, n'hésitez pas à ouvrir une issue ou à nous contacter.
📄 Licence
Ce projet est sous licence MIT License
