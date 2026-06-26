---
lang: fr
---

<!--toc:start-->

- [Architecture Logicielle](#architecture-logicielle)
- [Paramétrage et Configuration](#paramétrage-et-configuration)

<!--toc:end-->

# Rider Training Assistant (RTA) : Module Mobile

Module attaché au couple cavalier-cheval étant chargé de

- Mesurer en continu son mouvement
  - Vitesse du système
  - Accélération du système
  - Accélération individuelle de chaque jambe du cheval
- Récupérer la distance à l’obstacle en communiquant en BLE avec l’autre module
- Afficher l’ordre de saut dans les lunettes de réalité augmentée du cavalier

## Architecture Logicielle

Le projet est basé sur le framework **ESP-IDF** (C++23) et est organisé en
plusieurs services qui s'exécutent sur le microcontrôleur ESP32 :

- **`AppContext` / `main`** : Tâche principale de contrôle
  (`processAndDisplayTask`). Elle coordonne les différents services et
  synchronise l'affichage.
- **`GpsService`** : Interagit avec le récepteur GPS pour obtenir la vitesse,
  l'état du positionnement (fix), le nombre de satellites et les coordonnées
  géographiques.
- **`BleServer`** : Gère un serveur BLE (nommé `RTA_MOBILE`) exposant les
  données, notamment les informations GPS via des services GATT personnalisés.
- **`BleManager`** : Opère en tant que client/scanner BLE. Son rôle principal
  est de détecter et de se connecter aux lunettes de réalité augmentée
  (ActiveLook).
- **`EspNowMobile`** : Gère la réception de la distance par rapport à l'obstacle
  en utilisant le protocole **ESP-NOW** (basse latence). Si la communication
  ESP-NOW est établie avec succès avec le module fixe, la connexion BLE
  équivalente est désactivée afin de libérer des ressources.
- **`ActiveLook`** : S'occupe du formatage et de la transmission de l'affichage
  tête haute (vitesse et distance) vers les lunettes connectées du cavalier.
- **`LoggerService`** : Service dédié à l'enregistrement et à la restitution des
  logs (lecture sur console, statut GPS/Distance).

## Paramétrage et Configuration

La configuration du module s'appuie principalement sur les outils standards
ESP-IDF (CMake et fichiers `sdkconfig`) :

- **Environnement de compilation** : Configuration via CMake en C++23.
  L'environnement reproductible est fourni par Nix (`flake.nix`).
- **Bluetooth (BLE)** : La pile **NimBLE** est privilégiée
  (`CONFIG_BT_NIMBLE_ENABLED=y`) pour sa faible empreinte mémoire, avec un
  maximum de 3 connexions concurrentes.
- **Partitionnement Flash (`partitions.csv`)** :
  - **Taille totale** : 4 MB (`CONFIG_ESPTOOLPY_FLASHSIZE="4MB"`).
  - **Schéma sur mesure** : Alloue 1500 Ko pour le firmware (`factory`), 2300 Ko
    pour le système de fichiers (`spiffs` ou `storage`), et inclut un espace
    pour le `nvs` (Non-Volatile Storage).
- **Optimisations Mémoire (`sdkconfig.defaults`)** :
  - Afin de préserver la mémoire vive (RAM / IRAM), les optimisations IRAM du
    Wi-Fi sont désactivées.
  - Les fonctions de Ringbuf et FreeRTOS sont explicitement placées en mémoire
    Flash plutôt qu'en RAM (`CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH=y`).
