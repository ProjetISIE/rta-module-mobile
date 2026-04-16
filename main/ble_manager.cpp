#include "ble_manager.hpp"
#include "active_look.hpp"
#include <string.h>
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// On accède à l'instance globale 'myGlasses' définie dans main.cpp
extern ActiveLook myGlasses;

// Fonction de gestion des événements GAP (découverte, connexion, etc.)
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    
    // Cas 1 : Nous détectons un appareil Bluetooth pendant notre "scan"
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        // Décodage des données publicitaires (Advertising Data) envoyées par l'appareil
        ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        
        // Si le nom de l'appareil existe et correspond à "ENGO" 
        if (fields.name && strncmp((char *)fields.name, "ENGO", 4) == 0) {
            ble_gap_disc_cancel(); // L'appareil désiré est trouvé, on stoppe la recherche
            
            // Configuration des paramètres avancés pour la tentative de connexion
            struct ble_gap_conn_params cp;
            memset(&cp, 0, sizeof(cp));
            cp.scan_itvl = 16; cp.scan_window = 16; cp.itvl_min = 24; cp.itvl_max = 40; cp.supervision_timeout = 500;
            
            // Ordre de connexion au périphérique qui vient d'être découvert
            ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, &cp, gap_event_handler, NULL);
        }
    } 
    // Cas 2 : La configuration et connexion avec l'appareil sont validées avec succès
    else if (event->type == BLE_GAP_EVENT_CONNECT && event->connect.status == 0) {
        // Optionnel : On attend 1,5s pour laisser le temps aux services de l'appareil de s'initialiser
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        // La connexion est prête : on passe la main à 'myGlasses' avec le numéro de la connexion (handle)
        myGlasses.displayHello(event->connect.conn_handle);
    }
    return 0; // Succès
}

// Fonction appelée automatiquement quand le contrôleur Bluetooth a fini de démarrer (synchronisation)
static void ble_on_sync(void) {
    struct ble_gap_disc_params dp;
    memset(&dp, 0, sizeof(dp)); 
    // Démarrage de la découverte (scan) Bluetooth de manière indéfinie (BLE_HS_FOREVER)
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp, gap_event_handler, NULL);
}

// Fonction principale pour démarrer toute l'infrastructure Bluetooth depuis le main()
void ble_manager_init() {
    nimble_port_init(); // Initialisation de base du port NimBLE sur l'ESP32
    
    // Associe la fonction de démarrage (sync_cb) dès que la pile Bluetooth est opérationnelle
    ble_hs_cfg.sync_cb = ble_on_sync;
    
    // Démarre la pile Bluetooth sur un thread (tâche séparée dans FreeRTOS)
    xTaskCreate([](void* p) { nimble_port_run(); }, "ble", 4096, nullptr, 5, nullptr);
}
