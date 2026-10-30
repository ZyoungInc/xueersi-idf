#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define ESPNOW_LINK_MAX_PEERS 6
#define ESPNOW_LINK_CHANNEL   1

typedef enum {
    ESPNOW_LINK_OFF,
    ESPNOW_LINK_STARTING,
    ESPNOW_LINK_READY,
    ESPNOW_LINK_ERROR,
} espnow_link_state_t;

typedef struct {
    uint8_t mac[6];
    bool paired;
    bool online;
    bool has_signal;
    int8_t rssi;
} espnow_link_peer_t;

typedef struct {
    uint32_t revision;
    espnow_link_state_t state;
    esp_err_t error;
    esp_err_t storage_error;
    uint8_t local_mac[6];
    uint8_t peer_count;
    uint8_t online_count;
    uint8_t pairing_seconds;
    bool pairing;
    espnow_link_peer_t peers[ESPNOW_LINK_MAX_PEERS];
} espnow_link_status_t;

esp_err_t espnow_link_init(void);
esp_err_t espnow_link_start_pairing(void);
esp_err_t espnow_link_cancel_pairing(void);
esp_err_t espnow_link_forget(const uint8_t mac[6]);
void espnow_link_get_status(espnow_link_status_t *status);

#endif
