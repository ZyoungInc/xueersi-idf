#include "espnow_link.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#define LINK_MAGIC          0x584D4E57u
#define LINK_VERSION        1
#define LINK_PAIR_MS        30000
#define LINK_HEARTBEAT_MS   1000
#define LINK_TIMEOUT_MS     3500
#define LINK_PAIR_RETRY_MS  650
#define LINK_STORE_VERSION  1

typedef enum {
    PACKET_DISCOVER = 1,
    PACKET_REQUEST,
    PACKET_ACCEPT,
    PACKET_PING,
    PACKET_PONG,
    PACKET_FORGET,
} packet_type_t;

/* Fixed-size, little-endian control frames; no sensor or application payloads. */
typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t reserved;
    uint32_t session;
    uint32_t target_session;
    uint32_t pair_token;
    uint32_t pair_echo;
    uint32_t sequence;
} link_packet_t;

_Static_assert(sizeof(link_packet_t) == 28, "ESP-NOW control frame size changed");

typedef struct {
    uint8_t mac[6];
    int8_t rssi;
    bool broadcast;
    uint32_t received_ms;
    link_packet_t packet;
} receive_event_t;

typedef enum {
    COMMAND_PAIR,
    COMMAND_CANCEL,
    COMMAND_FORGET,
} command_type_t;

typedef struct {
    command_type_t type;
    uint8_t mac[6];
} link_command_t;

typedef struct {
    bool used;
    bool paired;
    bool driver_added;
    bool has_signal;
    bool has_ack;
    bool awaiting_ack;
    bool handshake_ready;
    uint8_t mac[6];
    int8_t rssi;
    uint32_t remote_session;
    uint32_t remote_pair_token;
    uint32_t last_seen_ms;
    uint32_t last_ack_ms;
    uint32_t last_ping_ms;
    uint32_t last_pair_ms;
    uint32_t sequence;
} link_peer_t;

typedef struct {
    uint32_t version;
    uint32_t count;
    uint8_t macs[ESPNOW_LINK_MAX_PEERS][6];
} stored_peers_t;

static const char *TAG = "espnow_link";
static const uint8_t s_broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static QueueHandle_t s_receive_queue;
static QueueHandle_t s_command_queue;
static SemaphoreHandle_t s_send_done;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static espnow_link_status_t s_status;

/* Runtime peers, Wi-Fi and NVS are owned exclusively by the link task. */
static link_peer_t s_peers[ESPNOW_LINK_MAX_PEERS];
static uint8_t s_local_mac[6];
static espnow_link_state_t s_state;
static esp_err_t s_error;
static esp_err_t s_storage_error;
static nvs_handle_t s_storage;
static bool s_storage_ready;
static bool s_network_ready;
static bool s_radio_ready;
static bool s_pairing;
static uint32_t s_session;
static uint32_t s_sequence;
static uint32_t s_pair_token;
static uint32_t s_pair_started_ms;
static uint32_t s_last_discovery_ms;
static uint32_t s_discovery_interval_ms;

static uint32_t link_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint32_t link_random(void)
{
    const uint32_t value = esp_random();
    return value ? value : 1;
}

static bool valid_mac(const uint8_t mac[6])
{
    static const uint8_t zero[6];
    return !(mac[0] & 1) && memcmp(mac, zero, 6) != 0 && memcmp(mac, s_local_mac, 6) != 0;
}

static bool pairing_active(void)
{
    return s_pairing && link_now() - s_pair_started_ms < LINK_PAIR_MS;
}

static link_peer_t *find_peer(const uint8_t mac[6])
{
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (s_peers[i].used && memcmp(s_peers[i].mac, mac, 6) == 0) {
            return &s_peers[i];
        }
    }
    return NULL;
}

static void publish_status(void)
{
    const uint32_t now = link_now();
    espnow_link_status_t next = {0};
    next.state = s_state;
    next.error = s_error;
    next.storage_error = s_storage_error;
    next.pairing = pairing_active();
    if (next.pairing) {
        next.pairing_seconds = (LINK_PAIR_MS - (now - s_pair_started_ms) + 999) / 1000;
    }
    memcpy(next.local_mac, s_local_mac, 6);
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        const link_peer_t *peer = &s_peers[i];
        if (!peer->used) {
            continue;
        }
        espnow_link_peer_t *view = &next.peers[next.peer_count++];
        memcpy(view->mac, peer->mac, 6);
        view->paired = peer->paired;
        view->has_signal = peer->has_signal;
        view->rssi = peer->rssi;
        view->online = peer->paired && s_radio_ready && peer->has_ack &&
                       now - peer->last_ack_ms < LINK_TIMEOUT_MS &&
                       now - peer->last_seen_ms < LINK_TIMEOUT_MS;
        next.online_count += view->online;
    }

    portENTER_CRITICAL(&s_status_lock);
    next.revision = s_status.revision;
    if (memcmp(&next, &s_status, sizeof(next)) != 0) {
        next.revision++;
        s_status = next;
    }
    portEXIT_CRITICAL(&s_status_lock);
}

void espnow_link_get_status(espnow_link_status_t *status)
{
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_status_lock);
}

static void load_peers(void)
{
    s_storage_error = nvs_flash_init();
    if (s_storage_error != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable, pairing is RAM-only: %s", esp_err_to_name(s_storage_error));
        return;
    }
    s_storage_error = nvs_open("espnow_link", NVS_READWRITE, &s_storage);
    if (s_storage_error != ESP_OK) {
        return;
    }
    s_storage_ready = true;
    stored_peers_t stored = {0};
    size_t size = sizeof(stored);
    esp_err_t err = nvs_get_blob(s_storage, "peers", &stored, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (err != ESP_OK || size != sizeof(stored) || stored.version != LINK_STORE_VERSION ||
        stored.count > ESPNOW_LINK_MAX_PEERS) {
        s_storage_error = err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
        return;
    }
    for (uint32_t i = 0; i < stored.count; ++i) {
        if (!valid_mac(stored.macs[i]) || find_peer(stored.macs[i])) {
            s_storage_error = ESP_ERR_INVALID_RESPONSE;
            continue;
        }
        s_peers[i].used = true;
        s_peers[i].paired = true;
        memcpy(s_peers[i].mac, stored.macs[i], 6);
    }
}

static void save_peers(void)
{
    if (!s_storage_ready) {
        return;
    }
    stored_peers_t stored = {.version = LINK_STORE_VERSION};
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (s_peers[i].used && s_peers[i].paired) {
            memcpy(stored.macs[stored.count++], s_peers[i].mac, 6);
        }
    }
    s_storage_error = nvs_set_blob(s_storage, "peers", &stored, sizeof(stored));
    if (s_storage_error == ESP_OK) {
        s_storage_error = nvs_commit(s_storage);
    }
    if (s_storage_error != ESP_OK) {
        ESP_LOGW(TAG, "Cannot save peers: %s", esp_err_to_name(s_storage_error));
    }
}

static esp_err_t add_driver_peer(const uint8_t mac[6])
{
    esp_now_peer_info_t info = {
        .channel = ESPNOW_LINK_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(info.peer_addr, mac, 6);
    const esp_err_t err = esp_now_add_peer(&info);
    return err == ESP_ERR_ESPNOW_EXIST ? ESP_OK : err;
}

static link_peer_t *ensure_peer(const uint8_t mac[6])
{
    link_peer_t *peer = find_peer(mac);
    if (peer) {
        return peer;
    }
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (!s_peers[i].used) {
            if (add_driver_peer(mac) != ESP_OK) {
                return NULL;
            }
            peer = &s_peers[i];
            memset(peer, 0, sizeof(*peer));
            peer->used = true;
            peer->driver_added = true;
            memcpy(peer->mac, mac, 6);
            return peer;
        }
    }
    return NULL;
}

static void remove_peer(link_peer_t *peer)
{
    if (peer->driver_added) {
        esp_now_del_peer(peer->mac);
    }
    memset(peer, 0, sizeof(*peer));
}

static void end_pairing(void)
{
    s_pairing = false;
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (s_peers[i].used && !s_peers[i].paired) {
            remove_peer(&s_peers[i]);
        }
    }
}

static void sent_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    (void)status;
    xSemaphoreGive(s_send_done);
}

static void receive_cb(const esp_now_recv_info_t *info, const uint8_t *data, int size)
{
    if (!info || !info->src_addr || !info->des_addr || !info->rx_ctrl || !data ||
        size != (int)sizeof(link_packet_t)) {
        return;
    }
    receive_event_t event = {0};
    memcpy(&event.packet, data, sizeof(event.packet));
    if (event.packet.magic != LINK_MAGIC || event.packet.version != LINK_VERSION ||
        event.packet.reserved != 0 || event.packet.session == 0 ||
        event.packet.type < PACKET_DISCOVER || event.packet.type > PACKET_FORGET) {
        return;
    }
    event.broadcast = memcmp(info->des_addr, s_broadcast, 6) == 0;
    if (!event.broadcast && memcmp(info->des_addr, s_local_mac, 6) != 0) {
        return;
    }
    memcpy(event.mac, info->src_addr, 6);
    event.rssi = info->rx_ctrl->rssi;
    event.received_ms = link_now();
    /* The Wi-Fi task never touches LVGL, flash, or peer state. */
    xQueueSend(s_receive_queue, &event, 0);
}

static esp_err_t start_radio(void)
{
    if (s_radio_ready) {
        return ESP_OK;
    }
    s_state = ESPNOW_LINK_STARTING;
    s_error = ESP_OK;
    publish_status();
    esp_err_t err;
    bool wifi_initialized = false;
    bool wifi_started = false;
    bool espnow_initialized = false;
    if (!s_network_ready) {
        err = esp_netif_init();
        if (err != ESP_OK) goto fail;
        s_network_ready = true;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) goto fail;
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    config.nvs_enable = 0;
    err = esp_wifi_init(&config);
    if (err != ESP_OK) goto fail;
    wifi_initialized = true;
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_start();
    if (err != ESP_OK) goto fail;
    wifi_started = true;
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) goto fail;
    err = esp_wifi_set_channel(ESPNOW_LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) goto fail;
    err = esp_now_init();
    if (err != ESP_OK) goto fail;
    espnow_initialized = true;
    err = esp_now_register_send_cb(sent_cb);
    if (err != ESP_OK) goto fail;
    err = esp_now_register_recv_cb(receive_cb);
    if (err != ESP_OK) goto fail;
    err = add_driver_peer(s_broadcast);
    if (err != ESP_OK) goto fail;
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (s_peers[i].used && s_peers[i].paired) {
            err = add_driver_peer(s_peers[i].mac);
            if (err != ESP_OK) goto fail;
            s_peers[i].driver_added = true;
        }
    }
    s_session = link_random();
    s_sequence = link_random();
    xSemaphoreTake(s_send_done, 0);
    xSemaphoreGive(s_send_done);
    s_radio_ready = true;
    s_state = ESPNOW_LINK_READY;
    ESP_LOGI(TAG, "Ready on channel %u", (unsigned)ESPNOW_LINK_CHANNEL);
    return ESP_OK;

fail:
    if (espnow_initialized) esp_now_deinit();
    if (wifi_started) esp_wifi_stop();
    if (wifi_initialized) esp_wifi_deinit();
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        s_peers[i].driver_added = false;
    }
    s_state = ESPNOW_LINK_ERROR;
    s_error = err;
    ESP_LOGW(TAG, "Radio initialization failed: %s", esp_err_to_name(err));
    return err;
}

static esp_err_t send_packet(link_peer_t *peer, packet_type_t type, uint32_t sequence)
{
    const link_packet_t packet = {
        .magic = LINK_MAGIC,
        .version = LINK_VERSION,
        .type = type,
        .session = s_session,
        .target_session = peer ? peer->remote_session : 0,
        .pair_token = pairing_active() ? s_pair_token : 0,
        .pair_echo = peer ? peer->remote_pair_token : 0,
        .sequence = sequence,
    };
    /* Keep at most one frame in flight; connection status still needs a PONG. */
    if (xSemaphoreTake(s_send_done, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = esp_now_send(peer ? peer->mac : s_broadcast,
                                     (const uint8_t *)&packet, sizeof(packet));
    if (err != ESP_OK) {
        xSemaphoreGive(s_send_done);
    }
    return err;
}

static void send_ping(link_peer_t *peer)
{
    const uint32_t sequence = ++s_sequence;
    if (send_packet(peer, PACKET_PING, sequence) == ESP_OK) {
        peer->sequence = sequence;
        peer->awaiting_ack = true;
    }
    peer->last_ping_ms = link_now();
}

static bool valid_pair_packet(const link_peer_t *peer, const link_packet_t *packet)
{
    return pairing_active() && packet->target_session == s_session && packet->pair_echo == s_pair_token &&
           packet->pair_token != 0 && packet->pair_token == peer->remote_pair_token &&
           packet->session == peer->remote_session;
}

static void confirm_peer(link_peer_t *peer)
{
    if (!peer->paired) {
        peer->paired = true;
        save_peers();
        ESP_LOGI(TAG, "Paired " MACSTR, MAC2STR(peer->mac));
    }
}

static void handle_receive(const receive_event_t *event)
{
    const link_packet_t *packet = &event->packet;
    if (!s_radio_ready || !valid_mac(event->mac) || link_now() - event->received_ms > LINK_TIMEOUT_MS ||
        (event->broadcast && packet->type != PACKET_DISCOVER)) {
        return;
    }
    link_peer_t *peer = find_peer(event->mac);
    if (packet->type == PACKET_DISCOVER || packet->type == PACKET_REQUEST) {
        if (!pairing_active() || packet->pair_token == 0) {
            return;
        }
        if (packet->type == PACKET_REQUEST &&
            (packet->target_session != s_session || packet->pair_echo != s_pair_token)) {
            return;
        }
        peer = ensure_peer(event->mac);
        if (!peer) {
            return;
        }
        peer->remote_session = packet->session;
        peer->remote_pair_token = packet->pair_token;
        if (packet->type == PACKET_REQUEST) {
            send_packet(peer, PACKET_ACCEPT, 0);
        }
    }
    else if (!peer) {
        return;
    }
    else if (packet->type == PACKET_ACCEPT) {
        if (!valid_pair_packet(peer, packet)) {
            return;
        }
        peer->handshake_ready = true;
        send_ping(peer);
    }
    else if (packet->type == PACKET_PING) {
        if (!peer->paired && !valid_pair_packet(peer, packet)) {
            return;
        }
        peer->remote_session = packet->session;
        send_packet(peer, PACKET_PONG, packet->sequence);
        confirm_peer(peer);
    }
    else if (packet->type == PACKET_PONG) {
        if (packet->target_session != s_session || !peer->awaiting_ack || packet->sequence != peer->sequence ||
            (!peer->paired && !valid_pair_packet(peer, packet))) {
            return;
        }
        peer->remote_session = packet->session;
        peer->awaiting_ack = false;
        peer->has_ack = true;
        peer->last_ack_ms = event->received_ms;
        confirm_peer(peer);
    }
    else if (packet->type == PACKET_FORGET) {
        if (peer->paired && packet->target_session == s_session && packet->session == peer->remote_session) {
            remove_peer(peer);
            save_peers();
        }
        return;
    }
    peer->has_signal = true;
    peer->rssi = event->rssi;
    peer->last_seen_ms = event->received_ms;
}

static void handle_command(const link_command_t *command)
{
    if (command->type == COMMAND_PAIR) {
        if (pairing_active() || start_radio() != ESP_OK) {
            return;
        }
        end_pairing();
        s_pair_token = link_random();
        s_pair_started_ms = link_now();
        s_pairing = true;
        s_discovery_interval_ms = 450 + esp_random() % 250;
        s_last_discovery_ms = s_pair_started_ms - s_discovery_interval_ms;
    }
    else if (command->type == COMMAND_CANCEL) {
        end_pairing();
    }
    else if (command->type == COMMAND_FORGET) {
        link_peer_t *peer = find_peer(command->mac);
        if (peer) {
            if (s_radio_ready) send_packet(peer, PACKET_FORGET, 0);
            remove_peer(peer);
            save_peers();
        }
    }
}

static void link_tick(void)
{
    if (!s_radio_ready) {
        return;
    }
    uint32_t now = link_now();
    if (s_pairing && !pairing_active()) {
        end_pairing();
    }
    if (s_pairing && now - s_last_discovery_ms >= s_discovery_interval_ms) {
        send_packet(NULL, PACKET_DISCOVER, 0);
        s_last_discovery_ms = link_now();
        s_discovery_interval_ms = 450 + esp_random() % 250;
    }
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        link_peer_t *peer = &s_peers[i];
        if (!peer->used) continue;
        now = link_now();
        if (peer->paired || peer->handshake_ready) {
            if (now - peer->last_ping_ms >= LINK_HEARTBEAT_MS) send_ping(peer);
        }
        if (!peer->paired && s_pairing && now - peer->last_pair_ms >= LINK_PAIR_RETRY_MS) {
            send_packet(peer, PACKET_REQUEST, 0);
            peer->last_pair_ms = link_now();
        }
    }
}

static void link_task(void *arg)
{
    (void)arg;
    load_peers();
    for (int i = 0; i < ESPNOW_LINK_MAX_PEERS; ++i) {
        if (s_peers[i].used) {
            start_radio();
            break;
        }
    }
    publish_status();
    for (;;) {
        link_command_t command;
        while (xQueueReceive(s_command_queue, &command, 0) == pdTRUE) {
            handle_command(&command);
        }
        receive_event_t event;
        for (int i = 0; i < 12 && xQueueReceive(s_receive_queue, &event, 0) == pdTRUE; ++i) {
            handle_receive(&event);
        }
        link_tick();
        publish_status();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t espnow_link_init(void)
{
    if (s_command_queue) {
        return ESP_OK;
    }
    esp_err_t err = esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) goto fail;
    s_receive_queue = xQueueCreate(24, sizeof(receive_event_t));
    s_command_queue = xQueueCreate(8, sizeof(link_command_t));
    s_send_done = xSemaphoreCreateBinary();
    if (!s_receive_queue || !s_command_queue || !s_send_done) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    xSemaphoreGive(s_send_done);
    publish_status();
    if (xTaskCreatePinnedToCore(link_task, "espnow_link", 6144, NULL, 3, NULL, 0) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    return ESP_OK;

fail:
    if (s_receive_queue) vQueueDelete(s_receive_queue);
    if (s_command_queue) vQueueDelete(s_command_queue);
    if (s_send_done) vSemaphoreDelete(s_send_done);
    s_receive_queue = NULL;
    s_command_queue = NULL;
    s_send_done = NULL;
    s_state = ESPNOW_LINK_ERROR;
    s_error = err;
    publish_status();
    return err;
}

static esp_err_t post_command(command_type_t type, const uint8_t *mac)
{
    if (!s_command_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    link_command_t command = {.type = type};
    if (mac) memcpy(command.mac, mac, 6);
    return xQueueSend(s_command_queue, &command, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t espnow_link_start_pairing(void)
{
    return post_command(COMMAND_PAIR, NULL);
}

esp_err_t espnow_link_cancel_pairing(void)
{
    return post_command(COMMAND_CANCEL, NULL);
}

esp_err_t espnow_link_forget(const uint8_t mac[6])
{
    return mac ? post_command(COMMAND_FORGET, mac) : ESP_ERR_INVALID_ARG;
}
