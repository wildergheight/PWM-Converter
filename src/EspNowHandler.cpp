#include "EspNowHandler.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "Config.h"
#include "Globals.h"

static uint8_t g_broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if UWB_EVAL_LOG
static unsigned long g_eval_last_pkt_time = 0; // for inter-packet gap, eval-log scope only
#endif

static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // Route by packet size:
    //   sizeof(UWBData)     = 10  → Anchor B  (packed struct)
    //   sizeof(ControlData) = 12  → T-Beam remote
    if (len == sizeof(UWBData)) {
        memcpy(&g_uwb_data, incomingData, sizeof(UWBData));
        unsigned long now = millis();
        g_last_uwb_data_time = now;

        #if UWB_EVAL_LOG
            g_uwb_eval_gap_ms        = (g_eval_last_pkt_time == 0) ? 0 : (now - g_eval_last_pkt_time);
            g_eval_last_pkt_time     = now;
            g_uwb_eval_t_ms          = now;
            g_uwb_eval_distance_m    = g_uwb_data.distance_m;
            g_uwb_eval_bearing_rad   = g_uwb_data.bearing_rad;
            g_uwb_eval_valid         = g_uwb_data.valid;
            g_uwb_eval_bearing_valid = g_uwb_data.bearing_valid;
            g_uwb_eval_pending       = true;
        #endif   

    } else if (len == sizeof(ControlData)) {
        memcpy(&espnowData, incomingData, sizeof(espnowData));
        g_last_espnow_command_time = millis();
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    // Fixed channel - matches the tag's fixed channel. No more hunting.
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    AUTO_SERIAL.print("My MAC Address: ");
    AUTO_SERIAL.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        AUTO_SERIAL.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, g_broadcast_addr, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        AUTO_SERIAL.println("Failed to add broadcast peer for status alerts");
    }
}

// void handleChannelScanning() {
//     unsigned long timeSinceLastPacket = millis() - g_last_espnow_command_time;
//     if (timeSinceLastPacket < SCAN_START_DELAY_MS) return;

//     if (millis() - g_last_scan_time > SCAN_INTERVAL_MS) {
//         g_last_scan_time = millis();
//         g_current_channel++;
//         if (g_current_channel > 13) g_current_channel = 1;
//         esp_wifi_set_channel(g_current_channel, WIFI_SECOND_CHAN_NONE);
//         AUTO_SERIAL.print("Link Lost. Hunting on Channel ");
//         AUTO_SERIAL.println(g_current_channel);
//     }
// }

void sendStatusAlert() {
    StatusAlert status;
    status.control_state  = (uint8_t)g_control_state;
    status.distance_fault = g_distance_fault;
    status.bearing_fault  = g_bearing_fault;
    status.lvc_active     = g_lvc_activated;
    status.bus_voltage    = g_bus_voltage;

    esp_now_send(g_broadcast_addr, (uint8_t *)&status, sizeof(status));
}

void checkUwbEvalLog() {
#if UWB_EVAL_LOG
    if (!g_uwb_eval_pending) return;
    g_uwb_eval_pending = false;

    // UWBLOG,<t_ms>,<gap_ms>,<distance_m>,<bearing_rad>,<valid>,<bearing_valid>
    AUTO_SERIAL.print("UWBLOG,");
    AUTO_SERIAL.print(g_uwb_eval_t_ms);           AUTO_SERIAL.print(",");
    AUTO_SERIAL.print(g_uwb_eval_gap_ms);         AUTO_SERIAL.print(",");
    AUTO_SERIAL.print(g_uwb_eval_distance_m, 4);  AUTO_SERIAL.print(",");
    AUTO_SERIAL.print(g_uwb_eval_bearing_rad, 5); AUTO_SERIAL.print(",");
    AUTO_SERIAL.print(g_uwb_eval_valid ? 1 : 0);  AUTO_SERIAL.print(",");
    AUTO_SERIAL.println(g_uwb_eval_bearing_valid ? 1 : 0);
#endif
}