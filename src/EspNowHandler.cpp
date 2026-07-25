#include "EspNowHandler.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "Config.h"
#include "Globals.h"

static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // Route by packet size:
    //   sizeof(UWBData)     = 10  → Anchor B  (packed struct)
    //   sizeof(ControlData) = 12  → T-Beam remote
    if (len == sizeof(UWBData)) {
        memcpy(&g_uwb_data, incomingData, sizeof(UWBData));
        g_last_uwb_data_time = millis();
    } else if (len == sizeof(ControlData)) {
        memcpy(&espnowData, incomingData, sizeof(espnowData));
        g_last_espnow_command_time = millis();
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    AUTO_SERIAL.print("My MAC Address: ");
    AUTO_SERIAL.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        AUTO_SERIAL.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
}

void handleChannelScanning() {
    unsigned long timeSinceLastPacket = millis() - g_last_espnow_command_time;
    if (timeSinceLastPacket < SCAN_START_DELAY_MS) return;

    if (millis() - g_last_scan_time > SCAN_INTERVAL_MS) {
        g_last_scan_time = millis();
        g_current_channel++;
        if (g_current_channel > 13) g_current_channel = 1;
        esp_wifi_set_channel(g_current_channel, WIFI_SECOND_CHAN_NONE);
        AUTO_SERIAL.print("Link Lost. Hunting on Channel ");
        AUTO_SERIAL.println(g_current_channel);
    }
}