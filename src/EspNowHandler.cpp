#include "EspNowHandler.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel
#include "Config.h"
#include "Globals.h"

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&espnowData, incomingData, sizeof(espnowData));
    g_last_espnow_command_time = millis();
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

    // 1. If we've heard from the remote recently, STAY on this channel.
    if (timeSinceLastPacket < SCAN_START_DELAY_MS) {
        return;
    }

    // 2. If we've been silent for more than SCAN_START_DELAY_MS, start hunting.
    if (millis() - g_last_scan_time > SCAN_INTERVAL_MS) {
        g_last_scan_time = millis();

        g_current_channel++;
        if (g_current_channel > 13) g_current_channel = 1;

        esp_wifi_set_channel(g_current_channel, WIFI_SECOND_CHAN_NONE);

        AUTO_SERIAL.print("Link Lost. Hunting on Channel ");
        AUTO_SERIAL.println(g_current_channel);
    }
}
