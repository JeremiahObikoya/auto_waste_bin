/*
 * =============================================================================
 * secrets.example.h — Main ESP32 Navigation Node Template
 * =============================================================================
 * Rename this file to secrets.h and fill in your credentials before flashing.
 * =============================================================================
 */
#ifndef SECRETS_H
#define SECRETS_H

// --- SoftAP Credentials for the Web Dashboard ---
#define AP_SSID         "Auto_Waste_Bin_AP"
#define AP_PASSWORD     "12345678"

// --- Home / Router Wi-Fi Credentials (same as ESP32-CAM) ---
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

// --- UDP Ports for ESP32 <-> ESP32-CAM Wi-Fi Router Communication ---
#define UDP_MAIN_RX_PORT 8888
#define UDP_CAM_RX_PORT  8889

#endif // SECRETS_H
