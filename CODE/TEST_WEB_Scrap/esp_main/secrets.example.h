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

// --- ESP32-CAM MAC Address ---
// Default: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} (Broadcast)
#define ESPCAM_MAC      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// --- ESP-NOW Channel ---
#define ESPNOW_CHANNEL  1

#endif // SECRETS_H
