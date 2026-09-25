/*
 * =============================================================================
 * secrets.example.h — ESP32-CAM Vision Middleman Node Template
 * =============================================================================
 * Rename this file to secrets.h and fill in your credentials before flashing.
 * =============================================================================
 */
#ifndef SECRETS_H
#define SECRETS_H

// --- Wi-Fi Credentials (Must match your laptop network) ---
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

// --- Laptop / Host Server IP running server_web_scraper.py ---
#define LAPTOP_IP       "192.168.1.100" // Replace with your laptop Wi-Fi IP
#define LAPTOP_PORT     5000

// --- UDP Ports for ESP32 <-> ESP32-CAM Wi-Fi Router Communication ---
#define UDP_MAIN_RX_PORT 8888
#define UDP_CAM_RX_PORT  8889

#endif // SECRETS_H
