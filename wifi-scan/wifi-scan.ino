#include "WiFi.h"

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Ensure no active connection
    delay(100);

    Serial.println("Scanning for WiFi networks...");
}

void loop() {
    int numNetworks = WiFi.scanNetworks();

    if (numNetworks == 0) {
        Serial.println("No networks found");
    } else {
        Serial.printf("Found %d networks:\n", numNetworks);
        for (int i = 0; i < numNetworks; i++) {
            Serial.printf("%d: %s (Signal: %d dBm, Encryption: %s)\n",
                          i + 1,
                          WiFi.SSID(i).c_str(),
                          WiFi.RSSI(i),
                          (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured");
        }
    }

    Serial.println("\nWaiting 5 seconds before next scan...\n");
    delay(5000);
}