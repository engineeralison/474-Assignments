// Filename: Part2
// Authors: Alison Tea and Shahnaz Mohideen
// Date:
// Desciption: Implements interrupts and Interrupt Service Routine

#include "esp_mac.h"  // required - exposes esp_mac_type_t values
void setup() {
 Serial0.begin(115200);


 Serial0.print("Wi-Fi Station (using 'esp_efuse_mac_get_default')\t");
 Serial0.println(getDefaultMacAddress());


 Serial0.print("WiFi Station (using 'esp_read_mac')\t\t\t");
 Serial0.println(getInterfaceMacAddress(ESP_MAC_WIFI_STA));


 Serial0.print("WiFi Soft-AP (using 'esp_read_mac')\t\t\t");
 Serial0.println(getInterfaceMacAddress(ESP_MAC_WIFI_SOFTAP));
}


void loop() { /* Nothing in loop */ }


String getDefaultMacAddress() {


 String mac = "";


 unsigned char mac_base[6] = {0};


 if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
   char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
   sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
   mac = buffer;
 }
 return mac;
}


String getInterfaceMacAddress(esp_mac_type_t interface) {


 String mac = "";


 unsigned char mac_base[6] = {0};


 if (esp_read_mac(mac_base, interface) == ESP_OK) {
   char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
   sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
   mac = buffer;
 }


 return mac;
}
