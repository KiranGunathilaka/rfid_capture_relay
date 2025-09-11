#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Start WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // Now you can safely get the MAC addresses
  Serial.print("Station MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
}

void loop() {
  // nothing needed here
}
