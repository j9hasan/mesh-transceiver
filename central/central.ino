#include "painlessMesh.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

Scheduler userScheduler;
painlessMesh mesh;

#define i2c_Address 0x3c  // Set the I2C address of your OLED
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void dispUpdate(String title, String msg);

#define LED_PIN 2

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
  dispUpdate("Message from Node", msg);
  pulseLed();
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> New Connection, nodeId = %u\n", nodeId);
  dispUpdate("New Connection", String(nodeId));
}

void changedConnectionCallback() {
  Serial.println("Changed connections");
  dispUpdate("Network", "Connections Changed");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
  dispUpdate("Time Sync", "Offset: " + String(offset));
}

void setup() {
  Serial.begin(115200);
  delay(250);
  display.begin(i2c_Address, true);
  display.display();
  delay(1000);
  display.clearDisplay();

  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  mesh.update();
}

void dispUpdate(String title, String msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Display the title and message with minimal display space
  display.setCursor(0, 0);
  display.print(title);

  display.setCursor(0, 15);  // Print message below the title
  display.print(msg);

  display.display();

  // Display the RSSI at the bottom of the screen
  int32_t rssi = WiFi.RSSI();                // Get the current RSSI
  display.setCursor(0, SCREEN_HEIGHT - 16);  // Position at the bottom of the display
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.print("RSSI: ");
  display.print(String(rssi));
  display.print(" dBm");

  // Update the display
  display.display();
  Serial.println(rssi);
}
void pulseLed() {
  digitalWrite(LED_PIN, HIGH);  // Turn on LED
  delay(100);                  // Wait for 100 milliseconds (adjust as needed)
  digitalWrite(LED_PIN, LOW);   // Turn off LED
}
