// #include <WiFi.h>

// const char* ssid = "MERCUSYS_D410";
// const char* password = "89459713";

// void setup() {
//   Serial.begin(9600);
//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//   }

//   Serial.println("Connected to the WiFi network");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());
//   Serial.print("Gateway: ");
//   Serial.println(WiFi.gatewayIP());
//   Serial.print("Subnet Mask: ");
//   Serial.println(WiFi.subnetMask());
//   Serial.print("DNS 1: ");
//   Serial.println(WiFi.dnsIP(0));
//   Serial.print("DNS 2: ");
//   Serial.println(WiFi.dnsIP(1));
// }

// void loop() {
//   // Nothing here
// }
#include <WiFi.h>
#include <WiFiUdp.h>

// Define this on the first ESP32, comment out for the second ESP32
//#define FIRST_ESP

const char* ssid = "MERCUSYS_D410";
const char* password = "89459713";

// Common Network Configuration
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 1, 1);
IPAddress secondaryDNS(0, 0, 0, 0);

IPAddress local_IP1(192, 168, 1, 250); // First ESP32
IPAddress local_IP2(192, 168, 1, 251); // Second ESP32

// IP addresses for the ESP32 boards
#ifdef FIRST_ESP
IPAddress local_IP = local_IP1;
IPAddress remote_IP = local_IP2;
#else
IPAddress local_IP = local_IP2;
IPAddress remote_IP = local_IP1;
#endif

const uint16_t udpPort = 4210;
WiFiUDP udp;

bool serialConnected = false;

void connectToWiFi() {
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(9600);

  Serial.println("Waiting for serial connection...");
}

void loop() {
  // Check if serial is connected
  if (!serialConnected && Serial) {
    serialConnected = true;
    Serial.println("Serial connected!");
    connectToWiFi();

    // Initialize UDP
    if (!udp.begin(udpPort)) {
      Serial.println("Failed to start UDP");
    } else {
      Serial.println("UDP started");
    }
  }

  // Only proceed with the main loop if the serial is connected
  if (serialConnected) {
    // First ESP32: Read from USB serial and send to second ESP32 via UDP
    if (Serial.available()) {
      String data = Serial.readString();
      if (!udp.beginPacket(remote_IP, udpPort)) {
        Serial.println("Error starting UDP packet");
        return;
      }
      udp.write((const uint8_t *)data.c_str(), data.length());
      if (!udp.endPacket()) {
        Serial.println("Error ending UDP packet");
      }
    }

    // Receive data from the other ESP32 and send to USB serial
    int packetSize = udp.parsePacket();
    if (packetSize) {
      char incomingPacket[255];
      int len = udp.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = 0;
        Serial.print("Received: ");
        Serial.println(incomingPacket);
      }
    }

    // Add a small delay to avoid overwhelming the serial and network buffers
    delay(100);
  }
}

