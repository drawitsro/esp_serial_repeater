#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

// Define this on the first ESP32, comment out for the second ESP32
//#define FIRST_ESP
int baudRate = 115200;
const char *ssid = "MERCUSYS_D410";
const char *password = "89459713";

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

bool wifi_connected = false;
bool udpStarted = false;
char incomingPacket[255];
unsigned long delay_micros = 0;

const int BUFFER_SIZE = 64; // Buffer size for batching
char commandBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastBatchTime = 0;
const unsigned long BATCH_INTERVAL = 1; // interval in ms for batching

void print_wifi_info() {
  Serial.println("(Connected to the WiFi network)");
  Serial.print("(IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println(")");

  Serial.print("(Gateway: ");
  Serial.print(WiFi.gatewayIP());
  Serial.println(")");

  Serial.print("(Subnet Mask: ");
  Serial.print(WiFi.subnetMask());
  Serial.println(")");

  Serial.print("(DNS 1: ");
  Serial.print(WiFi.dnsIP(0));
  Serial.println(")");

  Serial.print("(DNS 2: ");
  Serial.print(WiFi.dnsIP(1));
  Serial.println(")");
}

// Function to convert wl_status_t to string
const char *getStatusString(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "UNKNOWN";
  }
}

void connectToWiFi() {
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);
  wl_status_t status = WL_NO_SHIELD;
  for (int i = 0; i < 10; ++i) {
    status = WiFi.status();
    if (status == WL_CONNECTED) {
      break;
    }
    delay(1000);
    Serial.println("(Connecting to WiFi...)");
  }
  if (status != WL_CONNECTED) {
    Serial.print("(Error - Failed to connect to WiFi - Status: ");
    Serial.print(getStatusString(status));
    Serial.println(")");
    return;
  }
  wifi_connected = true;
  Serial.println("(Connected to WiFi)");
  print_wifi_info();
}

void init_udp() {
  if (udpStarted) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    Serial.println("(Trying to start UDP...)");
    udpStarted = udp.begin(udpPort);
    if (udpStarted) {
      Serial.println("(UDP started)");
      Serial.println("(ok)");
      break;
    } else {
      delay(1000);
      Serial.println("(Error - Failed to start UDP)");
    }
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("(WiFi connection lost. Attempting to reconnect...)");
    wifi_connected = false;
    connectToWiFi();
    init_udp();
  }
}

void flushBuffer() {
  if (bufferIndex > 0) {
    if (udp.beginPacket(remote_IP, udpPort)) {
      udp.write((const uint8_t *)commandBuffer, bufferIndex);
      if (!udp.endPacket()) {
        Serial.println("(Error ending UDP packet)");
      }
    } else {
      Serial.println("(Error starting UDP packet)");
    }
    bufferIndex = 0; // Reset buffer index after flushing
  }
}

void setup() {
  Serial.begin(baudRate);
  Serial.println("(Waiting for serial connection...)");
  while (!Serial) {
    // Wait for serial connection
    delay(100);
  }

  connectToWiFi();
  init_udp();

  // compute the delay in microseconds for serial based on baud rate
  delay_micros = 1000000 / baudRate + 1;
}

void preciseDelay(unsigned long microseconds) {
  unsigned long start = micros();
  while (micros() - start < microseconds);
}

void loop() {
  checkWiFiConnection();

  // Handle serial input and store in buffer
  while (Serial.available() && bufferIndex < BUFFER_SIZE - 1) {
    char c = Serial.read();
    commandBuffer[bufferIndex++] = c;
    if (bufferIndex >= BUFFER_SIZE - 1 || (millis() - lastBatchTime) >= BATCH_INTERVAL) {
      flushBuffer();
      lastBatchTime = millis();
    }
  }

  // Periodically flush buffer based on interval
  if ((millis() - lastBatchTime) >= BATCH_INTERVAL) {
    flushBuffer();
    lastBatchTime = millis();
  }

  // Handle UDP input and send to serial
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate the received string
      Serial.print(incomingPacket); // Use print instead of println
    }
  }

  // Add a small delay to avoid overwhelming the serial and network buffers
  //preciseDelay(delay_micros);
}
