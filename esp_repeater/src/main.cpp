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

void print_wifi_info()
{
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
const char *getStatusString(wl_status_t status)
{
  switch (status)
  {
  case WL_NO_SHIELD:
    return "WL_NO_SHIELD";
  case WL_IDLE_STATUS:
    return "WL_IDLE_STATUS";
  case WL_NO_SSID_AVAIL:
    return "WL_NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED:
    return "WL_SCAN_COMPLETED";
  case WL_CONNECTED:
    return "WL_CONNECTED";
  case WL_CONNECT_FAILED:
    return "WL_CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "WL_CONNECTION_LOST";
  case WL_DISCONNECTED:
    return "WL_DISCONNECTED";
  default:
    return "UNKNOWN";
  }
}

void connectToWiFi()
{
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);
  wl_status_t status = WL_NO_SHIELD;
  for (int i = 0; i < 10; ++i)
  {
    status = WiFi.status();
    if (status == WL_CONNECTED)
    {
      break;
    }
    delay(1000);
    Serial.println("(Connecting to WiFi...)");
  }
  if (status != WL_CONNECTED)
  {
    Serial.print("(Error - Failed to connect to WiFi - Status: ");
    Serial.print(getStatusString(status));
    Serial.println(")");
    return;
  }
  wifi_connected = true;
  Serial.println("(Connected to WiFi)");
  print_wifi_info();
}

void setup()
{
  Serial.begin(baudRate);
  Serial.println("(Waiting for serial connection...)");
}

void loop()
{
  if (Serial.available())
  {
    if (!wifi_connected)
    {
      connectToWiFi();

      // Initialize UDP
      for (int i = 0; i < 3; ++i)
      {
        udpStarted = udp.begin(udpPort);
        if (udpStarted)
        {
          break;
        }
        else
        {
          delay(1000);
        }
      }
      if (!udpStarted)
      {
        Serial.println("(Error - Failed to start UDP)");
      }
      else
      {
        Serial.println("(UDP started)");
      }
    }

    // Only proceed with the main loop if the serial is connected
    if (wifi_connected && udpStarted)
    {
      // First ESP32: Read from USB serial and send to second ESP32 via UDP
      if (Serial.available())
      {
        String data = Serial.readString();
        if (!udp.beginPacket(remote_IP, udpPort))
        {
          Serial.println("(Error starting UDP packet)");
          return;
        }
        udp.write((const uint8_t *)data.c_str(), data.length());
        if (!udp.endPacket())
        {
          Serial.println("(Error ending UDP packet)");
        }
      }

      // Receive data from the other ESP32 and send to USB serial
      int packetSize = udp.parsePacket();
      if (packetSize)
      {
        int len = udp.read(incomingPacket, 255);
        if (len > 0)
        {
          incomingPacket[len] = 0;
          Serial.println(incomingPacket);
        }
      }

      // Add a small delay to avoid overwhelming the serial and network buffers
      delay(10);
    }
  }
}
