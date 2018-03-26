#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"
 
#define WiDo_IRQ   7
#define WiDo_VBAT  5
#define WiDo_CS    10
 
Adafruit_CC3000 WiDo = Adafruit_CC3000(WiDo_CS, WiDo_IRQ, WiDo_VBAT,
SPI_CLOCK_DIVIDER); 
 
#define WLAN_SSID       "TP-LINK_33701E"  
#define WLAN_PASS       "19890226"
#define WLAN_SECURITY   WLAN_SEC_WPA2
 
 
#define TIMEOUT_MS      2000
 
void setup(){
 
  Serial.begin(115200);
 
  Serial.println(F("Hello, Wido!\n")); 
  Serial.println(F("\nInitialising the CC3000 ..."));
  if (!WiDo.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    while(1);
  }
 
  if (!WiDo.connectToAP(WLAN_SSID,WLAN_PASS,WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
 
  Serial.println(F("Connected!"));
  Serial.println(F("Request DHCP"));
  while (!WiDo.checkDHCP())
  {
    delay(100); 
  }  
}
 
 
void loop(){
  static Adafruit_CC3000_Client client;
 
  if(client.connected()){   
 
    client.fastrprintln("GET /twido/hi.txt HTTP/1.1");
    client.fastrprintln("Host: 192.168.1.102");
    client.fastrprintln("User-Agent: arduino-ethernet");
    client.fastrprintln("Connection: close");
    client.fastrprint(F("\r\n"));
    client.fastrprint(F("\r\n"));
     
    Serial.println();
 
    unsigned long lastRead = millis();
    while (client.connected() && (millis() - lastRead < TIMEOUT_MS)) {
      while (client.available()) {
        char c = client.read();
        Serial.print(c);
        lastRead = millis();
      }
    }
    client.close();
  }else{
    uint32_t ip = WiDo.IP2U32(192,168,1,102);
    client = WiDo.connectTCP(ip,80);
    Serial.println("Connecting IoT Server...");
  }  
  delay(5000);
}