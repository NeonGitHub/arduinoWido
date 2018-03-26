#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "utility/socket.h"
  
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   7  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
SPI_CLOCK_DIVIDER); // you can change this clock speed
  
  
#define WLAN_SSID       "GoodLuck"           //这里填写你的 WIFI  名称
#define WLAN_PASS       "zJx@19941211"            //这里填写你的 WIFI 密码 
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2
  
#define LISTEN_PORT           80   // What TCP port to listen on for connections.
  
Adafruit_CC3000_Server webServer(LISTEN_PORT);
boolean led_state;
//
void setup(void) {
        //简单起见，我们只用板载的 13pin 上的 LED 演示
        pinMode (13, OUTPUT);
           //默认是灭的
        digitalWrite (13, LOW);
        
           //使用串口输出Debug信息
        Serial.begin(115200);
        Serial.println(F("Hello, CC3000!\n"));
        //while (!Serial);
        //Serial.println ("Input any key to start:");
        //while (!Serial.available ());
           //输出当前可用内存
        Serial.print("Free RAM: ");
        Serial.println(getFreeRam(), DEC);
  
        /* Initialise the module */
        Serial.println(F("\nInitializing..."));
        if (!cc3000.begin()) {
                Serial.println(F("Couldn't begin()! Check your wiring?"));
                while(1);
        }
  
        Serial.print(F("\nAttempting to connect to "));
        Serial.println(WLAN_SSID);
        if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
                Serial.println(F("Failed!"));
                while(1);
        }
  
        Serial.println(F("Connected!"));
  
           //使用 DHCP 分配的IP
        Serial.println(F("Request DHCP"));
        while (!cc3000.checkDHCP()) {
                delay(100); // ToDo: Insert a DHCP timeout!
        } 
           
        //显示当前的IP信息
        /* Display the IP address DNS, Gateway, etc. */
        while (! displayConnectionDetails()) {
                delay(1000);
        }
  
        // Start listening for connections
        webServer.begin();
        Serial.println(F("Listening for connections..."));
}
  
//
void loop(void) {
        // Try to get a client which is connected.
        Adafruit_CC3000_ClientRef client = webServer.available();
        if (client) {
                   //处理输入的 GET 信息，对于 GET 方法来说，Url中既有传递的信息
                processInput (client);
                      //对发送 HTTP 请求的浏览器发送HTTP代码
                sendWebPage (client);
        }
        client.close();
}
  
//分析收到的 GET 方法的参数
void processInput (Adafruit_CC3000_ClientRef client) {
        char databuffer[45];
      //安全起见，保证截断
       databuffer[44]=’\0’;
        while (client.available ()) {
                client.read (databuffer, 40);
      
        //下面这个代码是查找PC端发送的数据中的换行，以此作为字符串的结尾
                char* sub = strchr (databuffer, '\r');
                if (sub > 0)
                        *sub = '\0';
                Serial.println (databuffer);
                      
                //下面是解析 GET 方法提供的参数
       //如果是 open 命令，那么点亮 LED
                if (strstr (databuffer, "open") != 0) {
                        Serial.println (F("clicked open"));
                        digitalWrite (13, HIGH);
                        led_state = true;
                }
           //如果是 close 命令，那么熄灭 LED
                else if (strstr (databuffer, "close") != 0) {
                        Serial.println (F("clicked close"));
                        digitalWrite (13, LOW);
                        led_state = false;
                }
                break;
        }
}
  
void sendWebPage (Adafruit_CC3000_ClientRef client) {
        //为了节省空间，这里只发送简单的提示字符
        webServer.write ("Waiting for command");
        delay (20);
        client.close();
}
  
//输出当前WIFI 设备通过 DHCP 取得的基本信息
bool displayConnectionDetails(void) {
        uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
        if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv)) {
                Serial.println(F("Unable to retrieve the IP Address!\r\n"));
                return false;
        }
        else {
                Serial.print(F("\nIP Addr: "));
                cc3000.printIPdotsRev(ipAddress);
                Serial.print(F("\nNetmask: "));
                cc3000.printIPdotsRev(netmask);
                Serial.print(F("\nGateway: "));
                cc3000.printIPdotsRev(gateway);
                Serial.print(F("\nDHCPsrv: "));
                cc3000.printIPdotsRev(dhcpserv);
                Serial.print(F("\nDNSserv: "));
                cc3000.printIPdotsRev(dnsserv);
                Serial.println();
                return true;
        }
}