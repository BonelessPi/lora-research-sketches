/*
Proof of concept for transmitting back to a TCP server
*/

#include <Arduino.h>
#include <memory>
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <vector>
#include <WiFi.h>
#include "HT_SSD1306Wire.h"
#include <ESP32_Mcu.h>

#define USERKEY_PIN 0
#define HOLD_THRESHOLD_MS 3000
#define DEEPSLEEP_TIME_SECS 600

#define MAGIC_NUM 0x61462cdf
#include "secrets.h"


SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

struct __attribute__((packed)) Measurement{
  uint32_t ms;
  int16_t rssi;

  Measurement(uint32_t a, int16_t b):ms(a),rssi(b){}
};
std::vector<struct Measurement> data;
uint64_t chipid;
String cid_str;
int prior_wifi_status;


uint64_t get_mac_reversed_byteorder(){
  uint64_t mac = ESP.getEfuseMac();
  uint64_t res = 0;
  for(int i=0; i<6; i++){
    res <<= 8;
    res |= mac & 0xff;
    mac >>= 8;
  }
  // res: 2 zero bytes, 3 constant bytes, 3 variable bytes
  return res;
}

// Exists because std::format in <format> is in c++20 while arduino is c++11?
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args ){
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if(size_s <= 0){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

void generate_data(){
  int N = 10;
  data.reserve(data.size()+N);

  for(int i = 0; i < N; i++){
    data.emplace_back(millis(),-i);
    delay(15);
  }

  Serial.printf("Free heap after generate_data: %lu\n",ESP.getFreeHeap());
}

// TODO speed up the sending
int connect_and_send_data(){
  int ret = 0;
  struct sockaddr_in serv_addr;
  uint32_t local_mstime;

  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  Serial.printf("sockfd = %d\n",sockfd);
  if(sockfd < 0) {
    Serial.println("Socket creation error");
    perror("socket");
    ret = -1;
    goto casd_cleanup;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(DEST_PORT);
  if(inet_pton(AF_INET,DEST_IP,&serv_addr.sin_addr) <= 0) {
    Serial.println("Invalid address/ Address not supported");
    ret = -1;
    goto casd_cleanup;
  }
  if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    Serial.println("Connection Failed");
    perror("connect");
    ret = -1;
    goto casd_cleanup;
  }
  Serial.println("Connected");
  
  uint32_t header[5];
  local_mstime = millis();
  header[0] = htonl(MAGIC_NUM);
  header[1] = htonl(chipid >> 32);
  header[2] = htonl(chipid & 0xffffffff);
  header[3] = htonl(local_mstime);
  header[4] = htonl(data.size());
  send(sockfd,header,20,0);

  for(struct Measurement m : data){
    uint32_t ms = htonl(m.ms);
    uint16_t rssi = htons(m.rssi);
    send(sockfd,&ms,sizeof(ms),0);
    send(sockfd,&rssi,sizeof(rssi),0);
  }
  Serial.println("Sending finished");

  casd_cleanup:
  if(sockfd >= 0){
    close(sockfd);
  }
  return ret;
}

// Turn on the display power?
void VextON(){
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

// Turn off the display power?
void VextOFF(){
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

// Put the chip into deep sleep (seems to act like a delayed reset?)
void intodeepsleep(){
  Serial.println("into deep sleep");
  WiFi.disconnect(true);
  VextOFF();
  SPI.end();
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_TIME_SECS*1000*(uint64_t)1000);
  esp_deep_sleep_start();
}

// TODO look into this
/*
21:35:41.093 -> key down at: 295637 ms
21:35:41.412 -> Free heap after generate_data: 242056
21:35:41.413 -> sockfd = 48
21:35:41.583 -> Guru Meditation Error: Core  1 panic'ed (Unhandled debug exception). 
21:35:41.615 -> Debug exception reason: Stack canary watchpoint triggered (checkUserkey1Ta) 
21:35:41.615 -> Core  1 register dump:
21:35:41.615 -> PC      : 0x40386e0b  PS      : 0x00060736  A0      : 0x4037f6fe  A1      : 0x3fcab990  
21:35:41.615 -> A2      : 0x00060723  A3      : 0x40056f5c  A4      : 0x00060720  A5      : 0x403793dc  
21:35:41.648 -> A6      : 0x00000001  A7      : 0x00000024  A8      : 0x8209e010  A9      : 0x3fcabb30
21:35:41.648 -> A10     : 0x00000640  A11     : 0x3fcef91c  A12     : 0x403799e9  A13     : 0x00060923  
21:35:41.648 -> A14     : 0x3fcacbc4  A15     : 0x0000abab  SAR     : 0x0000001f  EXCCAUSE: 0x00000001  
21:35:41.648 -> EXCVADDR: 0x00000000  LBEG    : 0x40056f5c  LEND    : 0x40056f72  LCOUNT  : 0xffffffff  
21:35:41.680 -> 
21:35:41.680 -> 
21:35:41.680 -> Backtrace: 0x40386e08:0x3fcab990 0x4037f6fb:0x3fcaba60 0x40383d85:0x3fcaba80 0x40383f25:0x3fcabaa0 0x40377a87:0x3fcabac0 0x40377aa9:0x3fcabaf0 0x4037764d:0x3fcabb10 0x403793e8:0x3fcabb30 0x4209e00d:0x3fcabb50 0x420a02d0:0x3fcabb80 0x400398b5:0x3fcabbb0 0x4209dbf1:0x3fcabbd0 0x4209dd3d:0x3fcabc00 0x4206acb9:0x3fcabc40 0x42047e0e:0x3fcabc60 0x4209a031:0x3fcabc80 0x4204d45a:0x3fcabca0 0x42031009:0x3fcabcc0 0x4202b2a5:0x3fcabce0 0x4202b546:0x3fcabd10 0x4202c465:0x3fcabd40 0x4202c4a1:0x3fcabd80 0x4202c4c9:0x3fcabdb0 0x42028818:0x3fcabde0 0x420248fb:0x3fcabe20 0x4202572f:0x3fcabe40 0x42025772:0x3fcabe70 0x420339ac:0x3fcabe90 0x42034192:0x3fcabec0 0x42022858:0x3fcabee0 0x42032f1d:0x3fcabf00 0x42032fd5:0x3fcabf20 0x42021418:0x3fcabf70 0x42020485:0x3fcabfc0 0x4208ba6e:0x3fcabfe0 0x42003421:0x3fcac000 0x42003c40:0x3fcac050 0x4037f30e:0x3fcac070
21:35:41.751 -> 
21:35:41.751 -> 
21:35:41.751 -> 
21:35:41.751 -> 
21:35:41.751 -> ELF file SHA256: 2d4d67d7d
*/
TaskHandle_t checkUserkey1kHandle = NULL;
// Function to handle the PRG button in a different task
void checkUserkey(void *pvParameters){
  uint32_t keydowntime;
  pinMode(USERKEY_PIN,INPUT);
  while(1){
    if(digitalRead(USERKEY_PIN) == 0){
      keydowntime = millis();
      Serial.printf("key down at: %lu ms\n",keydowntime);
      delay(10);
      while(digitalRead(USERKEY_PIN) == 0){
        if((millis()-keydowntime) > HOLD_THRESHOLD_MS){
          break;
        }
      }
      if((millis()-keydowntime) > HOLD_THRESHOLD_MS){
        // Hold button puts device into sleep
        intodeepsleep(); 
      }
      else{
        if(WiFi.status() == WL_CONNECTED){
          generate_data();
          if(connect_and_send_data() < 0){
            double_output("Socket connection issue!");
          }
        }
        else{
          double_output("No WiFi, no send");
        }
      }
    }
  }
}

void double_output(const char *str){
  Serial.println(str);
  factory_display.clear();
  factory_display.drawString(64,32,str);
  factory_display.display();
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  chipid = get_mac_reversed_byteorder();
  cid_str = String(string_format("%012llX",chipid).c_str());
  Serial.printf("ChipID = 0x%012llX\n",chipid);
  Serial.printf("Free heap at start: %lu\n",ESP.getFreeHeap());
  xTaskCreateUniversal(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle, CONFIG_ARDUINO_RUNNING_CORE);
  VextON();
  delay(100);

  factory_display.init();
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  factory_display.drawString(64, 24, ESP.getSketchMD5().substring(0,16));
  factory_display.drawString(64, 40, ESP.getSketchMD5().substring(16,32));
  factory_display.display();
  
  pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

  // TODO figure out why sometimes it doesn't reconnect
  WiFi.disconnect(true);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  WiFi.setAutoReconnect(true);
  prior_wifi_status = -999;
  delay(1000);
}

void loop() {
  String local_ip_str;

  int new_wifi_status = WiFi.status();
  if(new_wifi_status != prior_wifi_status){
    switch (new_wifi_status) {
      case WL_IDLE_STATUS:
        // Temp status, wait til change occurs
        break;
      case WL_NO_SSID_AVAIL:
        double_output("SSID not found");
        break;
      case WL_CONNECT_FAILED:
        double_output("WiFi failed");
        break;
      case WL_CONNECTION_LOST:
        double_output("Connection was lost");
        break;
      case WL_SCAN_COMPLETED:
        double_output("Scan is completed");
        break;
      case WL_DISCONNECTED:
        double_output("WiFi is disconnected");
        break;
      case WL_CONNECTED:
        Serial.println("WiFi is connected!");
        // TODO: fix mac endian issue here and hotter colder
        local_ip_str = WiFi.localIP().toString();
        factory_display.clear();
        factory_display.drawString(64, 16, cid_str);
        factory_display.drawString(64, 48, local_ip_str);
        factory_display.display();
        break;
      default:
        Serial.print("WiFi Status: ");
        Serial.println(WiFi.status());
        break;
    }
    prior_wifi_status = new_wifi_status;
  }

  Mcu.timerhandler();
  delay(50);
}
