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


SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

struct __attribute__((packed)) Measurement{
  uint32_t ms;
  int16_t rssi;
};
std::vector<struct Measurement> data;
uint64_t chipid;
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
  struct Measurement m;

  for(int i = 0; i < N; i++){
    m.ms = millis();
    m.rssi = -i;
    data.push_back(m);
    delay(15);
  }

  Serial.printf("Free heap after generate_data: %lu\n",ESP.getFreeHeap());
}

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
  WiFi.disconnect(true,true);
  VextOFF();
  SPI.end();
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_TIME_SECS*1000*(uint64_t)1000);
  esp_deep_sleep_start();
}

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
        generate_data();
        if(connect_and_send_data() < 0){
          double_output("Socket connection issue!");
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
  Serial.printf("ChipID = 0x%012llX\n",chipid);
  Serial.printf("Free heap at start: %lu\n",ESP.getFreeHeap());
  xTaskCreateUniversal(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle, CONFIG_ARDUINO_RUNNING_CORE);
  VextON();
  delay(100);

  factory_display.init();
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);

  pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

  WiFi.begin(WIFI_SSID,WIFI_PASS);
  prior_wifi_status = WiFi.status();
}

void loop() {
  String cid_str, local_ip_str;

  int new_wifi_status = WiFi.status();
  if(new_wifi_status != prior_wifi_status)
  switch (new_wifi_status) {
    case WL_IDLE_STATUS:
      // Temp status, wait til change occurs
      break;
    case WL_NO_SSID_AVAIL:
      double_output("SSID not found");
      break;
    case WL_CONNECT_FAILED:
      double_output("Failed - WiFi not connected!");
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
      cid_str = String(string_format("%012llX",chipid).c_str());
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

  Mcu.timerhandler();
  delay(50);
}
