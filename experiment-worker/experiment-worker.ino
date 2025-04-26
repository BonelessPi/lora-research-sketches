/*
This program will:
* Keep a connection to the runner
* Record the rssi of incoming packets
* Transmit to computer the recorded data over TCP periodically
*/

#include <Arduino.h>
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <WiFi.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"
#include "time.h"

#define USERKEY_PIN 0
#define HOLD_THRESHOLD_MS 3000
#define DEEPSLEEP_TIME_SECS 600

#define EXPERIMENT_PROTOCOL_MAGIC_NUM 0xba41dba8
#include "../secrets.h"
#define DESIRED_AUTOSEND_NUMBER 20
#define MEASUREMENT_BUF_CAPACITY 200
#define MEASUREMENT_BUF_SLACK 5
/* The slack is the min number of free spaces between tail and head;
The slack only has to be long enough so the RxDone callback doesn't begin overwriting memory at
  the "head" index in transmit_data_home(), like when the socket connect call takes a while.
*/

// LoRa defines
#define RF_FREQUENCY 915000000 // Hz
#define LOCAL_TX_POWER 10        // dBm
#define LORA_BANDWIDTH 0          // [0: 125 kHz,
                                  //  1: 250 kHz,
                                  //  2: 500 kHz,
                                  //  3: Reserved]
#define LORA_SPREADING_FACTOR 7   // [SF7..SF12]
#define LORA_CODINGRATE 1         // [1: 4/5,
                                  //  2: 4/6,
                                  //  3: 4/7,
                                  //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define LORA_PROTOCOL_MAGIC_NUMBER 0xb561482c
#define LORA_PACKET_SIZE 32
#define RX_TIMEOUT_MS 2000
#define TX_PERIOD_MS 200
#define RSSI_BUFFER_SIZE 5
#define LOCAL_ANTENNA_GAIN 5


SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

typedef enum{
  REACHING,
  CONNECTED,
  RECORDING,
  STOPPED,
  BLIND_TX
} States_t;

#define TIMESTAMP_TYPE double
struct __attribute__((packed)) Measurement{
  TIMESTAMP_TYPE ts;
  int16_t rssi;

  Measurement(TIMESTAMP_TYPE a, int16_t b):ts(a),rssi(b){}
  Measurement():ts(0),rssi(0){}
};
Measurement measurement_buf[MEASUREMENT_BUF_CAPACITY];
int measurement_buf_head = 0, measurement_buf_tail = 0;
int get_num_measurements(){
  return (MEASUREMENT_BUF_CAPACITY + measurement_buf_tail - measurement_buf_head) % MEASUREMENT_BUF_CAPACITY;
}

uint64_t chipid;
String cid_str;
String sketch_md5;
States_t state = REACHING;
uint32_t last_pulse_ms = 0;
uint8_t lora_txpacket[LORA_PACKET_SIZE];
static RadioEvents_t RadioEvents;
int runner_sockfd = -1;


bool is_big_endian(void){
  union {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 1;
}
bool is_little_endian(void){
  union {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 4;
}

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

int _send_all(int sockfd, void *base_ptr, ssize_t num_bytes){
  ssize_t bytes_sent;
  char *ptr = (char *)base_ptr;
  int i = 0;

  while(i < num_bytes){
    bytes_sent = send(sockfd,ptr+i,num_bytes-i,0);
    if(bytes_sent <= 0){
      perror("send");
      return -1;
    }
    i += bytes_sent;
  }

  return 0;
}

int send_data_to_runner(){
  int ret = 0;
  int head = measurement_buf_head, tail = measurement_buf_tail;
  ssize_t num_bytes_p1, num_bytes_p2;
  uint32_t local_mstime = millis();

  if(head == tail){
    ret = -6;
    Serial.println("No data to send!");
    goto casd_cleanup;
  }

  if(WiFi.status() != WL_CONNECTED){
    ret = -1;
    Serial.println("WiFi not conn!");
    goto casd_cleanup;
  }

  if(head <= tail){
    // No wrap
    num_bytes_p1 = sizeof(Measurement)*(tail-head);
    num_bytes_p2 = 0;
  }
  else{
    // Wrap
    num_bytes_p1 = sizeof(Measurement)*(MEASUREMENT_BUF_CAPACITY-head);
    num_bytes_p2 = sizeof(Measurement)*tail;
  }
  Serial.printf("head: %d, tail: %d, nbp1: %d, nbp2: %d\n",head,tail,num_bytes_p1,num_bytes_p2);
  //TODO check ret vals
  _send_all(runner_sockfd,measurement_buf+head,num_bytes_p1);
  _send_all(runner_sockfd,measurement_buf,num_bytes_p2);
  measurement_buf_head = tail;
  Serial.printf("Sending finished, took %lu ms\n",millis()-local_mstime);

  casd_cleanup:
  return ret;
}

int connect_to_runner(){
  int ret = 0;
  struct sockaddr_in serv_addr;
  char runner_answer = '0';

  if(WiFi.status() != WL_CONNECTED){
    ret = -1;
    Serial.println("WiFi not conn!");
    goto casd_cleanup;
  }

  if(runner_sockfd > 0){
    ret = -7;
    Serial.println("Runner already connected!");
    goto casd_cleanup;
  }

  runner_sockfd = socket(AF_INET, SOCK_STREAM, 0);
  Serial.printf("runner_sockfd = %d\n",runner_sockfd);
  if(runner_sockfd < 0) {
    ret = -2;
    perror("socket");
    goto casd_cleanup;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(DEST_PORT);
  if(inet_pton(AF_INET,DEST_IP,&serv_addr.sin_addr) <= 0) {
    ret = -3;
    Serial.println("Invalid address/ Address not supported");
    close(runner_sockfd);
    runner_sockfd = -1;
    goto casd_cleanup;
  }
  if(connect(runner_sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    ret = -4;
    perror("connect");
    close(runner_sockfd);
    runner_sockfd = -1;
    goto casd_cleanup;
  }
  Serial.println("Connected to runner!");
  
  uint32_t header[3];
  header[0] = htonl(EXPERIMENT_PROTOCOL_MAGIC_NUM);
  header[1] = htonl((is_little_endian() << 31) | chipid >> 32);
  header[2] = htonl(chipid & 0xffffffff);
  
  _send_all(runner_sockfd,header,sizeof(header));
  Serial.printf("Header sent to runner (size: %d)\n",sizeof(header));

  for(int iter=0; iter<10 && runner_answer == '0'; iter++){
    if(recv(runner_sockfd,&runner_answer,1,MSG_DONTWAIT) >= 0){
      break;
    }
    delay(250);
  }
  if(runner_answer == 'A'){
    Serial.println("Runner ACK recvd!");
  }
  else{
    ret = -8;
    Serial.printf("Runner failed to ACK in time, recvd: %c\n",runner_answer);
    close(runner_sockfd);
    runner_sockfd = -1;
    goto casd_cleanup;
  }

  casd_cleanup:
  return ret;
}

uint16_t lora_pulse_i = 0;
void send_lora_pulse(){
  *(uint32_t *)(lora_txpacket+0) = htonl(LORA_PROTOCOL_MAGIC_NUMBER);
  *(uint32_t *)(lora_txpacket+4) = htonl(chipid >> 32);
  *(uint32_t *)(lora_txpacket+8) = htonl(chipid & 0xffffffff);
  *(uint16_t *)(lora_txpacket+12) = htons(LOCAL_TX_POWER);
  *(uint16_t *)(lora_txpacket+14) = htons(LOCAL_ANTENNA_GAIN);
  *(uint16_t *)(lora_txpacket+16) = htons(lora_pulse_i);
  Serial.printf("TX mode, sending i = 0x%02x\n",lora_pulse_i++);
  Radio.Send(lora_txpacket, LORA_PACKET_SIZE);
}

void OnTxDone(){
  Serial.println("TX done......");
}

void OnTxTimeout(){
  Serial.println("TX Timeout......");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  Serial.println("RX done......");
  Serial.printf("Rx size : %d, rssi : %d, snr : %d\n",size,rssi,snr);

  // Check if the packet is large enough, has the correct protocol num, and same major version number
  uint32_t recv_magic_num = ntohl(*(uint32_t *)payload);
  uint16_t i;
  if(size >= LORA_PACKET_SIZE && recv_magic_num == LORA_PROTOCOL_MAGIC_NUMBER && state == RECORDING){
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME,&ts);
    double time = (double)ts.tv_sec + (double)ts.tv_nsec/1000000000;
    Serial.printf("ts.tv_sec: %lld, .tv_nsec: %ld\n",ts.tv_sec,ts.tv_nsec);

    measurement_buf[measurement_buf_tail] = Measurement(time,rssi);
    if(get_num_measurements() >= MEASUREMENT_BUF_CAPACITY-1-MEASUREMENT_BUF_SLACK){
      // If circ buf is about to link up, we choose to lose the oldest few values rather than all
      Serial.println("WARNING: CIRC BUF OUT OF ROOM; BURNING ITEMS!!!");
      measurement_buf_head = (measurement_buf_head + 1) % MEASUREMENT_BUF_CAPACITY;
    }
    measurement_buf_tail = (measurement_buf_tail + 1) % MEASUREMENT_BUF_CAPACITY;
    Serial.printf("head: %d, tail: %d\n",measurement_buf_head,measurement_buf_tail);

    i = ntohs(*(uint16_t *)(payload+16));
    Serial.printf("Recv i = 0x%x\n",i);
  }
  else{
    Serial.printf("Bad state, size, or protocol magic number/version!\nSize: %d bytes",size);
    if(size >= 4){
      Serial.printf("; Recvd magic num: 0x%lx, expected: 0x%lx",recv_magic_num,(uint32_t)LORA_PROTOCOL_MAGIC_NUMBER);
    }
    Serial.println();
  }
}

void OnRxTimeout(){
  Serial.println("RX Timeout......");
}

void OnRxError(){
  Serial.println("RX Error......");
}

void lora_init(){
  // Set the callback functions for the radio
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
  // Initialize the radio and set the configuration
  Radio.Init(&RadioEvents);
  srand1(Radio.Random());
  Radio.SetTxConfig(MODEM_LORA, LOCAL_TX_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetChannel(RF_FREQUENCY);
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
  Radio.Sleep();
  SPI.end();
  pinMode(RADIO_DIO_1,ANALOG);
  pinMode(RADIO_NSS,ANALOG);
  pinMode(RADIO_RESET,ANALOG);
  pinMode(RADIO_BUSY,ANALOG);
  pinMode(LORA_CLK,ANALOG);
  pinMode(LORA_MISO,ANALOG);
  pinMode(LORA_MOSI,ANALOG);
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_TIME_SECS*1000*(uint64_t)1000);
  esp_deep_sleep_start();
}

TaskHandle_t checkUserkey1kHandle = NULL;
// Function to handle the PRG button in a different task
void checkUserkey(void *pvParameters){
  uint32_t keydowntime;
  bool long_pressed = false;
  
  while(1){
    if(digitalRead(USERKEY_PIN) == 0){
      keydowntime = millis();
      Serial.printf("key down at: %lu ms\n",keydowntime);
      delay(10);
      while(digitalRead(USERKEY_PIN) == 0){
        long_pressed = (millis()-keydowntime) > HOLD_THRESHOLD_MS;
        if(long_pressed){
          break;
        }
      }
      
      if(state == BLIND_TX){
        state = REACHING;
      }
      else{
        state = BLIND_TX;
        if(runner_sockfd > 0){
          close(runner_sockfd);
        }
      }

      // Wait for release
      while(digitalRead(USERKEY_PIN) == 0){
        delay(5);
      }
    }
    delay(50);
  }
}

void redraw_screen(){
  factory_display.clear();
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  String packet = cid_str;
  switch(state){
    case REACHING:
      packet += " REACH";
      break;
    case CONNECTED:
      packet += " CONN";
      break;
    case RECORDING:
      packet += " REC";
      factory_display.drawString(64,48,"N = "+String(get_num_measurements()));
      break;
    case STOPPED:
      packet += " STOP";
      break;
    case BLIND_TX:
      packet += " B. TX";
      factory_display.drawString(64,48,"i = "+String(lora_pulse_i));
      break;
  }
  factory_display.drawString(64, 16, packet);
  factory_display.display();
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  WiFi.disconnect(false,true);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  WiFi.setAutoReconnect(true);
  chipid = get_mac_reversed_byteorder();
  cid_str = String(string_format("%012llX",chipid).c_str()).substring(6);
  sketch_md5 = ESP.getSketchMD5();
  Serial.printf("ChipID = 0x%012llX\n",chipid);
  Serial.printf("Free heap at start: %lu\n",ESP.getFreeHeap());
  pinMode(USERKEY_PIN,INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  configTime(0,0,"pool.ntp.org");

  xTaskCreateUniversal(checkUserkey, "checkUserkey1Task", 8192, NULL, 1, &checkUserkey1kHandle, CONFIG_ARDUINO_RUNNING_CORE);
  VextON();
  delay(100);

  lora_init();
  factory_display.init();
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  factory_display.drawString(64, 16, sketch_md5.substring(0,11));
  factory_display.drawString(64, 32, sketch_md5.substring(11,22));
  factory_display.drawString(64, 48, sketch_md5.substring(22,32));
  factory_display.display();
  delay(1000);
  redraw_screen();
}

void loop() {
  char runner_message = '0';
  ssize_t recv_ret = 0;
  Mcu.timerhandler();
  Radio.IrqProcess();

  if(WiFi.status() != WL_CONNECTED){
    // If WiFi disconnects, attempt closing the connection
    // TODO improve robustness
    Serial.println("WiFi disconnected!");
    if(runner_sockfd > 0){
      close(runner_sockfd);
      runner_sockfd = -1;
    }
    delay(250);
  }

  if(time(NULL) < 1000000){
    Serial.println("Time not set by NTP yet!");
  }

  switch(state){
    case REACHING:
      Radio.Sleep();
      if(connect_to_runner() == 0){
        Serial.println("State changed to CONNECTED");
        state = CONNECTED;
      }
      break;
    case CONNECTED:
      // Ready radio, do nonblocking recv to check for start message
      Radio.Standby();
      recv_ret = recv(runner_sockfd,&runner_message,1,MSG_DONTWAIT);
      if(recv_ret == 1 && runner_message == '^'){
        // Start message recvd
        Serial.println("Runner start recvd!\nState changed to RECORDING");
        state = RECORDING;
        Radio.Rx(RX_TIMEOUT_MS);
      }
      else if(recv_ret == 0){
        // Connection closed, no data
        Serial.println("Connection dropped");
        state = STOPPED;
      }
      else if(recv_ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK){
        // Error occurred
        perror("recv");
        state = STOPPED;
      }
      break;
    case RECORDING:
      // Autosend branch
      if(get_num_measurements() >= DESIRED_AUTOSEND_NUMBER){
        Serial.println("Attempting autosend");
        send_data_to_runner();
      }
      // Check for stop message
      recv_ret = recv(runner_sockfd,&runner_message,1,MSG_DONTWAIT);
      if(recv_ret == 1 && runner_message == '$'){
        // Stop message recvd
        Serial.println("State changed to STOPPED");
        state = STOPPED;
      }
      else if(recv_ret == 0){
        // Connection closed, no data
        Serial.println("Connection dropped");
        state = STOPPED;
      }
      else if(recv_ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK){
        // Error occurred
        perror("recv");
        state = STOPPED;
      }
      break;
    case STOPPED:
      // Send final section of data, then close
      // We are sending null_measurement instead of pushing to circ buff first to avoid race condition
      char null_measurement[sizeof(Measurement)];
      memset(null_measurement,0,sizeof(null_measurement));
      send_data_to_runner();
      _send_all(runner_sockfd,null_measurement,sizeof(null_measurement));
      close(runner_sockfd);
      runner_sockfd = -1;
      state = REACHING;
      break;
    case BLIND_TX:
      int d = TX_PERIOD_MS-(millis()-last_pulse_ms);
      if(d>0){
        delay(d);
      }
      last_pulse_ms = millis();
      send_lora_pulse();
      break;
  }

  redraw_screen();
  delay(25);
}
