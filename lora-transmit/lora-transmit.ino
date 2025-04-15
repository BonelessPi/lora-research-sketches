/*
Proof of concept for transmitting back to a TCP server
*/

#include <Arduino.h>
#include <string>
#include <stdexcept>
#include <sys/socket.h>
#include <WiFi.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

#define USERKEY_PIN 0
#define HOLD_THRESHOLD_MS 3000
#define DEEPSLEEP_TIME_SECS 600

#define TRANSMIT_PROTOCOL_MAGIC_NUM 0x61462cdf
#include "../secrets.h"
#define DESIRED_AUTOSEND_NUMBER 100
#define MEASUREMENT_BUF_CAPACITY 500
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

#define LORA_PROTOCOL_MAGIC_NUMBER 0x4bb7cfa3
#define LORA_PACKET_SIZE 32
#define RX_TIMEOUT_MS 2000
#define TX_PERIOD_MS 200
#define RSSI_BUFFER_SIZE 5
#define LOCAL_ANTENNA_GAIN 5


SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

typedef enum{
  IDLE,
  STATE_RX,
  STATE_TX
} States_t;


struct __attribute__((packed)) Measurement{
  uint32_t ms;
  int16_t rssi;

  Measurement(uint32_t a, int16_t b):ms(a),rssi(b){}
  Measurement():ms(0),rssi(0){}
};
Measurement measurement_buf[MEASUREMENT_BUF_CAPACITY];
int measurement_buf_head = 0, measurement_buf_tail = 0;
int get_num_measurements(){
  return (MEASUREMENT_BUF_CAPACITY + measurement_buf_tail - measurement_buf_head) % MEASUREMENT_BUF_CAPACITY;
}

uint64_t chipid;
String cid_str;
String sketch_md5;
States_t state = IDLE;
uint32_t last_pulse_ms = 0;
int16_t remote_tx_power = LOCAL_TX_POWER;
int16_t remote_antenna_gain = LOCAL_ANTENNA_GAIN;
uint8_t lora_txpacket[LORA_PACKET_SIZE];
static RadioEvents_t RadioEvents;


bool is_big_endian(void){
  union {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 1;
}

bool is_little_endian(void){
  return !is_big_endian();
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

float calc_dist(int16_t rssi){
  return pow(10.0,(-rssi + remote_tx_power + LOCAL_ANTENNA_GAIN + remote_antenna_gain)/20.0) * (299792458.0/4.0/PI/RF_FREQUENCY);
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

int transmit_data_home(){
  int ret = 0, sockfd = -1;
  struct sockaddr_in serv_addr;
  uint32_t local_mstime;
  int head = measurement_buf_head, tail = measurement_buf_tail;
  ssize_t num_bytes_p1, num_bytes_p2;

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

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  Serial.printf("sockfd = %d\n",sockfd);
  if(sockfd < 0) {
    Serial.println("Socket creation error");
    perror("socket");
    ret = -2;
    goto casd_cleanup;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(DEST_PORT);
  if(inet_pton(AF_INET,DEST_IP,&serv_addr.sin_addr) <= 0) {
    Serial.println("Invalid address/ Address not supported");
    ret = -3;
    goto casd_cleanup;
  }
  if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    Serial.println("Connection Failed");
    perror("connect");
    ret = -4;
    goto casd_cleanup;
  }
  local_mstime = millis();
  Serial.println("Connected");
  
  uint32_t header[5];
  header[0] = htonl(TRANSMIT_PROTOCOL_MAGIC_NUM);
  header[1] = htonl((is_little_endian() << 31) | chipid >> 32);
  header[2] = htonl(chipid & 0xffffffff);
  header[3] = htonl(local_mstime);
  header[4] = htonl((MEASUREMENT_BUF_CAPACITY + tail - head) % MEASUREMENT_BUF_CAPACITY);
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
  Serial.printf("N: %ld, head: %d, tail: %d, nbp1: %d, nbp2: %d\n",ntohl(header[4]),head,tail,num_bytes_p1,num_bytes_p2);
  _send_all(sockfd,header,20);
  _send_all(sockfd,measurement_buf+head,num_bytes_p1);
  _send_all(sockfd,measurement_buf,num_bytes_p2);
  measurement_buf_head = tail;
  Serial.printf("Sending finished, took %lu ms\n",millis()-local_mstime);
  recv(sockfd,NULL,0,0);

  casd_cleanup:
  if(sockfd >= 0){
    close(sockfd);
  }
  return ret;
}

void send_lora_pulse(){
  static uint16_t i = 0;
  *(uint32_t *)(lora_txpacket+0) = htonl(LORA_PROTOCOL_MAGIC_NUMBER);
  *(uint32_t *)(lora_txpacket+4) = htonl(chipid >> 32);
  *(uint32_t *)(lora_txpacket+8) = htonl(chipid & 0xffffffff);
  *(uint16_t *)(lora_txpacket+12) = htons(LOCAL_TX_POWER);
  *(uint16_t *)(lora_txpacket+14) = htons(LOCAL_ANTENNA_GAIN);
  *(uint16_t *)(lora_txpacket+16) = htons(i);
  Serial.printf("TX mode, sending i = 0x%02x\n",i++);
  Radio.Send(lora_txpacket, LORA_PACKET_SIZE);
}

void OnTxDone(){
  Serial.println("TX done......");
  decide_action();
}

void OnTxTimeout(){
  Serial.println("TX Timeout......");
  decide_action();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  uint32_t mstime = millis();
  Serial.println("RX done......");
  Serial.printf("Rx size : %d, rssi : %d, snr : %d\n",size,rssi,snr);

  // Check if the packet is large enough, has the correct protocol num, and same major version number
  uint32_t recv_magic_num = ntohl(*(uint32_t *)payload);
  uint16_t i;
  if(size >= LORA_PACKET_SIZE && recv_magic_num == LORA_PROTOCOL_MAGIC_NUMBER && state == STATE_RX){
    measurement_buf[measurement_buf_tail] = Measurement(mstime,rssi);
    if(get_num_measurements() >= MEASUREMENT_BUF_CAPACITY-1-MEASUREMENT_BUF_SLACK){
      // If circ buf is about to link up, we choose to lose the oldest few values rather than all
      Serial.println("WARNING: CIRC BUF OUT OF ROOM; BURNING ITEMS!!!");
      measurement_buf_head = (measurement_buf_head + 1) % MEASUREMENT_BUF_CAPACITY;
    }
    measurement_buf_tail = (measurement_buf_tail + 1) % MEASUREMENT_BUF_CAPACITY;
    Serial.printf("head: %d, tail: %d\n",measurement_buf_head,measurement_buf_tail);

    remote_tx_power = (int16_t)ntohs(*(uint16_t *)(payload+12));
    remote_antenna_gain = (int16_t)ntohs(*(uint16_t *)(payload+14));
    i = ntohs(*(uint16_t *)(payload+16));
    Serial.printf("Recv remote tx power: %d, remote antenna gain: %d, i = 0x%x\n",remote_tx_power,remote_antenna_gain,i);

    redraw_screen();
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

void transition_state(bool long_press){
  switch(state){
    case IDLE:
      if(long_press){
        state = STATE_TX;
      }
      else{
        state = STATE_RX;
      }
      break;
    case STATE_RX:
      state = IDLE;
      transmit_data_home();
      break;
    case STATE_TX:
      state = IDLE;
      break;
    default:
      Serial.printf("Unknown state: %d\n",state);
      state = IDLE;
  }
  decide_action();
  redraw_screen();
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
      transition_state(long_pressed);
      while(digitalRead(USERKEY_PIN) == 0){
        delay(5);
      }
    }
    if(get_num_measurements() >= DESIRED_AUTOSEND_NUMBER){
      Serial.println("Attempting autosend");
      transmit_data_home();
      // This delay is necessary to avoid blocking out the Radio process apparently
      delay(25);
    }
  }
}

void redraw_screen(){
  Serial.println("Redrawing screen");
  factory_display.clear();
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
  String packet = cid_str;
  switch(state){
    case IDLE:
      packet += " IDLE";
      break;
    case STATE_RX:
      packet += " RX";
      factory_display.drawString(64,48,"N = "+String(get_num_measurements()));
      break;
    case STATE_TX:
      packet += " TX";
      break;
    default:
      packet += " UNKNOWN";
      break;
  }
  factory_display.drawString(64, 16, packet);
  factory_display.display();
}

void decide_action(){
  Serial.printf("deciding action, state=%d\n",state);
  long d = 0;
  switch(state){
    case IDLE:
      Radio.Sleep();
      break;
    case STATE_RX:
      Radio.Rx(RX_TIMEOUT_MS);
      break;
    case STATE_TX:
      d = TX_PERIOD_MS-(millis()-last_pulse_ms);
      if(d>0){
        delay(d);
      }
      last_pulse_ms = millis();
      send_lora_pulse();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  chipid = get_mac_reversed_byteorder();
  cid_str = String(string_format("%012llX",chipid).c_str()).substring(6);
  sketch_md5 = ESP.getSketchMD5();
  Serial.printf("ChipID = 0x%012llX\n",chipid);
  Serial.printf("Free heap at start: %lu\n",ESP.getFreeHeap());
  pinMode(USERKEY_PIN,INPUT);
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
	delay(100);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  WiFi.disconnect(false,true);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  WiFi.setAutoReconnect(true);
  delay(1000);
  redraw_screen();
}

void loop() {
  Mcu.timerhandler();
  Radio.IrqProcess();
  delay(50);
}
