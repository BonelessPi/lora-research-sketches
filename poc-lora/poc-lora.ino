/*
 * Adapted from HelTec Automation(TM) WIFI_LoRa_32 factory test code
 *
*/

#include <Arduino.h>
#include "images.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
/********************************* lora  *********************************************/
#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 10        // dBm

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

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 10 // Define the payload size here

#define USERKEY 0
#define HOLD_THRESHOLD_MS 3000
#define DEEPSLEEP_TIME_SECS 600

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

// Buffers for the sending and receiving of data. rxpacket is currently unused in this PoC
uint8_t txpacket[BUFFER_SIZE];
uint8_t rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

typedef enum{
  IDLE,
  STATE_RX,
  STATE_TX
}States_t;

States_t state = IDLE;
String packet;
uint64_t chipid;


void OnTxDone(){
  Serial.println("TX done......");
}

void OnTxTimeout(){
  Radio.Sleep();
  Serial.println("TX Timeout......");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
  Radio.Sleep();
  Serial.println("RX done......");
  Serial.printf("Rx size : %d, rssi : %d, snr : %d\n",size,rssi,snr);
  for(int i = 0; i < size; i++)
  {
    Serial.printf("%02X ",payload[i]);
  }
  Serial.println();
}

void OnRxTimeout(){
  Radio.Sleep();
  Serial.println("RX Timeout......");
}

void OnRxError(){
  Radio.Sleep();
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
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetChannel(RF_FREQUENCY);
}

// Draw the logo defined in images.h
void logo(){
	factory_display.clear();
	factory_display.drawXbm(0,0,logo_width,logo_height,(const unsigned char *)logo_bits);
	factory_display.display();
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
  pinMode(USERKEY,INPUT);
  while(1){
    if(digitalRead(USERKEY) == 0){
      keydowntime = millis();
      Serial.printf("key down at: %u ms\n",keydowntime);
      delay(10);
      while(digitalRead(USERKEY) == 0){
        if((millis()-keydowntime) > HOLD_THRESHOLD_MS){
          break;
        }
      }
      if((millis()-keydowntime) > HOLD_THRESHOLD_MS){
        // Hold button puts device into sleep
        intodeepsleep(); 
      }
      else{
        // Pressed button changes the mode
        switch(state){
          case IDLE:
            state = STATE_TX;
            break;
          case STATE_TX:
            state = STATE_RX;
            break;
          case STATE_RX:
            state = IDLE;
            break;
        }
        
        Serial.printf("changed state to %d\n",state);
        factory_display.clear();
        factory_display.setFont(ArialMT_Plain_16);
        factory_display.setTextAlignment(TEXT_ALIGN_CENTER);
        packet = "STATE ";
        switch(state){
          case IDLE:
            packet += "IDLE";
            break;
          case STATE_TX:
            packet += "TX";
            break;
          case STATE_RX:
            packet += "RX";
            break;
          default:
            packet += "UNKNOWN";
            break;
        }
        packet = "STATE "+String(state);
        factory_display.drawString(64, 24, packet);
        factory_display.display();
        delay(1000);
        factory_display.setFont(ArialMT_Plain_10);
      }
    }
  }
}

void setup(){
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  chipid = ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.
  xTaskCreateUniversal(checkUserkey, "checkUserkey1Task", 2048, NULL, 1, &checkUserkey1kHandle, CONFIG_ARDUINO_RUNNING_CORE);
  VextON();
  delay(100);

  factory_display.init();
  factory_display.fillRect(0, 0, factory_display.getWidth(), factory_display.getHeight());
  factory_display.display();
  delay(1000);
	logo();
  delay(1000);
  lora_init();
	delay(100);
  
  factory_display.setFont(ArialMT_Plain_16);
  factory_display.setTextAlignment(TEXT_ALIGN_CENTER);

  factory_display.clear();
  factory_display.display();
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);
}

uint16_t i = 0;
void loop(){
  switch(state){
    case STATE_TX:
      *(uint64_t *)txpacket = chipid;
      *(uint16_t *)(txpacket+8) = i;
      Serial.printf("TX mode, sending i = %02x\n",i++);
      Radio.Send(txpacket, 10);
      break;
    case STATE_RX:
      Serial.println("RX mode");
      Radio.Rx(0);
      break;
    case IDLE:
      Serial.println("IDLE mode");
      break;
    default:
      break;
  }
  delay(150);
  Mcu.timerhandler();
  Radio.IrqProcess();
}
