/*
 * Adapted from HelTec Automation(TM) WIFI_LoRa_32 factory test code
 * 
 * - Basic OLED function test;
 * 
 * - Basic serial port test(in baud rate 115200);value
 * 
 * - LED blink test;
 * 
 * - WIFI connect and scan test;
 * 
 * - LoRa Ping-Pong test (DIO0 -- GPIO26 interrup check the new incoming messages);
 * 
 * - Timer test and some other Arduino basic functions.
*/

#include "Arduino.h"
#include "WiFi.h"
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
#define BUFFER_SIZE 50 // Define the payload size here

#define USERKEY 0
#define HOLD_THRESHOLD_MS 3000
#define DEEPSLEEP_TIME_SECS 600

SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

uint8_t txpacket[BUFFER_SIZE];
uint8_t rxpacket[BUFFER_SIZE];


static RadioEvents_t RadioEvents;

typedef enum{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

States_t state = LOWPOWER;
int16_t maxTxRssi = -255;
int16_t maxRxRssi = -255;

String packet;

uint64_t chipid;
uint8_t rx_cnt = 0;
bool loratimeout = false;
int wifiScanCnt = 0;
int wifiScanMaxRssi = -255;


void OnTxDone(){
  Serial.println("TX done......");
  //state = LOWPOWER;
}

void OnTxTimeout(){
  Radio.Sleep();
  Serial.println("TX Timeout......");
  //state = LOWPOWER;
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
  //state = LOWPOWER;
  /*if(lora_mode == 0){
    state = STATE_TX;
    if(size == 10){
      uint64_t rxchipid = *((uint64_t *)payload);
      if(rxchipid == chipid){
        int16_t txrssi = *(int16_t *)(payload+8);
        Serial.printf("rx own mes,rssi :%d\n",txrssi);
        if(maxTxRssi < txrssi){
          maxTxRssi = txrssi;
        }
        if(maxRxRssi < rssi){
          maxRxRssi = rssi;
        }
        rx_cnt++;
        if(rx_cnt >= 3){
          showStatus();
          state = LOWPOWER;
        }
      }
    }
  }
  else{
    if(size == 8){
      state = STATE_TX;
      memcpy(txpacket,payload,8);
      memcpy(txpacket+8,(uint8_t *)&rssi,2);
    }
    else{
      Radio.Rx(0);
    }
  }*/
}

void OnRxTimeout(){
  Radio.Sleep();
  Serial.println("RX Timeout......");
  //state = LOWPOWER;
}

void OnRxError(){
  Radio.Sleep();
  Serial.println("RX Error......");
  //state = LOWPOWER;
}

void lora_init(){
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxError = OnRxError;
  RadioEvents.RxTimeout = OnRxTimeout;
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

void logo(){
	factory_display.clear();
	factory_display.drawXbm(0,0,logo_width,logo_height,(const unsigned char *)logo_bits);
	factory_display.display();
}

/*void WIFISetUp(){
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	delay(100);
	factory_display.drawString(0, 10, "WIFI Setup done");
	factory_display.display();
	delay(500);
}

void WIFIScan(unsigned int value){
	unsigned int i;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

	for(i = 0; i < value; ++i){
		factory_display.drawString(0, 20, "Scan start...");
		factory_display.display();

		int n = WiFi.scanNetworks();
    wifiScanCnt = n;
		factory_display.drawString(0, 30, "Scan done");
		factory_display.display();
		delay(500);
		factory_display.clear();

		if (n == 0){
			factory_display.clear();
			factory_display.drawString(0, 0, "no network found");
			factory_display.display();
		}
    else{
			factory_display.drawString(0, 0, (String)n);
			factory_display.drawString(14, 0, "networks found:");
			factory_display.display();
			delay(250);

			for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
        if(wifiScanMaxRssi < WiFi.RSSI(i)){
          wifiScanMaxRssi = WiFi.RSSI(i);
        }
				factory_display.drawString(0, (i+1)*9,(String)(i + 1));
				factory_display.drawString(6, (i+1)*9, ":");
				factory_display.drawString(12,(i+1)*9, (String)(WiFi.SSID(i)));
				factory_display.drawString(90,(i+1)*9, " (");
				factory_display.drawString(98,(i+1)*9, (String)(WiFi.RSSI(i)));
				factory_display.drawString(114,(i+1)*9, ")");
				factory_display.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
        delay(10);
			}
		}

		factory_display.display();
		delay(3000);
		factory_display.clear();
	}
}*/

void VextON(){
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(){
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void showStatus(){
  //TODO: test
  Serial.printf("WiFi   Count: %d   Max RSSI: %d\r\n",wifiScanCnt,wifiScanMaxRssi);
  Serial.printf("LoRa   TX RSSI: %d   RX RSSI: %d\r\n",maxTxRssi,maxRxRssi);

  factory_display.clear();
  factory_display.setFont(ArialMT_Plain_10);
  factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  packet = "WIFI   XX";
  factory_display.drawString(0, 0, packet);
  packet = "WIFI Scan Cnt:"+String(wifiScanCnt)+" Rssi:"+String(wifiScanMaxRssi);
  factory_display.drawString(0, 15, packet);
  packet = "LoRa Tx Rssi : "+String(maxTxRssi);
  factory_display.drawString(0, 30, packet);
  packet = "LoRa Rx Rssi : "+String(maxRxRssi);
  factory_display.drawString(0, 45, packet);

  uint32_t t = millis()/1000;
  factory_display.setTextAlignment(TEXT_ALIGN_RIGHT);
  packet = "time: "+String(t);
  factory_display.drawString(0, 60, packet);
  
  factory_display.display();
  //TODO: make LED react to messages coming in
  digitalWrite(LED, HIGH);
}

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
        intodeepsleep(); 
      }
      else{
        switch(state){
          case LOWPOWER:
            state = STATE_TX;
            break;
          case STATE_TX:
            state = STATE_RX;
            break;
          case STATE_RX:
            state = LOWPOWER;
            break;
        }
        //TODO MAKE SEND PACKET
        Serial.printf("changed state to %d\n",state);
        factory_display.clear();
        factory_display.setFont(ArialMT_Plain_16);
        factory_display.setTextAlignment(TEXT_ALIGN_CENTER);
        packet = "STATE ";
        switch(state){
          case LOWPOWER:
            packet += "LOWPOWER";
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
	factory_display.clear();
  lora_init();
	//WIFISetUp();
	//WiFi.disconnect();
	delay(100);

	//WIFIScan(1);
  
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
    case LOWPOWER:
      Serial.println("LOWPOWER mode");
      break;
    default:
      break;
  }
  delay(150);
  Mcu.timerhandler();
  Radio.IrqProcess();
}
