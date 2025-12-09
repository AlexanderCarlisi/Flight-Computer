/// Working Transmitter Example
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#define HALT_ON_INIT_ERR  1
#define RADIO_IS_CLOSE    1
#define RF_CHNL           110
#define RF_TX             {'P', 'A', 'T', '0', '1', '\0'}
#define RF_RX             {'P', 'A', 'T', '0', '2', '\0'}
#define RF_CE             9
#define RF_CSN            8
#define PAYLOAD_SIZE      128

const byte RADIO_TX_BYTES[6] = RF_TX;
const byte RADIO_RX_BYTES[6] = RF_RX;
RF24 rf24_radio(RF_CE, RF_CSN);
byte payload[PAYLOAD_SIZE];

void halt() {
  while (1) delay(10);
}

void radio_init(RF24& radio) {
  bool radio_initialized = radio.begin();
  if (!radio_initialized) {
    Serial.println("Radio Initialization Failed");
    if (HALT_ON_INIT_ERR) halt();

  } else {
    if (RADIO_IS_CLOSE)
      radio.setPALevel(RF24_PA_LOW); // MAX is default
    radio.setChannel(RF_CHNL);
    radio.openWritingPipe(RADIO_TX_BYTES); // set writes on Pipe 0, also puts in writting mode
    radio.openReadingPipe(1, RADIO_RX_BYTES); // reads on Pipe 1
    radio.setPayloadSize(PAYLOAD_SIZE);
    // radio.setDataRate(RF24_250KBPS);
    // radio.startListening() puts it in listening mode / RX Mode
  }
}

void transmit(RF24& radio) {
  radio.openWritingPipe(RADIO_TX_BYTES);
  bool report = radio.write(&payload, PAYLOAD_SIZE);

  if (report) {
    Serial.println("Transmission Successful");

  } else {
    Serial.println("Transmission failed or timed out");
    if (HALT_ON_INIT_ERR) halt();
  }
}

void setup() {
  do {
    Serial.begin(115200); // 9600 - 115200
  } while (!Serial);
  Serial.println("\n>>> Serial Connected <<<\n");
  Serial.println(">>> Setup Begun <<<");

  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    payload[i] = 0xFF;
  }

  Serial.println(">>> Radio Setup <<<");
  radio_init(rf24_radio);
  Serial.println(">>> Radio Initialized <<<");
}

void loop() {
  transmit(rf24_radio);
  delay(1000);
}
