#include "nRF24L01_lib.h"

void nrfInit(uint8_t RX_TX) {

  // RX_TX: 0 = Transmitter, 1 = Receiver
  uint8_t value[PACKET_SIZE];
  int i;

  digitalWrite(CSN, HIGH);
  digitalWrite(CE, LOW);
  delay(100); //delay 100 miliseconds for startup

  value[0] = 0x01;
  accessRegister(W, EN_RXADDR, value, 1); // enable data pipe 0
  value[0] = 0x03;
  accessRegister(W, SETUP_AW, value, 1); // Setup address width to 5 bytes
  value[0] = 0x01;
  accessRegister(W, RF_CH, value, 1); // set frequency to 2,401 GHz
  value[0] = 0x07;
  accessRegister(W, RF_SETUP, value, 1); // 00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode (""11" = -0dB ; "00"= -18dB)
  value[0] = 0x10;
  accessRegister(W, RX_ADDR_P0, value, 1);

  // Set Receiver address on Pipe 0 to 5 x 0x07
  for (i = 0; i < 5; i++) {
    value[i] = 0x07;
  }
  accessRegister(W, RX_ADDR_P0, value, 5);

  // Set Transmitter address on Pipe 0 to 5 x 0x07
  for (i = 0; i < 5; i++) {
    value[i] = 0x07;
  }
  accessRegister(W, TX_ADDR, value, 5);

  value[0] = PACKET_SIZE;
  accessRegister(W, RX_PW_P0, value, 1); // Send 5 bytes per package

  if (RX_TX == 1) {
    value[0] = 0x33;
    Serial.println("nRF initialised as Receiver");
  }
  else {
    value[0] = 0x32;
    Serial.println("nRF initialised as Transmitter");
  }
  accessRegister(W, CONFIG, value, 1); //bit "1":1=power up,  bit "0":0=transmitter,bit "0":1=Reciever, bit "4":1=>mask_Max_RT

  delay(100);
}

uint8_t GetReg(uint8_t reg){
  delayMicroseconds(10);
  digitalWrite(CSN, 0);
  delayMicroseconds(10);
  SPI.transfer(READ_REGISTER + reg);
  delayMicroseconds(10);
  reg = SPI.transfer(NOP);
  delayMicroseconds(10);
  digitalWrite(CSN, 1);
  return reg;
}

uint8_t *accessRegister(uint8_t readWrite, uint8_t reg, uint8_t *value, uint8_t arraySize) {

  int i;
  static uint8_t ret[PACKET_SIZE];

  if (readWrite == W) {
    reg = WRITE_REGISTER + reg;
  }

  delayMicroseconds(10); //_delay_us(10);
  digitalWrite(CSN, 0);
  delayMicroseconds(10);
  SPI.transfer(reg);
  delayMicroseconds(10);

  for (i = 0; i < arraySize; i++) {
    if (readWrite == R && reg != W_TX_PAYLOAD) {
      ret[i] = SPI.transfer(0);
      delayMicroseconds(10);
    }
    else {
      SPI.transfer(value[i]);
      delayMicroseconds(10);
    }
  }

  digitalWrite(CSN, 1);
  return ret;
}

void transmit_payload(uint8_t *w_buffer) {

  accessRegister(R, FLUSH_TX, w_buffer, 0);
  accessRegister(R, W_TX_PAYLOAD, w_buffer, PACKET_SIZE);
  Serial.print(" ID: ");
  Serial.print(w_buffer[0]);
  delay(19);
  digitalWrite(CE, 1);
  delayMicroseconds(30);
  digitalWrite(CE, 0);
}

void transmit_payloadAck(uint8_t *w_buffer) {

  uint8_t i, j;
  uint8_t *data;
  uint8_t receivedData[PACKET_SIZE]; // container to hold the received data

  for (i = 0; i < NUMBER_RETRIES; i++) {

    // Send the package
    transmitterStart();
//    Serial.print("Transmitting: "); // For debugging
//    Serial.print(i); // For debugging
    transmit_payload(w_buffer);
    transmitterStop();
       
    // listen for acknowledgement
    receive_payload(w_buffer);

    if (packet_received){//((GetReg(STATUS) & (1<<6)) != 0)){   //if (packet_received) {
      packet_received = 0;
      Serial.print("Package received: ");
      data = accessRegister(R, R_RX_PAYLOAD, data, PACKET_SIZE);

      // Make a local copy of the received data
      for (j = 0; j < PACKET_SIZE; j++) {
        receivedData[j] = data[j];
      }

      if (receivedData[0] == w_buffer[0]) {
        // Serial.print(receivedData[0]); // For debugging
        // Serial.print(" = Package acknowledged"); // For debugging
        break;
      }
      else {
        // Serial.print(receivedData[0]); // For debugging
        // Serial.println(" = Invalid answer received..."); // For debugging
                
        if(i==2){
          invalidPackets++; // For debugging
        }
        
        packet_received = 0;
        reset();
      }
    }
    else {      
      //Serial.println("did not receive answer"); // For debugging
      if(i==2){
        noReceived++; //for debugging
      }
      packet_received = 0;
      reset();      
    } 
  }
}

void receive_payload(uint8_t *w_buffer) {
  unsigned long timer = 0;
  unsigned long time_begin = 0;

  accessRegister(R, FLUSH_RX, w_buffer, 0);
  digitalWrite(CE, 1);

  time_begin = millis();
  while ((!packet_received) && (timer < 1000)) {  // (!packet_received) ||
    timer = millis() - time_begin;
  }
  if(timer >= 500){
    Serial.println(" Timer overflow!");
  }
  
  digitalWrite(CE, 0);
  delay(50);
}

void reset(void) {
  delayMicroseconds(50);
  digitalWrite(CSN, 0);
  delayMicroseconds(50);
  SPI.transfer(WRITE_REGISTER + STATUS);
  delayMicroseconds(10);
  SPI.transfer(0x70);
  delayMicroseconds(50);
  digitalWrite(CSN, 1);
}

void transmitterStart(void) {

  uint8_t value[PACKET_SIZE];
  uint8_t *data;

  delay(19);
  digitalWrite(CE, LOW); // goto Standby mode
  delayMicroseconds(20);
  data = accessRegister(R, CONFIG, data, 1);
  value[0] = data[0] & ~(1 << 0); // set PRIMRX=0 - Transmitter mode
  accessRegister(W, CONFIG, value, 1);
  digitalWrite(CE, HIGH); // goto Transmitter mode
  delayMicroseconds(250);
}

void transmitterStop(void) {
  uint8_t value[PACKET_SIZE];
  uint8_t *data;
  
  delay(19);
  digitalWrite(CE, LOW); // goto Standby mode
  delayMicroseconds(20);
  data = accessRegister(R, CONFIG, data, 1);
  value[0] = data[0] | (1 << 0); // set PRIMRX=0 - Transmitter mode
  accessRegister(W, CONFIG, value, 1);
  digitalWrite(CE, HIGH); // goto Transmitter mode
  delayMicroseconds(250);
  accessRegister(R, FLUSH_RX, data, 0);
}

