#include <SPI.h>

#include "nRF24L01_lib.h"

#define LED D0

uint8_t value[PACKET_SIZE];
// uint8_t volatile *received_data;
uint8_t count;
uint8_t volatile packet_received;

// For debugging:
uint16_t invalidPackets = 0;
uint16_t noReceived = 0;
uint16_t totalPackets = 0;

void setup() {
  SPI.begin();
  Serial.begin(250000);

  //Setup PIN modes
  pinMode(CSN, OUTPUT);
  pinMode(CE, OUTPUT);
  digitalWrite(CSN, HIGH);
  digitalWrite(CE, LOW);

  // Setup LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);

  // Setup Interrupt
  pinMode(IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IRQ), ISR, FALLING);
  
  Serial.println();
  nrfInit(1); // Setup for receiver

  count = 0;
}

void loop() {
  uint8_t write_buffer[PACKET_SIZE];
  int i;
  count++;

  // Package ID = 1
  write_buffer[0] = count;

  // Receiver ID = 3
  write_buffer[1] = 0;
  write_buffer[2] = 0;
  write_buffer[3] = 0;
  write_buffer[4] = 3;

  // Data = 65793
  write_buffer[5] = 1;
  write_buffer[6] = 1;
  write_buffer[7] = 1;
  
  totalPackets++;
  transmit_payloadAck(write_buffer);
  reset();

  Serial.print("Invalid packets: ");
  Serial.print(invalidPackets);
  Serial.print(" no ack: ");
  Serial.print(noReceived);
  Serial.print(" out of ");
  Serial.println(totalPackets);

  //delay(1000);
}

void ISR(){
  packet_received = 1;
  reset();
}

