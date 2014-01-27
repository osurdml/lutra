#include "Platypus.h"
#include <adk.h>

// Include necessary Arduino headers to inform the IDE to link these libraries
#include <Servo.h>

// TODO: remove me
#include "Board.h"

/** ADK USB Host configuration */
// Accessory descriptor. 
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName, versionNumber, url, serialNumber);

// Android receive buffer
const size_t RECEIVE_BUFFER_SIZE = 512;
unsigned char input_buffer[RECEIVE_BUFFER_SIZE+1];

// TODO: move this to platypus header in wrapper object
platypus::LED rgb_led;
platypus::VaporPro motor(0);

bool state = 1;
void serialEvent1() {
  rgb_led.B(state);
  state = !state;

  while(Serial1.available()) {
    Serial.write(Serial1.read());
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO: probably not needed for Due
  
  // Make the ADK buffer a null terminated string  
  input_buffer[RECEIVE_BUFFER_SIZE] = 0;
  
  // Turn on the RS232 xmit transciever for sensor port 2
  
  // Disable RSxxx receiver
  pinMode(board::SENSOR[1].RX_DISABLE, OUTPUT);
  digitalWrite(board::SENSOR[1].RX_DISABLE, HIGH);

  // Disable RSxxx transmitter
  pinMode(board::SENSOR[1].TX_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[1].TX_ENABLE, LOW);

  // Disable RS485 termination resistor
  pinMode(board::SENSOR[1].RS485_TE, OUTPUT);
  digitalWrite(board::SENSOR[1].RS485_TE, LOW);

  // Select RS232 (deselect RS485)
  pinMode(board::SENSOR[1].RS485_232, OUTPUT);
  digitalWrite(board::SENSOR[1].RS485_232, LOW);

  // Disable half-duplex
  pinMode(board::HALF_DUPLEX01, OUTPUT);
  digitalWrite(board::HALF_DUPLEX01, LOW);

  // Disable loopback test
  pinMode(board::LOOPBACK, OUTPUT);
  digitalWrite(board::LOOPBACK, LOW);

  // Set RX_NEG pin to output low to act as pulldown for TX_POS output.
  // (This is a bug in the R2 board)
  pinMode(board::SENSOR[1].GPIO[board::RX_NEG], INPUT);
  digitalWrite(board::SENSOR[1].GPIO[board::RX_NEG], LOW);

  // Set TX_NEG pin as input to not interfere with RX_POS input.
  // (This is a bug in the R2 board)
  pinMode(board::SENSOR[1].GPIO[board::TX_NEG], INPUT);
  digitalWrite(board::SENSOR[1].GPIO[board::TX_NEG], LOW);
  
  // Enable sensor serial port
  Serial1.begin(1200);
  
  // Manually turn off TX_POS pin after serial setup.
  // (This is a bug in the R2 board)
  pinMode(board::SENSOR[1].GPIO[board::TX_POS], OUTPUT);
  digitalWrite(board::SENSOR[1].GPIO[board::TX_POS], LOW);
}

void loop() {
  digitalWrite(board::SENSOR[1].TX_ENABLE, LOW);
  rgb_led.R(1);
  rgb_led.G(0);
  
  delay(1000);
  
  digitalWrite(board::SENSOR[1].TX_ENABLE, HIGH);
  rgb_led.R(0);
  rgb_led.G(0);
  
  Serial.println("ES-2:");
  delay(1000);
  
  /*
  while(true) {
    Serial.print(sensorSerial2.read());
  }
  */
}
