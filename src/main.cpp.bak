#include <Arduino.h>

#define RS485_BAUD 9600
#define PIN_RS485_TX 27
#define PIN_RS485_RX 26
#define PIN_RS485_REDE 4

//HardwareSerial RS485Serial(1);

void preTransmission() {
  digitalWrite(PIN_RS485_REDE, HIGH);
}

void postTransmission() {
  digitalWrite(PIN_RS485_REDE, LOW);
}

void blink() {
  digitalWrite(18, HIGH);
  delay(10);
  digitalWrite(18, LOW);
}


void setup() {
  Serial.begin(9600);

  // set RE/DE-pin
  pinMode(PIN_RS485_REDE, OUTPUT);
  postTransmission();

  Serial2.begin(RS485_BAUD);

  //RS485Serial.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // set blink-pin
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);

  // initial status-blink
  blink();
  delay(500);
  blink();
  delay(500);
  blink();
  delay(2000);
}

void loop() {
  Serial.println("serial-keepalive");
  Serial.flush();

  preTransmission();
  Serial2.println("rs485-keepalive");
  Serial2.flush();
  postTransmission();
  blink();


  if(Serial2.available()) {
    int incoming = Serial2.read();
    blink();
    preTransmission();
    Serial.print("Data received: ");
    Serial.println(incoming);
    Serial.flush();
    postTransmission();
  }


//  preTransmission();
//  digitalWrite(18, HIGH);
//  delay(1000);
//  postTransmission();
//  digitalWrite(18, LOW);
//  delay(1000);

  delay(1000); // Wait for 1 second before the next read
}