#include <Arduino.h>


void preTransmission() {
  digitalWrite(4, HIGH);
}

void postTransmission() {
  digitalWrite(4, LOW);
}

void blink() {
  digitalWrite(18, HIGH);
  delay(10);
  digitalWrite(18, LOW);
}


void setup() {
  Serial.begin(9600);

  pinMode(4, OUTPUT);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  digitalWrite(4, HIGH);
  //postTransmission();

  blink();
  delay(500);
  blink();
  delay(500);
  blink();
  delay(2000);
}

void loop() {
  preTransmission();
  Serial.println("keepalive");
  Serial.flush();
  postTransmission();
  blink();


  if(Serial.available()) {
    int incoming = Serial.read();
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