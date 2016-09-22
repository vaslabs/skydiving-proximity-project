//Arduino UNO is the slave
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

byte x;
const int chipSelect = 10;
const int SIZE = 64;

void setup() {
  Wire.begin(9);
  Wire.onReceive(receiveEvent); // data slave received
  Serial.begin(9600);
  Serial.println("I2C Slave ready!");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void receiveEvent(int countToRead) {
  String response = "";
  while (Wire.available()) {
      char b = Wire.read();
      response += b;
  }
  byte buf[sizeof(response)];
  File dataFile = SD.open("datalog.sdc", FILE_WRITE);
  if (dataFile) {
    response.getBytes(buf, sizeof(buf));
    dataFile.write(buf,SIZE*32);
    dataFile.close();
    Serial.println(response);
  }
}

void loop() {
  delay(50);
}
