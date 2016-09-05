#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DanyBotLCD.h>
#include <SD.h>

DanyBotLCD lcd(0x27, 16, 2);
TinyGPS gps;
int led = 13;
const int chipSelect = 10;

long lat, lon, alt;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;
//int year;
//byte month, day, hour, minute, second, hundredths;
 
int DEG;
int MIN1;
int MIN2;

class GPSEntry {
  public: 
     long timestamp;
     long latitude;
     long longitude;
     long altitude;
};

const int SIZE = 8;

GPSEntry gps_entries[SIZE];
int fifo_index = -1;

void add(long timestamp, long lat, long lon, long alt) {
    if (fifo_index == SIZE - 1) {
      write_data();  
      fifo_index = 0; 
    }
    else
      fifo_index++;
    gps_entries[fifo_index].timestamp = timestamp;
    gps_entries[fifo_index].latitude = lat;
    gps_entries[fifo_index].longitude = lon;
    gps_entries[fifo_index].altitude = alt;
}

byte writeBuffer[SIZE*32];

int toByteBuffer(int i, long value) {
    int longIndex;
    long mask = 0xff;
    long shiftedValue;
    byte convertedValue;
    for (longIndex = 0; longIndex < 8; longIndex++) {
        shiftedValue = (value >> ((7-longIndex)*8) );
        convertedValue = shiftedValue & mask;
        writeBuffer[i++] = convertedValue;
    }
    return i;
}

void toBytes() {
   int i = 0;
   int entryIndex = 0;
   while (i < SIZE*32) {
       i = toByteBuffer(i, gps_entries[entryIndex].timestamp);
       i = toByteBuffer(i, gps_entries[entryIndex].latitude);
       i = toByteBuffer(i, gps_entries[entryIndex].longitude);
       i = toByteBuffer(i, gps_entries[entryIndex].altitude);
   }
   entryIndex++;
}

void write_data() {
  toBytes();
  //Serial.write(writeBuffer,SIZE*32);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.write(writeBuffer,SIZE*32);
  dataFile.close();
}

void LAT(){                       //Latitude state
  DEG=lat/1000000;
  MIN1=(lat/10000)%100;
  MIN2=lat%10000;
  lcd.setCursor(0,0);             // set the LCD cursor   position 
  lcd.print("LAT:");              
  lcd.print(DEG);
  lcd.write(0xDF);
  lcd.print(MIN1);
  lcd.print(".");
  lcd.print(MIN2);
  lcd.print("'   ");
}
void LON(){                        //Longitude state
  DEG=lon/1000000;
  MIN1=(lon/10000)%100;
  MIN2=lon%10000;
 
  lcd.setCursor(0,1);              // set the LCD cursor   position 
  lcd.print("LON:");              
  lcd.print(DEG);
  lcd.write(0xDF);
  lcd.print(MIN1);
  lcd.print(".");
  lcd.print(MIN2);
  lcd.print("'   ");
}

void setup(){
  Serial.begin(9600);
  toBytes();
  pinMode(led, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  lcd.init();                     //Initializing the LCD 
  lcd.backlight();
  lcd.begin(16, 2);               // start the library
  lcd.setCursor(0,0);             // set the LCD cursor   position 
  lcd.print("GPS test");          // print a simple message on the LCD 
  delay(2000);
}

void loop(){
  while (Serial.available()){
    digitalWrite(led, HIGH);
    int c = Serial.read();                   // Read the GPS data
    if (gps.encode(c))                        // Check the GPS data
    {
      // process new gps info here
    }
  }
  digitalWrite(led, LOW);
  gps.get_position(&lat, &lon, &fix_age);     // retrieves +/- lat/long in 100000ths of a degree
  alt = gps.altitude();
  gps.get_datetime(&date, &time, &fix_age);   // time in hhmmsscc, date in ddmmyy
  add(time, lat, lon, alt);
  //gps.crack_datetime(&year, &month, &day,    //Date/time cracking
  //&hour, &minute, &second, &hundredths, &fix_age);  
  LAT();
  LON();
}
