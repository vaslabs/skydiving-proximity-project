#include <SPI.h>

#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <stdint.h>
#include <MS561101BA.h>
#define MOVAVG_SIZE 32

MS561101BA baro = MS561101BA();

float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;

const float sea_press = 1013.25;
float press, temperature;


// Connect the GPS RX/TX to arduino pins 2 and 3
SoftwareSerial serial = SoftwareSerial(2,3);
const int chipSelect = 10;

const int buttonPin = 6;     // the number of the pushbutton pin
const int ledPin =  7;       // the number of the LED pin
int buttonState = 0;
boolean loggingStarted = false;
boolean SDCardCheck;

boolean dataFile_created = false;
String dataFile_name;

//-------------------Buffer-----------------------
class GPSEntry {
  public: 
     uint64_t timestamp;
     int32_t latitude;
     int32_t longitude;
     float altitude;
};

const int SIZE = 2;

GPSEntry gps_entries[SIZE];

int fifo_index = -1;

void add(unsigned long long timestamp, uint32_t lat, uint32_t lon, float alt) {
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

byte writeBuffer[SIZE*20];

int toByteBuffer(int i, uint64_t value, int bitSize) {
    int bitIndex;
    int64_t mask = 0xff;
    int64_t shiftedValue;
    byte convertedValue;
    for (bitIndex = 0; bitIndex < bitSize; bitIndex++) {
        shiftedValue = (value >> ((bitSize - 1 -bitIndex)*bitSize) );
        convertedValue = shiftedValue & mask;
        writeBuffer[i++] = convertedValue;
    }
    return i;
}

void toBytes() {
   int i = 0;
   int entryIndex = 0;
   while (i < SIZE*20) {
       i = toByteBuffer(i, gps_entries[entryIndex].timestamp, 8);
       i = toByteBuffer(i, gps_entries[entryIndex].latitude, 4);
       i = toByteBuffer(i, gps_entries[entryIndex].longitude, 4);
       i = toByteBuffer(i, gps_entries[entryIndex].altitude, 4);
       entryIndex++;    
   }
}

void write_data() {
  toBytes();
  File dataFile = SD.open(dataFile_name, FILE_WRITE);
  if(dataFile){
    dataFile.write(writeBuffer,SIZE*32);
    dataFile.close();
  }else{
    blink();
    SetupSDCard();
  }
}

//-----------------End of buffer------------------


//Configure GPS module
const char UBLOX_INIT[] PROGMEM = {
  // Reset to manufacturer defaults
  0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x07,0x1F,0x9E,
  
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24,  //GGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B,  //GLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32,  //GSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39,  //GSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40,  //RMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47,  //VTG off
  
  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC,   //NAV-PVT off
  
  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1,  //NAV-PVT on
  
  // Update Rate
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,            //Update 10Hz
  
  // Save the changes
  0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0x31,0xBF
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  unsigned long reserved3;     // Reserved
};

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

boolean SetupSDCard(){
  if (!SD.begin(chipSelect)){
    Serial.println("Card failed, or not present");
    SDCardCheck = false;
    return false;
  }else{
    Serial.println("card initialized.");
    SDCardCheck = true;
    return true;
  }
}

void blink(){
  digitalWrite(ledPin, HIGH);delay(300); digitalWrite(ledPin, LOW); delay(300); digitalWrite(ledPin, HIGH);delay(300); digitalWrite(ledPin, LOW); delay(300);
}
void setup(){
  Wire.begin();
  delay(1000);

  // Suppose that the CSB pin is connected to GND.
  // You'll have to check this on your breakout schematics
  baro.init(MS561101BA_ADDR_CSB_LOW); 
  delay(100);
  
  // populate movavg_buff before starting loop
  for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
  }
  
  Serial.begin(9600);
  serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    serial.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  Serial.println("GPS Device has been configured");
  SetupSDCard();
}

const uint64_t YEAR_SHIFT = 10000000000000L;
const uint64_t MONTH_SHIFT = 100000000000L;
const uint64_t DAY_SHIFT = 1000000000L;
const uint64_t HOUR_SHIFT = 10000000L;
const uint64_t MINUTE_SHIFT = 100000L;
const uint64_t SECOND_SHIFT = 1000L;

void loop() {
  if(SDCardCheck == false){
    SetupSDCard();
  }
  if (processGPS()) {
    temperature = baro.getTemperature(MS561101BA_OSR_4096);
    press = baro.getPressure(MS561101BA_OSR_4096);
    pushAvg(press);
    press = getAvg(movavg_buff, MOVAVG_SIZE);
    int64_t milliseconds = pvt.nano/1000000;
    milliseconds = milliseconds < 0L ? 0 : milliseconds;
    uint64_t timestamp = (pvt.year) * YEAR_SHIFT + (pvt.month)*MONTH_SHIFT
                      + (pvt.day)*DAY_SHIFT + (pvt.hour)*HOUR_SHIFT
                      + (pvt.minute)*MINUTE_SHIFT + (pvt.second)*SECOND_SHIFT
                      + milliseconds;
    add(timestamp, pvt.lat, pvt.lon,getAltitude(press, temperature));
    if(!dataFile_created){
      dataFile_name = random(99999999);
      dataFile_name = dataFile_name+".sdc";
      while(SD.exists(dataFile_name)){
        dataFile_name = "";
        dataFile_name = random(99999999);
        dataFile_name = dataFile_name+".sdc";
      }
      dataFile_created = true;
    }
  }
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
  float sum = 0.0;
  for(int i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}
