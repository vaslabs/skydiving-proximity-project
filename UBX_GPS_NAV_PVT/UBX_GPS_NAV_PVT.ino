#include <SD.h>

#include <SoftwareSerial.h>
// Connect the GPS RX/TX to arduino pins 4 and 3
SoftwareSerial serial = SoftwareSerial(2,3);
const int chipSelect = 10;

//-------------------Buffer-----------------------
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
       entryIndex++;    
   }
}

void write_data() {
  toBytes();
  //Serial.write(writeBuffer,SIZE*32);
  File dataFile = SD.open("datalog.sdc", FILE_WRITE);
  dataFile.write(writeBuffer,SIZE*32);
  dataFile.close();
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

void setup() 
{
  Serial.begin(9600);
  serial.begin(9600);
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    serial.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  Serial.println("GPS Device has been configured");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

const long YEAR_SHIFT = 10000000000000;
const long MONTH_SHIFT = 100000000000;
const long DAY_SHIFT = 1000000000;
const long HOUR_SHIFT = 10000000;
const long MINUTE_SHIFT = 100000;
const long SECOND_SHIFT = 1000;
void loop() {
  if ( processGPS() ) {
    Serial.print("#SV: ");      Serial.print(pvt.numSV);
    Serial.print(" fixType: "); Serial.print(pvt.fixType);
    Serial.print(" Date:");     Serial.print(pvt.year); Serial.print("/"); Serial.print(pvt.month); Serial.print("/"); Serial.print(pvt.day); Serial.print(" "); Serial.print(pvt.hour); Serial.print(":"); Serial.print(pvt.minute); Serial.print(":"); Serial.print(pvt.second);
    Serial.print(" lat/lon: "); Serial.print(pvt.lat/10000000.0f); Serial.print(","); Serial.print(pvt.lon/10000000.0f);
    Serial.print(" gSpeed: ");  Serial.print(pvt.gSpeed/1000.0f);
    Serial.print(" heading: "); Serial.print(pvt.heading/100000.0f);
    Serial.print(" hAcc: ");    Serial.print(pvt.hAcc/1000.0f);
    Serial.println();
    long milliseconds = pvt.nano/1000000;
    milliseconds = milliseconds < 0L ? 0 : milliseconds;
    long timestamp = pvt.year * YEAR_SHIFT + pvt.month*MONTH_SHIFT
                      + pvt.day*DAY_SHIFT + pvt.hour*HOUR_SHIFT
                      + pvt.minute*MINUTE_SHIFT + pvt.second*SECOND_SHIFT
                      + milliseconds;
    add(pvt.iTOW, pvt.lat, pvt.lon, pvt.hMSL);
  }
}

