#include <TinyGPS.h>
 
TinyGPS gps;
int led = 13;
 
long lat, lon, alt;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;
//int year;
//byte month, day, hour, minute, second, hundredths;

class GPSEntry {
  public: 
     long timestamp;
     long latitude;
     long longitude;
     long altitude;
};

const int SIZE = 64;

GPSEntry gps_entries[SIZE];
int fifo_index = -1;


int DEG;
int MIN1;
int MIN2;

void write_data() {
     
}

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
 
void setup()
{
  Serial.begin(9600);            //Set the GPS baud rate.
  
}
 
void loop()
{
  gps.get_position(&lat, &lon, &fix_age);     // retrieves +/- lat/long in 100000ths of a degree
  alt = gps.altitude();
  gps.get_datetime(&date, &time, &fix_age);   // time in hhmmsscc, date in ddmmyy
  add(time, lat, lon, alt);
}
