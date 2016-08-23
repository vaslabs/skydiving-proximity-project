#include <TinyGPS.h>
#include <DanyBotLCD.h>

DanyBotLCD lcd(0x27, 16, 2);
 
TinyGPS gps;
int led = 13;
 
long lat, lon;
unsigned long fix_age, time, date, speed, course;
unsigned long chars;
unsigned short sentences, failed_checksum;
//int year;
//byte month, day, hour, minute, second, hundredths;
 
int DEG;
int MIN1;
int MIN2;
  
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
 
 
 
void setup()
{
  Serial.begin(9600);            //Set the GPS baud rate.
  
  pinMode(led, OUTPUT);  
  lcd.init();  //Initializing the LCD 
  lcd.backlight();
  lcd.begin(16, 2);               // start the library
  lcd.setCursor(0,0);             // set the LCD cursor   position 
  lcd.print("GPS test");          // print a simple message on the LCD 
  delay(2000);
}
 
void loop()
{
  while (Serial.available())
  {
    digitalWrite(led, HIGH);
    int c = Serial.read();                   // Read the GPS data
    if (gps.encode(c))                        // Check the GPS data
    {
      // process new gps info here
    }
 
  }
  digitalWrite(led, LOW);
  gps.get_position(&lat, &lon, &fix_age);     // retrieves +/- lat/long in 100000ths of a degree
  gps.get_datetime(&date, &time, &fix_age);   // time in hhmmsscc, date in ddmmyy
 
  //gps.crack_datetime(&year, &month, &day,    //Date/time cracking
  //&hour, &minute, &second, &hundredths, &fix_age);  
 
LAT();
LON();
 
}
