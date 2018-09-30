//Balloon cutdown code
//Built on the Adafruit GPS parsing library
// www.adafruit.com

/***********************************
This is our GPS library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

//  print raw gps data
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//pin 12 sends cutdown signal to balloon
int trigger_out = 12;
int cutdown_trys = 0;

void setup() {
  pinMode(trigger_out, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Balloon cutdown!");

  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS spits out RMCGGA (see http://aprs.gids.nl/nmea/)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(500);
  Serial.print("\nSTARTING LOGGING....");
  if (GPS.LOCUS_StartLogger())
    Serial.println(" STARTED!");
  else
    Serial.println(" no response :(");
  delay(1000);  
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;      
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop() {
  // if a sentence is received, check checksum, parse
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  //if failed to parse a sentence, wait for another
  }

  // if millis() or timer wraps around, reset it
  if (timer > millis())  timer = millis();

  // every 30 seconds  check GPS and decide if cutdown should be made
  if (millis() - timer > 30000) { 
    timer = millis(); // reset the timer
    
	 //Uncomment to print out GPS data
    //Serial.print("\nTime: ");
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      float alt = GPS.altitude;
		
		//currently set up to cutdown at 20,000 m
		//alternatively use lat, lon, etc
		//5 cutdown attempts
      if((alt > 20000) && (cutdown_trys < 5)){
        digitalWrite(trigger_out, HIGH);
        delay(15000);
        Serial.println("Cutdown");
        Serial.println("Attempt");
        Serial.println(cutdown_trys);
        cutdown_trys = cutdown_trys + 1;
        digitalWrite(trigger_out, LOW);
      }
    }
  }
}
