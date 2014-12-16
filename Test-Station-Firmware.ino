/*****************************************************************
 * Phant_Ethernet.ino
 * Post data to SparkFun's data stream server system (phant) using
 * an Arduino and an Ethernet Shield.
 * Jim Lindblom @ SparkFun Electronics
 * Original Creation Date: July 3, 2014
 * 
 * This sketch uses an Arduino Uno to POST sensor readings to 
 * SparkFun's data logging streams (http://data.sparkfun.com). 
 * 
 * Before uploading this sketch, there are a number of global vars
 * that need adjusting:
 * 1. Ethernet Stuff: Fill in your desired MAC and a static IP, even
 * if you're planning on having DCHP fill your IP in for you.
 * The static IP is only used as a fallback, if DHCP doesn't work.
 * 2. Phant Stuff: Fill in your data stream's public, private, and 
 * data keys before uploading!
 * 
 * Hardware Hookup:
 * These components are connected to the Arduino's I/O pins:
 * D3 - Active-low momentary button (pulled high internally)
 * A0 - Photoresistor (which is combined with a 10k resistor
 * to form a voltage divider output to the Arduino).
 * D5 - SPST switch to select either 5V or 0V to this pin.
 * A CC3000 Shield sitting comfortable on top of your Arduino.
 * 
 * Development environment specifics:
 * IDE: Arduino 1.0.5
 * Hardware Platform: RedBoard & PoEthernet Shield
 * 
 * This code is beerware; if you see me (or any other SparkFun 
 * employee) at the local, and you've found our code helpful, please 
 * buy us a round!
 * 
 * Much of this code is largely based on David Mellis' WebClient
 * example in the Ethernet library.
 * 
 * Distributed as-is; no warranty is given.
 *****************************************************************/
/**************************************************************************/
/*!
 @file     Adafruit_MPL3115A2.cpp
 @author   K.Townsend (Adafruit Industries)
 @license  BSD (see license.txt)
 
 Example for the MPL3115A2 barometric pressure sensor
 
 This is a library for the Adafruit MPL3115A2 breakout
 ----> https://www.adafruit.com/products/1893
 
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 
 @section  HISTORY
 
 v1.0 - First release
 */
/**************************************************************************/

#include <SPI.h> // Required to use Ethernet
#include <Ethernet.h> // The Ethernet library includes the client
#include <Progmem.h> // Allows us to sacrifice flash for DRAM
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


/////////////
//Altimeter//
/////////////
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code just echos whatever is coming from the GPS unit to the
// serial monitor, handy for debugging!
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada



// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
#define GPSSerial true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


///////////////////////
// Ethernet Settings //
///////////////////////
// Enter a MAC address for your controller below.
byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
IPAddress server(54,86,132,254);  // numeric IP for data.sparkfun.com
//char server[] = "data.sparkfun.com";    // name address for data.sparkFun (using DNS)
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192,168,1,33);

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

/////////////////
// Phant Stuff //
/////////////////
const String publicKey = "KJQE5LjM03T0gVbnD4dM";
const String privateKey = "vznMWJm1KeHy1pbevEDd";
const byte NUM_FIELDS = 23;
const String fieldNames[NUM_FIELDS] = {"accuracy","altitude_meters","elevation_gps_meters","heading","humidity","latitude","longitude","magnetometer_x_ut","pressure_kpa","satellites","station","tempature_c","timestamp","value1","value2","value3","value4","value5","value6","value7","value8","y_ut","z_ut"};
String fieldData[NUM_FIELDS];

//////////////////////
// Input Pins, Misc //
//////////////////////

String name = "Lab_Test";

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit_MPL3115A2 test!");
  fieldData[10] = name;
  setup_GPS();

  // Set Up Ethernet:
  setupEthernet();

}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
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

void loop()
{
  ///////////////////
  //Altimiter Stuff//
  ///////////////////
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  altimeter_get_debug();
  altimeter_post_to_table();
  delay(100);
  GPSloop();
  delay(250);
  ///////////////
  //Phant Stuff//
  ///////////////
  Serial.println("Posting!");
  postData(); // the postData() function does all the work, 
  // check it out below.

  delay(20000);

}

void altimeter_post_to_table()
{
  float pascals = baro.getPressure();
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();
  
  fieldData[1] = String(altm);
  fieldData[8] = String(pascals/1000);
  fieldData[11] = String(tempC);
}

void altimeter_get_debug()
{
  float pascals = baro.getPressure();
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();
  Serial.print(pascals/1000); Serial.println(" kPa");
  Serial.print(altm); Serial.println(" meters");
  Serial.print(tempC); Serial.println("*C");
  delay(250);
}

void postData()
{
  
  // Make a TCP connection to remote host
  if (client.connect(server, 80))
  {
    // Post the data! Request should look a little something like:
    // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&name=Jim HTTP/1.1\n
    // Host: data.sparkfun.com\n
    // Connection: close\n
    // \n
    client.print("GET /input/");
    client.print(publicKey);
    client.print("?private_key=");
    client.print(privateKey);
    for (int i=0; i<NUM_FIELDS; i++)
    {
      client.print("&");
      client.print(fieldNames[i]);
      client.print("=");
      client.print(fieldData[i]);
    }
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    client.println();
  }
  else
  {
    Serial.println(F("Connection failed"));
  } 

  // Check for a response from the server, and route it
  // out the serial port.
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      Serial.print(c);
    }      
  }
  Serial.println();
  client.stop();
}

void setupEthernet()
{
  Serial.println("Setting up Ethernet...");
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // give the Ethernet shield a second to initialize:
  delay(1000);
}
void setup_GPS()
{
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}
void GPSloop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000 && GPSSerial) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
String GPSDatestamp()
{
  char datestamp[25] = "0000000000000000000000000";
  datestamp = GPS.year+"-"+GPS.month+"-"+GPS.day
}
void GPSPost()
{
  
}

