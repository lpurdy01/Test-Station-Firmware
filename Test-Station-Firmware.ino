//See Phant 1
//See Altimeter 1
//See Magnentometer 1

#include <SPI.h> // Required to use Ethernet
#include <Ethernet.h> // The Ethernet library includes the client
#include <avr/pgmspace.h>
#include <Wire.h>
#include "Adafruit_MPL3115A2.h"
#include "Adafruit_GPS.h"
#include <SoftwareSerial.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"


/////////////
//Altimeter//
/////////////
//See Altimeter 2
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();


///////
//GPS//
///////

//See GPS 2
// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):

//SoftwareSerial mySerial(3, 2);
//Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
#define GPSSerial true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//////////////////
// Magnetometer //
//////////////////
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


///////////////////////
// Ethernet Settings //
///////////////////////
// Enter a MAC address for your controller below.
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
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
const String fieldNames[NUM_FIELDS] PROGMEM = {"accuracy","altitude_meters","elevation_gps_meters","heading","humidity","latitude","longitude","magnetometer_x_ut","pressure_kpa","satellites","station","tempature_c","timestamp","value1","value2","value3","value4","value5","value6","value7","value8","y_ut","z_ut"};
String fieldData[NUM_FIELDS];

//////////////////////
// Input Pins, Misc //
//////////////////////

String name = "Lab_Test";

///////////////////////////////////////////////////////////////////////////////////////
//////setup                                                  //////////////////////////

void setup()
{
  Serial.begin(115200);
  Serial.println("Adafruit_MPL3115A2 test!");
  fieldData[10] = name;
  setup_GPS();
  setup_magnetometer();

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

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////Loop                                                   /////////////////////////////////////////

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
  ///////
  //GPS//
  ///////
  while(! GPS.fix)
  {
    GPSloop();
    delay(2000);
  }
  delay(100);
  GPSPost();
  //////////////////
  // Magnetometer //
  //////////////////
  
  loop_magnetometer();
  
  ///////////////
  //Phant Stuff//
  ///////////////
  Serial.println("Posting!");
  postData(); // the postData() function does all the work, 
  // check it out below.

  delay(20000);

}
void setup_magnetometer()
{
  Serial.println("HMC5883 Mag Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("No HMC5883 detected");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}
void loop_magnetometer()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
  delay(100);
}
void post_magnetometer()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  fieldData[7] = String(event.magnetic.x);
  fieldData[21] = String(event.magnetic.y);
  fieldData[22] = String(event.magnetic.z);
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
      client.print(pgm_read_word(&fieldNames[i]));
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
    Serial.println(("Connection failed"));
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
  Serial.println("Setuping Ethernet");
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // give the Ethernet shield a second to initialize:
  delay(1000);
}
void setup_GPS()
{
  Serial.println("Setup GPS");

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
  Serial1.println(PMTK_Q_RELEASE);
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
      Serial.print("Location degrees: ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Spd knots: "); Serial.println(GPS.speed);
      Serial.print("Ang: "); Serial.println(GPS.angle);
      Serial.print("Alt: "); Serial.println(GPS.altitude);
      Serial.print("Sats: "); Serial.println((int)GPS.satellites);
    }
  }
}
void GPSPost()
{
  fieldData[0] = String(GPS.fixquality);
  fieldData[2] = String(GPS.altitude);
  fieldData[5] = String(GPS.latitude);
  fieldData[6] = String(GPS.longitude);
  fieldData[9] = String(GPS.satellites);
}

