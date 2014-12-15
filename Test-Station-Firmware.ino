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

/////////////
//Altimeter//
/////////////
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

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

  // Set Up Ethernet:
  setupEthernet();

}

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


