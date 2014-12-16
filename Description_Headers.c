
// Phant 1/////////////////////////////////////////////////////////////////////////////////////////
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
//Altimeter 1/////////////////////////////////////////////////////////////////////////////////////////
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
//Magnentometer 1/////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746
 
  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/
 
 
//Altimeter 2/////////////////////////////////////////////////////////////////////////////////////////
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire
 
 
//GPS 2///////////////////////////////////////////////////////////////////////////////////////////////

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


