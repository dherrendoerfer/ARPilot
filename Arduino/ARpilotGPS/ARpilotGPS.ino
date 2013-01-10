/*
 * This file is part of arpilotGPS.
 *
 * Copyright (C) 2012  D.Herrendoerfer
 *
 *   arpilot is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   arpilot is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with arpilot.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/*
 * Disclaimer:
 *
 * This Software is ALPHA ! It is provided for reference only. Use it at
 * your own risk. Use of this software may loose you your drone, or put
 * you in all sorts of touble. Make sure to obey local rules, laws, and regulations
 * considering air traffic where you fly. I'm telling you to do so now !
 * 
 * Have fun. 
 * DH.
 */
 
 /*
  * Setup: Ports 0 and 1 connect to the drones serial port.
  *        Use a level shifter for arduino RX, use a resistor
  *        voltage divider for TX. Only TX is needed at this time.
  *
  *        Ports 3 and 4 connect to the GPS, make sure it is in NMEA
  *        Mode at 9600 baud.
  *
  */

#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define VERSION "v1.0_20121223_1"

SoftwareSerial GPS = SoftwareSerial(4,3);
TinyGPS        gps;
int  fix_led = 13;
unsigned int time = 0;
unsigned int lastmsg = 0;
float flat, flon;
float falt;
unsigned long fix_age = 0;

int aquired = 0;

float lastpos_lon, lastpos_lat, lastpos_alt;
float course_b;



float mycourse()
{
  int min_dist = 4; /*Minimal distance between course updates*/
  
  if (lastpos_lat != 0 ){
    if (gps.hdop() < 200) 
      min_dist = 2;

    if (gps.distance_between(flat, flon, lastpos_lat, lastpos_lon) > 2 ) {
      course_b = gps.course_to(lastpos_lat, lastpos_lon, flat, flon);

      lastpos_lat = flat;
      lastpos_lon = flon;
      lastpos_alt = falt;
    }
  }
  else {
    lastpos_lat = flat;
    lastpos_lon = flon;
    lastpos_alt = falt;
  }  
  return course_b;  
}

void pilot_setup()
{
  Serial.print("\n\n$QUIT\n");
  delay(1500);
  Serial.print("cat - | /data/video/usb/arpilot2.arm > /data/video/usb/plog.txt\n");
  delay(20000);
  Serial.print("cat - | /data/video/usb/arpilot2.arm > /data/video/usb/plog.txt\n");

  delay(2000);

  Serial.print("$ECHO,#ARpilotGPS sketch version: ");
  Serial.print(VERSION);
  Serial.print("\n");
}

void setup()
{
  GPS.begin(9600);
  Serial.begin(115200);
  
  pilot_setup();
}

void loop()
{
  time = millis();
  
  if (GPS.available()) {
     int byte = GPS.read();
     
     if (gps.encode(byte)) {
       digitalWrite(fix_led, HIGH);
       aquired = 1;
     }
   }

  // retrieves +/- lat/long in 100000ths of a degree
  gps.f_get_position(&flat, &flon, &fix_age);

  // If Fix, then read the position info
  if (aquired == 1 && fix_age < 2000) {
    falt = gps.f_altitude();     // +/- altitude in meters
    mycourse();                  //update course_b
//    fkmh = gps.f_speed_kmph();   // speed in km/hr
    
    Serial.print("$PLON,");
    Serial.print(flon,14);
    Serial.print("\n");
    
    Serial.print("$PLAT,");
    Serial.print(flat,14);
    Serial.print("\n");
  
    Serial.print("$PALT,");
    Serial.print(falt,0);
    Serial.print("\n");
    
    Serial.print("$PCOU,");
    Serial.print(course_b,10);
    Serial.print("\n");
    
    Serial.print("$PHDO,");
    Serial.print(gps.hdop());
    Serial.print("\n");
    
    lastmsg = time;
    
    aquired=0;
    
    return;
  }
  else {
    aquired = 0;
  }
  
  if ( time - lastmsg > 999) {
    int hdop = gps.hdop();
    
    if (fix_age > 5000)
      hdop = 9999; 
    
    Serial.print("$PHDO,");
    Serial.print(hdop);
    Serial.print(
    "\n");
    
    lastmsg = time;
  }
  
  if (fix_age > 5000 && aquired != 2) {
     aquired = 2;
     digitalWrite(fix_led, LOW);
  }
}
