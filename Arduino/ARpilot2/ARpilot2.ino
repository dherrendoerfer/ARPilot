/*
 * This file is part of arpilot.
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

#define VERSION "v2.0_20130108_1"

SoftwareSerial GPS = SoftwareSerial(4,3);
TinyGPS        gps;
int  fix_led = 13;

/* Schoenaich */
/*                        Lat,             Lon,       Ground Height, Fly Height */
float waypoints[][8] ={{  48,39,46.53 ,    9,3,6.45,           0,            3 },
                       {  48,39,43.65 ,    9,3,5.71,           0,            5 },
                       {  48,39,44.32 ,    9,2,58.90,          0,            5 },
                       {  48,39,43.65 ,    9,3,5.71,           0,            5 },
                       {  48,39,43.01 ,    9,3,9.19,           0,            3 }};
int num_waypoints = 5;
char wp_location[]="Schoenaich 5WP";

/* Hausen am Bach */
/*                        Lat,             Lon,       Ground Height, Fly Height
float waypoints[][8] ={{  49,17,28.62 ,    10,6,56.81,         0,            2 },
                       {  49,17,25.79 ,    10,6,58.66,         0,            10 },
                       {  49,17,25.80 ,    10,7,2.80,          0,            10 },
                       {  49,17,28.36 ,    10,7,2.51,          0,            10 },
                       {  49,17,27.34 ,    10,6,59.90,         0,            3 }};

int num_waypoints = 5;
char wp_location[]="Hausen am Bach 5WP";
*/


/*FSM Global Variables*/

float flat, flon;
float falt; 
float fkmh;

float currwp_lat, currwp_lon, currwp_alt, currwp_height;
float statwp_lat, statwp_lon, statwp_alt;
float homewp_lat, homewp_lon, homewp_alt;
int no_waypoint;
int total_distance = 0; /*will be calculated*/
float course_angle;
float course_delta;

float course_head;
float drone_angle;

float course_err;
float head_err;

float course_to_wp;
float course_from_wp;
float distance_to_wp;
float distance_from_wp;
float way_dist;

float lastpos_lon, lastpos_lat, lastpos_alt;
float course_b;

float camera_angle = 0;  /*Drone offset angel (BROKEN)*/
float drone_angle_err = 0;

void setup()
{
  GPS.begin(9600);
  Serial.begin(115200);
 
  pilot_setup();
}

int gps_counter =0;
int aquired = 0;
int navigation = 0;
unsigned long fix_age = 0;

void loop()
{
  if (GPS.available()) {
     int byte = GPS.read();
//     Serial.write(byte);
     
     if (gps.encode(byte)) {
//       Serial.println("Fix !");
       digitalWrite(fix_led, HIGH);
       
       if ( gps_counter++ % 2 == 0)
         aquired = 1;
     }   
   }

  // retrieves +/- lat/long in 100000ths of a degree
  gps.f_get_position(&flat, &flon, &fix_age);

  // If Fix, then read the position info
  if (aquired == 1) {
    falt = gps.f_altitude();     // +/- altitude in meters
//    fc   = gps.f_course();       // course in degrees
//    fc   = mycourse();           // course in degrees
    mycourse();                  //update course_b
    fkmh = gps.f_speed_kmph();   // speed in km/hr
    
    drone_setpos();

    if (navigation == 0) {
      Serial.print("$ECHO,#NAV:START\n"); 
      navigation_init();
    }
    
    if (navigation == 1 || navigation == 4) {
      navigation_update();
      pilot_fly();
    }
    
    if (navigation == 2) {
      Serial.print("$ECHO,#NAV:NEXT\n"); 
      navigation_nextwp();
    }
    
    if (navigation == 3) {
      Serial.print("$ECHO,#NAV:HOME\n"); 
      navigation_home();
    }
    
    if (navigation == 5) {
      Serial.print("$ECHO,#NAV:END\n"); 
      pilot_end();
    }
    
    aquired=0;
  }
  
  if (fix_age > 5000 && aquired != 2) {
     aquired = 2;
     digitalWrite(fix_led, LOW);
     //Serial.print("$ECHO,#NO GPS\n");
  }
}
