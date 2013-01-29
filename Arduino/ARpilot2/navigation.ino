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

int event_course = 0;

/* Angle1: my course
   Angle2: target course
   result: course correction 
*/
float angle_diff( float angle1, float angle2)
{
  float result = angle2 - angle1;
  
  if (result < 0)
    result += 360;

  if (result < 0)
    result += 360;
    
  return result;
}

float angle_sum( float angle1, float angle2)
{
  return angle_diff(-1*angle1, angle2);
}

/*Relative results -180 < x < 180 degrees */
float course_diff( float angle1, float angle2)
{
  float result = angle2 - angle1;
  
  if (result < -180)
    result += 360;

  if (result > 180)
    result -= 360;
    
  return result;
}

/* Distance to the flight path, taking the error angle and
   covered flight distance into account.                   */
float course_delta_m( float angle, float distance)
{
  float result = distance * sin(angle*PI/180);
  return result;
}

float mycourse()
{
  int min_dist = 4; /*Minimal distance between course updates*/
  
  if (lastpos_lat != 0 ){
    if (gps.hdop() < 200) 
      min_dist = 2;

    if (gps.distance_between(flat, flon, lastpos_lat, lastpos_lon) > 2 ) {
      course_b = gps.course_to(lastpos_lat, lastpos_lon, flat, flon);

      event_course = 1;

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

void set_mycourse(float course)
{
  lastpos_lat = 0;
  event_course = 0;
  course_b = course;  
}

void load_wp(int index)
{
  if (index == 0) { /*Load waypoint, and startwaypoint */
    statwp_lat = homewp_lat;
    statwp_lon = homewp_lon;
    statwp_alt = homewp_alt;
  }
  else {
    statwp_lat = currwp_lat;
    statwp_lon = currwp_lon;
    statwp_alt = currwp_alt;
  }

  if (index == -1) { /* -1 means HOME */
    currwp_lat=homewp_lat;
    currwp_lon=homewp_lon;
    currwp_alt=homewp_alt;
  }
  else {  
    currwp_lat = waypoints[index][0]+waypoints[index][1]/60+waypoints[index][2]/3600;
    currwp_lon = waypoints[index][3]+waypoints[index][4]/60+waypoints[index][5]/3600;
    currwp_alt = waypoints[index][6];
    currwp_height = waypoints[index][7];
  }
  
  if (currwp_alt == 0)
    currwp_alt = homewp_alt;

  course_angle = gps.course_to(statwp_lat, statwp_lon, currwp_lat, currwp_lon);

  way_dist=gps.distance_between(flat, flon, currwp_lat, currwp_lon);
//  way_elev_delta=(currwp_alt+currwp_height-falt) / way_dist;

  pilot_command_fly(); /*Tell the pilot we will be flying soon (select pilot mode)*/

#ifdef DEBUG  
  Serial.print("wp  lon: ");
  Serial.println(currwp_lon);
  Serial.print("wp  lat: ");
  Serial.println(currwp_lat);
  Serial.print("wp  alt: ");
  Serial.println(currwp_alt);
#endif
}

int reached()
{
  return (distance_to_wp < 5);
}

int near()
{
  return (distance_to_wp < 20);
}

void navigation_init ()
{
  int i;
  
  if (gps.hdop() > 350){
    Serial.print("$ECHO,#hdop: ");
    Serial.print(gps.hdop());
    Serial.print("\n");
    return;  
  }

  homewp_lon=flon;
  homewp_lat=flat;
  homewp_alt=falt;

  no_waypoint=0;
  load_wp(no_waypoint++);

  total_distance += gps.distance_between(homewp_lat, homewp_lon, currwp_lat, currwp_lon);
  
  /* Abort navigation if 1st WP is more than 500m away */
  if(total_distance > 5000000) {
    navigation = 5;
    Serial.print("$ECHO,#1ST WP too far.\n");
    return;
  }

  /* Sum up the distance between waypoints for log. */
  for (i=0 ; i<num_waypoints-1; i++) {
    total_distance += gps.distance_between( waypoints[i][0]+waypoints[i][1]/60+waypoints[i][2]/3600,
                                            waypoints[i][3]+waypoints[i][4]/60+waypoints[i][5]/3600,
                                            waypoints[i+1][0]+waypoints[i+1][1]/60+waypoints[i+1][2]/3600,
                                            waypoints[i+1][3]+waypoints[i+1][4]/60+waypoints[i+1][5]/3600);
  }

  total_distance += gps.distance_between( waypoints[i][0]+waypoints[i][1]/60+waypoints[i][2]/3600,
                                          waypoints[i][3]+waypoints[i][4]/60+waypoints[i][5]/3600,
                                          homewp_lat,homewp_lon );

  Serial.print("$ECHO,#Flight plan distance: ");
  Serial.print(total_distance);
  Serial.print("\n");

  /* Abort navigation if distance is more than 2000m */
  if(total_distance > 20000000) {
    navigation = 5;
    Serial.print("$ECHO,#Distance too great.\n");
    return;
  }
/* END summing up*/    

  set_mycourse(gps.course_to(flat, flon, currwp_lat, currwp_lon));

  pilot_liftoff();

  navigation++;
}

void navigation_update()
{
  course_to_wp = gps.course_to(flat, flon, currwp_lat, currwp_lon);
  course_from_wp = gps.course_to(statwp_lat, statwp_lon, flat, flon);

  distance_to_wp = gps.distance_between(flat, flon, currwp_lat, currwp_lon);
  distance_from_wp = gps.distance_between(statwp_lat, statwp_lon, flat, flon);

  course_err = course_diff( course_from_wp, course_angle);
  head_err = course_diff( course_to_wp , course_angle);

  course_delta = course_delta_m( course_err, distance_from_wp);
  
  lognavupdate();
  
  if (reached())
    navigation++;
}

void navigation_nextwp()
{
  if (no_waypoint < num_waypoints ) {
    load_wp(no_waypoint++);

    navigation--;
  }
  else
    navigation++;
}

void navigation_home()
{
  load_wp(-1);
  navigation++;
}

