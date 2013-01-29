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
 
int pilot_count = 0;
int pilot_wait = 0;
int pilot_manoever = 0; //Just fly
int pilot_manoever_step = 0;

float pilot_temp_lat = 0;
float pilot_temp_lon = 0;
float pilot_temp_alt = 0;

int pilot_temp_incrementer = 0;

float drone_heading = 0;

void pilot_turn(float direction)
{
  drone_turn(direction);
  
  // Trust this direction
  drone_heading += direction;
  set_mycourse(drone_heading);
}

void pilot_point(float direction)
{
  if (direction < 0)
    direction += 360;
  
  drone_head(direction);
  
  // Trust this direction
  drone_heading=direction;
  set_mycourse(direction);
}

void pilot_fly()
{
//  int head=adj_head();
  int alti = falt-currwp_alt;
  
  int evt_course = 0;
  
  int pitch = -500;
  int yaw   = 0;
  int roll  = 0;
  int gaz   = 0;

  pilot_count++; /*increment counter*/

  if (pilot_wait) { /* Delay helper for no action */
    pilot_wait--;
    return;
  }

  if (event_course == 1) { /* Catch course event and reset*/
    evt_course = 1;
    event_course = 0;
  }

  /*Manoevers may consist of several actions, so we trap 
    them here. The basic manoever is zero, which means to
    just fly in the right direction. */

  if (pilot_manoever == 0) {
    /*Fly from A to B: 
      First step: hover */
    if (pilot_manoever_step == 0) {
      drone_hover();
      pilot_manoever_step++;
      return;
    }
    /*Second step: Wait for GPS data to settle */
    if (pilot_manoever_step == 1) {
      pilot_wait = 4;
      pilot_manoever_step++;
      return;
    }
    /*Point head towards target */
    if (pilot_manoever_step == 2) {
      pilot_point(course_angle);
      pilot_wait = 2;
      pilot_manoever_step++;
      return;
    }
    /*Actually flying now */
    /*Start flying straight ahead */
    if (pilot_manoever_step == 3) {
      /* Forward 70% */
      pitch = -700;
      drone_move(roll, pitch, gaz, yaw);
      /* Reset course calculation*/
      set_mycourse(-1); 
      pilot_manoever_step++;
      return;
    } 
    /*Wait for course update */
    if (pilot_manoever_step == 4) {
      pitch = -700;
      if (evt_course){
        pilot_manoever_step++;
        /*Check if drone heading and course match */
        Serial.print("$ECHO,#INFO GPS course:");
        Serial.print(course_b,0);
        Serial.print(" drone angle:");
        Serial.print(drone_angle);
        Serial.print("\n");
        int diff = course_diff(drone_angle, course_b);
        if (abs(diff) > 15) {
          Serial.print("$ECHO,#WARN! course / heading diff:");
        }
        else {
          Serial.print("$ECHO,#INFO course / heading diff:");
          /*Stash the heading error in the error value for pointing.*/
          drone_angle_err -= diff;
          /*Correct drone_angle to what was measured*/
          drone_angle += diff;
        }
        Serial.print(diff);
        Serial.print("\n");

        /*Set the next waypoint height*/
        drone_alti(currwp_height, 0);
      }
      drone_move(roll, pitch, gaz, yaw);
      return;
    }

    /*Check course */ 
    if (pilot_manoever_step == 5) {
      /*Switch to near mode if near target*/
      if (near()){
        pilot_manoever_step = 10;
      }
      else {
        /*Is drone in nav corridor */
        if(check_in_corridor(4)) {
          pilot_manoever_step = 6;
        }
        else {
          pilot_manoever_step = 8;
        }
      }
    }

    if (pilot_manoever_step == 6) {
      /*In corridor*/
      /*Face towards target */
      if (drone_angle != course_angle)
        pilot_point(course_angle);
      
      pilot_manoever_step++;
      return;
    }

    if (pilot_manoever_step == 7) {
      /*In corridor*/
      /*Forward, hard*/
      move_toward(drone_angle, course_to_wp, 900, &pitch, &roll);
      pitch = -900;
      roll = roll / 2;
      drone_move(roll, pitch, gaz, yaw);
      
      if (!check_in_corridor(4)) {
        pilot_manoever_step = 5;
      }
      return;
    }


    if (pilot_manoever_step == 8) {
      /*Out of corridor*/
      /*Face towards target */
      if (drone_angle != course_to_wp)
        pilot_point(course_to_wp);
      pilot_manoever_step++;
      return;
    }

    if (pilot_manoever_step == 9) {
      /*Out of corridor*/
      /*lean in hard*/
      move_toward(drone_angle, course_to_wp, 900, &pitch, &roll);
      pitch = pitch / 2;
      drone_move(roll, pitch, gaz, yaw);
      if (check_in_corridor(4)) {
        pilot_manoever_step = 5;
      }
      return;
    }
    
    
    /*Near to target*/
    if (pilot_manoever_step == 10) {
      /*Point head to target*/
      if (drone_angle != course_to_wp)
        pilot_point(course_to_wp);
      pilot_manoever_step++;
      return;
    }
    if (pilot_manoever_step == 11) {
      int speed = distance_to_wp * distance_to_wp;
      if (speed < 200)
        speed=200;
      if (speed > 500)
        speed=500;
      /*Fly to target by leaning in only*/
      move_toward(drone_angle, course_to_wp, speed, &pitch, &roll);
      drone_move(roll, pitch, gaz, yaw);
      return;
    }
  }
 
  
  
/* GPS altitude is not good enough - need to work it out.
  if (alti > 2 &&  
      abs(alti - currwp_height) > 2) {
    if (alti - currwp_height > 0)
      gaz = -200;
    else
      gaz = 200;
  }
*/

  logfsm();
//  if (alti < 0)
//    alti = 0;
    
//  Serial.print("$ERRO,");
//  Serial.print(alti);
//  Serial.print(",");
//  Serial.print(fc,0);
//  Serial.print("\n");


  Serial.print("$ECHO,POS:");
  Serial.print(flat,10);
  Serial.print(",");
  Serial.print(flon,10);
  Serial.print(",");
  Serial.print(falt,0);
  Serial.print(",");
  Serial.print(alti);
  Serial.print("\n");
  
  Serial.print("$PRIN\n");
}

void pilot_end()
{
  delay(20000);
  Serial.print("$STAT,0\n");
  Serial.print("$RECO,0\n");
}

void pilot_setup()
{
  Serial.print("\n\n$QUIT\n");
  delay(1500);
  Serial.print("cat - | /data/video/usb/arpilot2.arm > /data/video/usb/plog.txt\n");
  delay(20000);
  Serial.print("cat - | /data/video/usb/arpilot2.arm > /data/video/usb/plog.txt\n");

  delay(2000);

  Serial.print("$ECHO,#arpilot sketch version: ");
  Serial.print(VERSION);
  Serial.print("\n");

  Serial.print("$ECHO,#Flight description: ");
  Serial.print(wp_location);
  Serial.print("\n");

  /*Limits: max_yaw 2.50
             max_vz  700
          max_euler 0.30
           altitude 50000 (50m)*/
  Serial.print("$LIMI,250,700,30,50000\n"); /*Not sure that this works yet*/
}

void pilot_liftoff()
{
  Serial.print("$STAT,1\n");
  delay(10000);
  Serial.print("$CALI\n");
  delay(6000);
  Serial.print("$ALTI,1500,0\n");
  delay(3000);
  Serial.print("$RECO,1\n");
  delay(2000);
/*
  delay(10000);
  Serial.print("$STAT,0\n");
  delay(360000);
*/
}

void pilot_command_fly(){
  pilot_manoever = 0;
  pilot_manoever_step = 0;
}

