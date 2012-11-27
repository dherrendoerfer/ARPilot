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
 
void logfsm ()
{
  Serial.print("$ECHO,#dist=");
  Serial.print(distance_to_wp);
  Serial.print(",cb=");
  Serial.print(course_b);
  Serial.print(",ctwp=");
  Serial.print(course_to_wp);
  Serial.print(",cerr=");
  Serial.print(course_err);
  Serial.print(",herr=");
  Serial.print(head_err);
  Serial.print("\n");
}

void lognavupdate()
{
  return;
  Serial.print("$ECHO,#NAVUPD;course_to_wp=");
  Serial.print(course_to_wp);
  Serial.print(",course_from_wp=");
  Serial.print(course_from_wp);
  Serial.print(",distance_to_wp=");
  Serial.print(distance_to_wp);
  Serial.print(",distance_from_wp=");
  Serial.print(distance_from_wp);
  Serial.print(",course_err=");
  Serial.print(course_err);
  Serial.print(",head_err=");
  Serial.print(head_err);
  Serial.print(",course_delta=");
  Serial.print(course_delta);
  Serial.print("\n");
}

void test()
{
  int tpitch = -1;
  int troll   = -1;
   
  move_toward(0, 0, -100, &tpitch, &troll);
  Serial.print("$ECHO,TEST Ahead:");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(troll);
  Serial.print("\n");
  
  move_toward(-45, 45, -100, &tpitch, &troll);
  Serial.print("$ECHO,TEST Ahead-Right:");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(troll);
  Serial.print("\n");
  
  move_toward(0, 90, -100, &tpitch, &troll);
  Serial.print("$ECHO,TEST Right:");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(troll);
  Serial.print("\n");
  
  move_toward(0, 180, -100, &tpitch, &troll);
  Serial.print("$ECHO,TEST Back:");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(troll);
  Serial.print("\n");
  
  move_toward(0, 270, -100, &tpitch, &troll);
  Serial.print("$ECHO,TEST Left:");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(troll);
  Serial.print("\n");
}
