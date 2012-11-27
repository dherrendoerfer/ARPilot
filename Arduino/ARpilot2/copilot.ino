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
 
int adj_head()
{
  int mhead=angle_diff( course_to_wp,course_b);

  if (reached())
    return 0;

#ifdef DEBUG
  Serial.print("mhead: ");
  Serial.println(mhead);
  Serial.print("fc: ");
  Serial.println(fc);
  Serial.print("hdop: ");
  Serial.println(gps.hdop());
#endif

  if ( mhead > 344 )
    return 1;
  if ( mhead > 270 )
    return 5;
  if ( mhead > 180 )
    return 4;
  if ( mhead > 90 )
    return 3;
  if ( mhead > 15 )
    return 2;
  return 1;
}

int check_in_corridor(int width)
{
  float off_path_distance = course_delta_m( course_err, distance_from_wp);
  
  if (abs(off_path_distance) > width/2)    /*Left the corridor*/
    return 0;
    
  if (distance_from_wp > (way_dist + 10))  /*Overshot the Target*/
    return 0;

  return 1;
}

int check_course()
{
  if (abs(course_err) > 30)
    return 1;

  return 0;
}

void translate_pitch_roll(int pitch, int roll, float angle, int *tpitch, int *troll)
{
  *tpitch = (int)(pitch*cos(angle*PI/180) + roll*sin(angle*PI/180));
  *troll = (int)-1*(roll*cos(angle*PI/180) + pitch*sin(angle*PI/180));
}

void move_toward(float my_angle, float target_angle, int speed, int *tpitch, int *troll)
{
  float real_angle = course_diff(my_angle,target_angle);
  
  translate_pitch_roll(-1*speed,0,real_angle, tpitch, troll);
}



