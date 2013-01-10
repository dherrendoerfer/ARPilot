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
 
void drone_hover()
{
  Serial.print("$HOVE\n");
}

void drone_setpos()
{
  Serial.print("$PLON,");
  Serial.print(flon,10);
  Serial.print("\n");
  
  Serial.print("$PLAT,");
  Serial.print(flat,10);
  Serial.print("\n");

  Serial.print("$PALT,");
  Serial.print(falt,0);
  Serial.print("\n");
  
  Serial.print("$PCOU,");
  Serial.print(course_b,10);
  Serial.print("\n");
  
  Serial.print("$HDOP,");
  Serial.print(gps.hdop());
  Serial.print("\n");
  
}

void drone_move(int roll, int pitch, int gaz, int yaw)
{
  int tpitch  = -1;
  int troll   = -1;
   
  translate_pitch_roll(pitch, roll, camera_angle, &tpitch, &troll);

  Serial.print("$MOVE,");
  Serial.print(troll);
  Serial.print(",");
  Serial.print(tpitch);
  Serial.print(",");
  Serial.print(gaz);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print("\n");
}

void drone_alti(float height, int monitor_on)
{ 
  Serial.print("$ALTI,");
  Serial.print((height*1000),0);
  Serial.print(",0\n");
}

void drone_head(float direction)
{
  /*Remember, this is where we want to point*/
  drone_angle = direction;

  direction = angle_sum(direction,camera_angle);
  direction = angle_sum(direction,drone_angle_err);
    
  Serial.print("$HEAD,");
  Serial.print(direction*1000, 0);
  Serial.print("\n");
}

void drone_turn(float direction)
{
  
  Serial.print("$TURN,");
  Serial.print(direction*1000, 0);
  Serial.print("\n");

  drone_angle += direction;
  
  if (drone_angle > 360)
    drone_angle -= 360;
  if (drone_angle < 0)
    drone_angle += 360;
}

float drone_translated_angle(int pitch, int roll) 
{
  float result = 0;
  
  if (pitch == 0){
    if (roll > 0)
      return drone_angle + 90;
    else
      return drone_angle - 90;
  }
  
  result = drone_angle + atan(roll/-pitch);
  
  return result;
}
