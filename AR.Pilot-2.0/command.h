/*
 * This file is part of libarpilot.
 *
 * Copyright (C) 2012  D.Herrendoerfer
 *
 *   libarpilot is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   libarpilot is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser Public License
 *   along with libarpilot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#ifndef COMMAND_H_
#define COMMAND_H_

void set_command_timestamp();

int command_set_lon(float lon);

int command_set_lat(float lat);

int command_set_alt(int lat);

int command_set_course(int course);

int command_set_hdop(int hdop);

int command_state(int state);

int command_move(int roll, int pitch, int gaz, int yaw);

int command_trim(void);

int command_hover(void);

int command_cali(void);

int command_print(void);

int command_record(int state);

int command_head(int angle);

int command_alti(int altitude, int monitor_state);

int command_turn(int delta_angle);

int command_rise(int delta_altitude);

int command_error(int my_alti, int my_phi);

int command_limit(int yaw, int vz, int angle, int altitude);

void command_idle();

void update_drone();

#endif /* COMMAND_H_ */
