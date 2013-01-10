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

#include <stdlib.h>

#include "states.h"
#include "command.h"
#include "arpilot.h"

int auto_pilot()
{
	if (pilot_head) {
		// Continously turn until head is looking into
		// right direction

		if (navdata_valid){
			float32_t heading = navdata_unpacked.navdata_demo.psi;
			if (heading < 0)
                        	heading += 360000;

			float32_t diff_angle = pilot_heading - heading;
			if (diff_angle < 0)
				diff_angle += 360000;

//			printf("diff: %f\n",diff_angle);

			if (diff_angle < 3000){
				drone_yaw  = 0;
				pilot_head = 0;
				return 1;
			}
			else {
				int speed = 300;
				if (abs(diff_angle > 35000))
					speed = 900;

				if (diff_angle > 180000){
					drone_yaw  = -1 * speed;
				}
				else {
					drone_yaw  = speed;
				}
				set_command_timestamp();
			}
		}
	}
	if (pilot_alti) {
		// Continously rise or sink until at the
		// right altitude

		int alt = navdata_unpacked.navdata_demo.altitude + err_drone_altitude;
		int alt_diff=abs(alt-pilot_altitude);

//		printf("diff: %f\n",alt_diff);

		if (alt_diff < 200){
			drone_gaz = 0;
			pilot_alti = 0;
			return 1;
		}
		else {
			int speed = 400;
			if (alt_diff > 800)
				speed = 900;

			if(alt < pilot_altitude){
				drone_gaz = speed;
			}
			else {
				drone_gaz = -1 * speed;
			}
			set_command_timestamp();
		}
	}

	return 0;
}
