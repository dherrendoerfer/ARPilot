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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "command.h"
#include "states.h"
#include "arpilot.h"

int emergency_timestamp = 0;
int emergency_active = 0;

int auto_monitor()
{
	int ret = 0;

    if (navdata_valid) {
        /* Low Battery helpers
         * 10% -> go to 5m altitude if above 6m
         *  5% -> Land                            */
         if (navdata_unpacked.navdata_demo.vbat_flying_percentage < 10){
        	 if (navdata_unpacked.navdata_demo.altitude + err_drone_altitude > 6000) {
        		 pilot_altitude = 5000;
        		 pilot_alti = 1;
        		 ret = 1;
        	 }
        }

        if (navdata_unpacked.navdata_demo.vbat_flying_percentage < 5){
            if (drone_fly == 1){
                drone_fly = 0;
                ret = 2;
            }
        }

        /* Flight height helpers
         * < 60cm -> go to 85cm altitude         */
        if (drone_fly && (navdata_unpacked.navdata_demo.altitude < 600)){
            pilot_altitude = 850;
            pilot_alti = 1;
            ret = 4;
        }

        /* Flight height helpers
         * try to hold the altitude exact to .5m          */
        if (drone_fly && monitor_alti && !pilot_alti) {
            if ( abs(navdata_unpacked.navdata_demo.altitude
                 + err_drone_altitude - pilot_altitude) > 500) {
                int alt = navdata_unpacked.navdata_demo.altitude
                      + err_drone_altitude;

                if(alt < pilot_altitude){
                    drone_gaz = 200;
                }
                else {
                    drone_gaz = -200;
                }
                ret = 8;
            }
        }

        /* Emergency detect
         * Switch off video recording after 20s  */
        if (navdata_unpacked.mykonos_state & MYKONOS_EMERGENCY_MASK) {
            struct timeval ts;

            ret = 16;

            gettimeofday(&ts, 0);

            if (!emergency_active) {
                emergency_timestamp=ts.tv_sec;
                emergency_active = 1;
                printf("!EMER\n");
            }
            else {
                if(is_recording && (emergency_active == 1)&& (ts.tv_sec == emergency_timestamp + 20)) {
                    // stop recording
                    command_record(0);
                    emergency_active = 2;
                    ret = 128;
                }
                else if((emergency_active == 2) && (ts.tv_sec == emergency_timestamp + 25)) {
                	// record till the end ....
                    command_record(1);
                    emergency_active = 3;
                    ret = 128;
                }
            }
        }
        else {
        	if (emergency_active)
        		emergency_active = 0; // reset emergency flag .... continue
        }
    }

    return ret;
}
