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

#include "drone.h"

#include "controller.h"
#include "emitter.h"
#include "network.h"

#include "states.h"

int  ini_cycle;
int  init_state = 0;

float config_max_yaw       = 1.75;
float config_max_vz        = 700.0;
float config_euler_max     = 0.1;
int   config_altitude_max  = 10000;


void init()
{
    char tmp[64];
    struct timeval tv;

    switch (ini_cycle++)
    {
        case 10:
            setSESSIONID(cmdbuffer, "da110000");
            addATCONFIG(cmdbuffer,"custom:session_id","-all");
            break;
        case 20:
            addATCONFIG(cmdbuffer,"custom:session_id","-all");
            break;
        case 30:
            setSESSIONID(cmdbuffer, "da110001");
            break;
        case 32:
            setAPPLICATIONID(cmdbuffer, "da110003");
            break;
        case 34:
            setPROFILEID(cmdbuffer, "da110002");
            break;
        case 40:
            addATCONFIG(cmdbuffer,"video:bitrate_ctrl_mode", "1");
            break;
        case 50:
            addATCONFIG(cmdbuffer,"custom:application_desc", "com.dh.libarpilot");
            break;
        case 52:
            addATCONFIG(cmdbuffer,"custom:profile_desc", "libarpilot client");
            break;
        case 54:
            addATCONFIG(cmdbuffer,"custom:session_desc", "libarpilot session");
            break;
        case 60:
            addATCONFIG(cmdbuffer,"general:navdata_demo", "TRUE");
            break;
        case 70:
        	gettimeofday(&tv,0);
        	sprintf(tmp,"%d",(int)tv.tv_sec);
            addATCONFIG(cmdbuffer,"general:localtime", tmp);
            break;
        case 80:
            addATCONFIG(cmdbuffer,"control:outdoor", "TRUE");
            break;
        case 90:
            sprintf(tmp,"%#.2f",config_max_yaw);
            addATCONFIG(cmdbuffer, "control:control_yaw", tmp);
            break;
        case 100:
            sprintf(tmp,"%#.2f",config_max_vz);
            addATCONFIG(cmdbuffer, "control:control_vz_max", tmp);
            break;
        case 110:
            sprintf(tmp,"%#.2f",config_euler_max);
            addATCONFIG(cmdbuffer, "control:euler_angle_max", tmp);
            break;
        case 120:
            sprintf(tmp,"%d",config_altitude_max);
            addATCONFIG(cmdbuffer, "control:altitude_max", tmp);
            break;
        case 130:
            addATCONFIG(cmdbuffer,"control:control_level", "1");
            break;
        case 140:
            addATCONFIG(cmdbuffer,"video:video_on_usb", "TRUE");
            break;
        case 150:
            addATCONFIG(cmdbuffer,"video:video_channel", "2");
            break;
        case 160:
            addATCONFIG(cmdbuffer,"video:codec_fps", "25");
            break;
        case 170:
            addATCONFIG(cmdbuffer,"video:video_codec", "129");
            break;
        case 180:
            addATCONFIG(cmdbuffer,"video:max_bitrate", "1500");
            break;
        case 200:
            init_state = 1;
            printf("Init complete.\n");
            break;
    }
}

int config_init()
{
	int count = 0;

	while (!navdata_valid) {
		if (count++ == 500) {
			fprintf(stderr,"timeout, no valid navdata\n");
			return -1;
		}
		process();
	}

    count = 0;

    while (!init_state) {
    	init();
    	while (process() == 2);
    }

	return 0;
}
