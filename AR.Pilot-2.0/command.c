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

#include "states.h"
#include "arpilot.h"

#include "controller.h"
#include "emitter.h"
#include "network.h"
#include "web.h"

int drone_init  = 0; // Initialized
int drone_fly   = 0; // Flying
int drone_roll  = 0; // Roll
int drone_pitch = 0; // Pitch
int drone_gaz   = 0; // Gaz
int drone_yaw   = 0; // Yaw

// Record command var
int is_recording = 0;

// Autopilot command vars
int pilot_heading = 0;
int pilot_head = 0;

int pilot_altitude = 0;
int pilot_alti = 0;
int monitor_alti = 0;

// Drone correction values
int err_drone_altitude = 0;
int err_drone_phi = 0;

int drone_pcmd_flags = 0;
int drone_ref_flags  = 0;

float pos_lon;
float pos_lat;
int   pos_alt;
int   pos_course;
int   pos_hdop = 9999;

void set_command_timestamp()
{
    struct timeval ts;
    struct timezone tz;

    gettimeofday(&ts,&tz);
    last_timestamp_command=ts.tv_sec*1000+ts.tv_usec/1000;
}

int command_set_lon(float lon)
{
	pos_lon = lon;
	return 0;
}

int command_set_lat(float lat)
{
	pos_lat = lat;
	return 0;
}

int command_set_alt(int alt)
{
	pos_alt = alt;
	return 0;
}

int command_set_course(int course)
{
	pos_course = course;
	return 0;
}

int command_set_hdop(int hdop)
{
	pos_hdop = hdop;
	return 0;
}

int command_state(int state)
{
    if (state == 0) {
        pilot_alti = 0;
        pilot_head = 0;
        drone_fly = 0;
    }

    if (state == 1)
        drone_fly = 1;

    if (state == 666){
        // Send an emergency message block
        seq=1;
        addATREF(cmdbuffer, DRONE_REF_FLAG_BASIC);
        addATREF(cmdbuffer, DRONE_REF_FLAG_BASIC | DRONE_REF_FLAG_EMERGENCY);
        addATREF(cmdbuffer, DRONE_REF_FLAG_BASIC);

        pilot_alti = 0;
        pilot_head = 0;
        drone_fly = 0;
    }

    if (state == 777){
        // Send a recovery message block
        addATREF(cmdbuffer, DRONE_REF_FLAG_BASIC | DRONE_REF_FLAG_EMERGENCY);

        pilot_alti = 0;
        pilot_head = 0;
        drone_fly = 0;
    }

    set_command_timestamp();

    return 0;
}
int command_move(int roll, int pitch, int gaz, int yaw)
{
    drone_roll       = roll;
    drone_pitch      = pitch;

    drone_gaz  = gaz;
    if (gaz != 0) {
        // cancel a running autopilot
        pilot_alti = 0;
        monitor_alti = 0;
    }

    drone_yaw  = yaw;
    if (yaw != 0) {
        // cancel a running autopilot
        pilot_head = 0;
    }

    drone_pcmd_flags = DRONE_PCMD_FLAG_PROGRESSIVE;

    set_command_timestamp();

    return 0;
}

int command_trim(void)
{
    addATFTRIM(cmdbuffer);

    return 0;
}

int command_hover(void)
{
    drone_roll       = 0;
    drone_pitch      = 0;
    drone_gaz        = 0;
    drone_yaw        = 0;
    drone_pcmd_flags = 0;

    pilot_alti = 0;
    pilot_head = 0;
    monitor_alti = 0;

    return 0;
}

int command_cali(void)
{
    addATCALIB(cmdbuffer);

    set_command_timestamp();

    return 0;
}

int command_print(void)
{
    if(navdata_valid){

        float32_t heading = navdata_unpacked.navdata_demo.psi;
        if (heading < 0)
            heading += 360000;
        //clock_t uptime = clock() / (CLOCKS_PER_SEC / 1000);
//        clock_t uptime = clock();


        printf("$STATUS: Current drone state.\n");
        printf("$STAT:altitude,%d\n",navdata_unpacked.navdata_demo.altitude);
        printf("$STAT:batt,%d\n",navdata_unpacked.navdata_demo.vbat_flying_percentage);
        printf("$STAT:state,%d\n",navdata_unpacked.navdata_demo.ctrl_state);
        printf("$STAT:theta,%f\n",navdata_unpacked.navdata_demo.theta);
        printf("$STAT:phi,%f\n",navdata_unpacked.navdata_demo.phi);
        printf("$STAT:psi,%f\n",heading);
//        printf("$STAT:timestamp_io,%u\n",last_timestamp_io);
        printf("$STAT:timestamp_nav,%u\n",last_timestamp_nav);
        printf("$STAT:err_phi,%u\n",err_drone_phi);
        printf("$STAT:err_altitude,%u\n",err_drone_altitude);
    }

    return 0;
}

int command_record(int state)
{
    char tmp[64];

    if (state == 1)
        addATCONFIG(cmdbuffer,"userbox:userbox_cmd", "1,20120405_121400");

    if (state == 0)
        sprintf(tmp,"%d",H264_360P_CODEC);
    if (state == 1)
        sprintf(tmp,"%d",MP4_360P_H264_720P_CODEC);

    addATCONFIG(cmdbuffer,"video:video_codec", tmp);

    if (state == 0)
        addATCONFIG(cmdbuffer,"userbox:userbox_cmd", "0");

    is_recording = state;

    return 0;
}

int command_head(int angle)
{
    pilot_heading=angle;
    pilot_head=1;

    set_command_timestamp();

    return 0;
}

int command_alti(int altitude, int monitor_state)
{
    pilot_altitude=altitude;
    pilot_alti=1;

    monitor_alti = monitor_state;

    set_command_timestamp();

    return 0;
}

int command_turn(int delta_angle)
{
    if (navdata_valid){
        float32_t heading = navdata_unpacked.navdata_demo.psi;
        if (heading < 0)
                        heading += 360000;

        heading += delta_angle;

        if (heading < 0)
                        heading += 360000;
        if (heading > 360000)
                        heading -= 360000;

        pilot_heading=heading;
        pilot_head=1;

    }
    else {
        return -1;
    }

    set_command_timestamp();

    return 0;
}

int command_rise(int delta_altitude)
{
    if (navdata_valid){
        /* Dont add height error here, we only increase by X */
        int altitude = navdata_unpacked.navdata_demo.altitude;

        altitude += delta_altitude;

        pilot_altitude=altitude;
        pilot_alti=1;
    }
    else {
        return -1;
    }

    set_command_timestamp();

    return 0;
}

int command_error(int my_alti, int my_phi)
{
    if (navdata_valid){
        int altitude = navdata_unpacked.navdata_demo.altitude;
//        int phi = navdata_unpacked.navdata_demo.phi;

        int err_phi = navdata_unpacked.navdata_demo.phi - my_phi;
        if (err_phi < 0)
        	err_phi += 360000;

        err_drone_altitude = ((7*err_drone_altitude) + altitude - (my_alti*1000) / 2 ) / 8;
        err_drone_phi = ((7*err_drone_phi) + err_phi) / 8;
    }

    return 0;
}

int command_limit(int yaw, int vz, int angle, int altitude)
{
    char tmp[64];

    float f_yaw = (float)yaw/100;
    float f_angle = (float)angle/100;

    sprintf(tmp,"%#.2f",f_yaw);
    addATCONFIG(cmdbuffer, "control:control_yaw", tmp);
    sprintf(tmp,"%d",vz);
    addATCONFIG(cmdbuffer, "control:control_vz_max", tmp);
    sprintf(tmp,"%#.2f",f_angle);
    addATCONFIG(cmdbuffer, "control:euler_angle_max", tmp);
    sprintf(tmp,"%d",altitude);
    addATCONFIG(cmdbuffer, "control:altitude_max", tmp);

    return 0;
}


void command_idle()
{
	//timeout, switch to hover
	drone_roll       = 0;
	drone_pitch      = 0;
	drone_gaz        = 0;
	drone_yaw        = 0;
	drone_pcmd_flags = 0;
}

void update_drone()
{
	int ref_flags  = DRONE_REF_FLAG_BASIC;
	char buffer[256];

	bzero(buffer,256);

	if (drone_fly) {
		ref_flags |= DRONE_REF_FLAG_START;
		addATPCMD(buffer, drone_roll, drone_pitch, drone_gaz, drone_yaw);
	}

	addATCWDG(buffer);
	addATREF(buffer, ref_flags);

	udp_send_command(buffer);
}
