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

#include <assert.h>

#include "emitter.h"
#include "states.h"

int  seq = 1;
int  config_confirm_wait = 0;

char c_session[9] = "00000000";
char c_profile[9] = "00000000";
char c_application[9] = "00000000";

void addATCTRL(char *buffer, int key, int val)
{
    sprintf(buffer+strlen(buffer),"AT*CTRL=%d,%d,%d\r", seq++, key, val);
}

void addATCONFIGID(char *buffer, char *session, char *profile ,char *application)
{
    sprintf(buffer+strlen(buffer),"AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", seq++,
                                       session, profile, application);
}

void addATCONFIG(char *buffer, char* key, char* val)
{
	addATCONFIGID(buffer, c_session, c_profile ,c_application);
    sprintf(buffer+strlen(buffer),"AT*CONFIG=%d,\"%s\",\"%s\"\r", seq++, key, val);
    config_confirm_wait = 1;
}

void setSESSIONID(char *buffer, char *session)
{
	addATCONFIG(buffer,"custom:session_id", session);
	strncpy(c_session, session, 8);
}

void setPROFILEID(char *buffer, char *profile)
{
	addATCONFIG(buffer,"custom:profile_id", profile);
	strncpy(c_profile, profile, 8);
}

void setAPPLICATIONID(char *buffer, char *application)
{
	addATCONFIG(buffer,"custom:application_id", application);
	strncpy(c_application, application, 8);
}

void addATPCMD(char *buffer, int roll, int pitch, int gaz, int yaw )
{
    assert(sizeof(int)==sizeof(float));
    int i_arg0 = drone_pcmd_flags;
    float f_arg1 = (float)roll  / 1000;
    float f_arg2 = (float)pitch / 1000;
    float f_arg3 = (float)gaz   / 1000;
    float f_arg4 = (float)yaw   / 1000;

    sprintf(buffer+strlen(buffer),"AT*PCMD=%d,%d,%d,%d,%d,%d\r",
    seq++,
    i_arg0,
    *(int*)(&f_arg1),
    *(int*)(&f_arg2),
    *(int*)(&f_arg3),
    *(int*)(&f_arg4) );
}

void addATCWDG(char *buffer)
{
    sprintf(buffer+strlen(buffer),"AT*COMWDG=%d\r", seq++);
}

void addATREF(char *buffer, int val)
{
    sprintf(buffer+strlen(buffer),"AT*REF=%d,%d\r", seq++,val);
}

void addATFTRIM(char *buffer)
{
    sprintf(buffer+strlen(buffer),"AT*FTRIM=%d\r", seq++);
}

void addATCALIB(char *buffer)
{
    sprintf(buffer+strlen(buffer),"AT*CALIB=%d,0\r", seq++);
}

