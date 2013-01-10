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

#include "states.h"
#include "arpilot.h"

#include "web.h"


char logbuffer[100][LOGLENGTH];
int logcounter=0;

void logmsg(char *buffer,int len)
{
    int counter = logcounter++;

    /*Send to direct receivers, then archive*/
    send_log_data(buffer, len);

    if (len > LOGLENGTH-1 )
        len=LOGLENGTH -1;

    while ( counter >= 100)
        counter -= 100;

    bzero(logbuffer[counter], LOGLENGTH);
    strncpy(logbuffer[counter],buffer,len);
}

/*get the last index message, 0=newest, 99=oldest*/
int getlogmsg(int index, char *buffer)
{
    int lastmsg = logcounter -1;
    int thismsg;

    if (index > 99)
        return -1;

    while ( lastmsg >= 100)
        lastmsg -= 100;

    thismsg = lastmsg - index;

    if (thismsg < 0)
        thismsg += 100;

    strncpy(buffer, logbuffer[thismsg],LOGLENGTH);

    return 0;
}
