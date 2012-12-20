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


#include <poll.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/time.h>

#include "drone.h"
#include "states.h"

#include "command.h"
#include "emitter.h"
#include "pilot.h"
#include "monitor.h"
#include "navdata.h"
#include "network.h"
#include "web.h"

unsigned int last_timestamp_nav = 0;

char cmdbuffer[BUFLEN];
int  http_fd = 0;

int process()
{
    struct pollfd fds[4];
    int    nfds;
    int    rc,i,timeout,ret,mret;
    struct timeval ts;
    struct timezone tz;
    int flag_no_navdata = 0;

    timeout = 20;
    ret = 0;

    if (strlen(cmdbuffer) > 0) {
    	udp_send_command(cmdbuffer);
    	bzero(cmdbuffer,BUFLEN);
    }

    gettimeofday(&ts,&tz);
    last_timestamp_nav=ts.tv_sec*1000+ts.tv_usec/1000 + 5000;

    nfds = 0;

    if (vid_state == 1){
        fds[nfds].fd = vid_sock;
        fds[nfds++].events = POLLIN;
    }

    if (nav_state == 1){
        fds[nfds].fd = nav_sock;
        fds[nfds++].events = POLLIN;
    }

    if (web_state == 1){
        if (http_fd){
            fds[nfds].fd = http_fd;
            fds[nfds++].events = POLLIN;
        }
        else {
            fds[nfds].fd = web_sock;
            fds[nfds++].events = POLLIN;
        }
    }

    // Loop step 1 : Read stdin

//      printf("Waiting on poll()...\n");
    rc = poll(fds, nfds, timeout);

    if (rc < 0) {
        perror("  poll() failed");
    }

    if (rc == 0) {
//          printf("  poll() timed out.\n");
        // if no input for 2s hover the drone
    }
    else {
        for (i = 0; i < nfds; i++) {
            if(fds[i].revents == 0)
            continue;

            if(fds[i].revents != POLLIN) {
            	printf("  Error! revents = %d\n", fds[i].revents);
//                    break;

            }
            if (fds[i].fd == nav_sock) {
                int size;
//                      printf("  Listening nav socket is readable\n");
                gettimeofday(&ts,&tz);
                last_timestamp_nav=ts.tv_sec*1000+ts.tv_usec/1000;
                        size = recvfrom (nav_sock, navdata_buffer, NAVDATA_BUFFER_SIZE, 0,
                        (struct sockaddr *)&si_nav,
                        (socklen_t *)&slen);
                decode_navdata(navdata_buffer,size);
            }
            if (fds[i].fd == vid_sock) {

            	int size;
            	char tmpbuffer[8192];
//                      printf("  Listening vid socket is readable\n");
            	size = read (vid_sock, tmpbuffer, 8192);

            	send_vid_data(tmpbuffer,size);

//            	printf("Siz=%d\n",size);
            }
            if (fds[i].fd == web_sock) {
            	http_fd = accept_web(web_sock);
            }
            if (fds[i].fd == http_fd) {
            	handle_web(http_fd);
            	http_fd = 0;
            }
        }
    }

    /* Timeout Handling
    *   Timeouts are handled diffenently for input and navdata.
    * */

    gettimeofday(&ts,&tz);

    if ((ts.tv_sec*1000+ts.tv_usec/1000) > last_timestamp_nav+2000) {
        // nav timeout
        if (!flag_no_navdata )
            printf("!WARN:navdata\n");
            navdata_valid = 0;
        flag_no_navdata = 1;
    }
    else {
        flag_no_navdata = 0;
    }

    // If waiting for a config confirmation wait until the COMMAND_MASK has
    // gone to 1 and then been reset.
    if (config_confirm_wait == 1) {
        if (navdata_unpacked.mykonos_state & MYKONOS_COMMAND_MASK) {
        	config_confirm_wait++;
            char buffer[BUFLEN];
            bzero(buffer,BUFLEN);

            addATCTRL(buffer,5,0);
            udp_send_command(buffer);
//            printf("ACK=1\n");
        }
        else {
        	return 2; // AGAIN
        }
    }
    else {
    	if (config_confirm_wait == 2) {
            if (!(navdata_unpacked.mykonos_state & MYKONOS_COMMAND_MASK)) {
            	config_confirm_wait = 0;
//                printf("ACK=0\n");
            }
            else {
            	return 2; // AGAIN
            }
    	}
    }

    // Execute automations
    if (auto_pilot()){
        printf("P\n");
        ret = 4;
    }

    // Monitor limits and alerts
	// if monitor reuturns 128 it wants to run again
    mret = auto_monitor();

    if (mret == 128)
    	return 2;
    else
    	ret = ret | (mret << 8);

    // Send an update packet to the drone
    // also: current flight instructions
    update_drone();

    return ret;
}
