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

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>
#include <termios.h>
#include <string.h>

#include <sys/time.h>

#include "drone.h"

#include "command.h"
#include "controller.h"
#include "init.h"
#include "log.h"
#include "network.h"
#include "web.h"

char commandbuffer[BUFLEN];
int  commandbufferlen=0;

char command[80];
int  commandlen = 0;

unsigned int last_timestamp_io  = 0;

int exit_now = 0;

int execcommand()
{
    int ret=-1;

    int index = 5;
    int args = 0;
    int arg[8];
    float farg[8];

    while (index < commandlen)
        if (command[index++] == ',')
            args++;

    //printf("Args: %i\n",args);

    if(args > 8)
        return -1;

    index=5;
    args=0;

    while (index < commandlen)
        if (command[index++] == ','){
        	if (strncmp((char*)(command+index),".", commandlen - index) < 0)
        		arg[args]=atoi((char*)(command+index));
        	else {
        		farg[args]=atof((char*)(command+index));
        		arg[args]=(int)farg[args];
        	}
//            printf("Arg[%i] : %i \n",args,arg[args]);
//            printf("Farg[%i]: %f \n",args,farg[args]);
            args++;
    }

//  printf("Cmd: %s\n",command);
    logmsg(command,commandlen);

    if ((strncmp(command,"$PLON",5) == 0) && args == 1)
        ret=command_set_lon(farg[0]);

    if ((strncmp(command,"$PLAT",5) == 0) && args == 1)
        ret=command_set_lat(farg[0]);

    if ((strncmp(command,"$PALT",5) == 0) && args == 1)
        ret=command_set_alt(arg[0]);

    if ((strncmp(command,"$PCOU",5) == 0) && args == 1)
        ret=command_set_course(arg[0]);

    if ((strncmp(command,"$PHDO",5) == 0) && args == 1)
        ret=command_set_hdop(arg[0]);

    if ((strncmp(command,"$MOVE",5) == 0) && args == 4)
    	ret=command_move(arg[0],arg[1],arg[2],arg[3]);

    if ((strncmp(command,"$STAT",5) == 0) && args == 1)
        ret=command_state(arg[0]);

    if ((strncmp(command,"$TRIM",5) == 0) && args == 0)
        ret=command_trim();

    if ((strncmp(command,"$HOVE",5) == 0) && args == 0)
        ret=command_hover();

    if ((strncmp(command,"$CALI",5) == 0) && args == 0)
        ret=command_cali();

    if ((strncmp(command,"$PRIN",5) == 0) && args == 0)
        ret=command_print();

    if ((strncmp(command,"$TURN",5) == 0) && args == 1)
        ret=command_turn(arg[0]);

    if ((strncmp(command,"$RISE",5) == 0) && args == 1)
        ret=command_rise(arg[0]);

    if ((strncmp(command,"$RECO",5) == 0) && args == 1)
        ret=command_record(arg[0]);

    if ((strncmp(command,"$HEAD",5) == 0) && args == 1)
        ret=command_head(arg[0]);

    if ((strncmp(command,"$ALTI",5) == 0) && args == 2)
        ret=command_alti(arg[0],arg[1]);

    if ((strncmp(command,"$ERRO",5) == 0) && args == 2)
        ret=command_error(arg[0],arg[1]);

    if ((strncmp(command,"$LIMI",5) == 0) && args == 4)
        ret=command_limit(arg[0],arg[1],arg[2],arg[3]);

    if (strncmp(command,"$QUIT",5) == 0)
        exit_now = 1;

    if (strncmp(command,"$ECHO",5) == 0) {
    	printf("%s\n",(char*)command+6);
    	ret=0;
    }

    commandlen=0;
    return ret;
}

int parsecommand()
{
    int index=0;
    int cmdlen=0;
    char *dest;

    while (commandbuffer[index] != '$') {
        index++;
        if (index == BUFLEN || index == commandbufferlen){
            commandbufferlen=0;
            return 1;
        }
    }

    while (commandbuffer[index+cmdlen] != '\n') {
        cmdlen++;

        if (index+cmdlen > commandbufferlen){
            return 1;
        }
    }

    bzero (command,80);
    dest = (char*)(commandbuffer+index);
    strncpy (command,dest,cmdlen);
    commandlen=cmdlen;

    cmdlen++;  //also skip over the \n

    dest = (char*)(commandbuffer+index+cmdlen);
        commandbufferlen=commandbufferlen-index-cmdlen;
    strncpy (commandbuffer,dest,commandbufferlen);
    bzero((void*)(commandbuffer+commandbufferlen),BUFLEN-commandbufferlen);

//  printf("Command: %s\n",command);

    return 0;
}

void readstdin()
{
    char buffer[80];
    int read;
    char *dest = (char*)(commandbuffer+commandbufferlen);

    read=fread(buffer,1,80,stdin);

    if (read + commandbufferlen < BUFLEN) {
        commandbufferlen+=read;
        strncpy(dest,buffer,read);
    } else {
        printf("\nCommand buffer is full !\n");
    }
}

int main()
{
	struct pollfd fds[4];
	int    nfds;

	int    rc,i,timeout;
	struct timeval ts;
	struct timezone tz;

	int flag_no_command = 0;
    struct termios ttystate, ttysave;

    // Prepare the terminal, make stdin NONBLOCKing
    tcgetattr(STDIN_FILENO, &ttystate);
    ttysave = ttystate;
    ttystate.c_lflag &= ~(ICANON | ECHO);
    ttystate.c_cc[VMIN] = 1;

    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

    fcntl(0, F_SETFL, O_NONBLOCK);

    printf("tty \n");

    if (setup_net())
    	exit(1);
    printf("net \n");

    if (init_navdata())
    	exit(1);
    printf("nav \n");

//    if (init_vid())
//    	exit(1);
//    printf("vid \n");

    // Init the drone
    if (config_init())
    	exit(1);
    printf("ini \n");

    if (init_web())
    	exit(1);
    printf("web \n");

    // Start the main loop

    timeout = 20;

    printf("loop: \n");
    while (!exit_now) {
		nfds = 0;
		fds[nfds].fd = 0;
		fds[nfds++].events = POLLIN;

		rc = poll(fds, nfds, timeout);

		if (rc < 0) {
			perror("  poll() failed");
			break;
		}

		if (rc == 0) {
//			printf("  poll() timed out.\n");
			// if no input for 2s hover the drone
		} else {
			for (i = 0; i < nfds; i++) {
				if(fds[i].revents == 0)
				continue;

      				if(fds[i].revents != POLLIN) {
        				printf("  Error! revents = %d\n", fds[i].revents);
        				break;

      				}
      				if (fds[i].fd == 0) {

//        				printf("  Listening socket is readable\n");
					gettimeofday(&ts,&tz);
					last_timestamp_io=ts.tv_sec*1000+ts.tv_usec/1000;
					readstdin();
				}
			}
		}

		/* Timeout Handling
 		*	Timeouts are handled diffenently for input and navdata.
 		* */

		gettimeofday(&ts,&tz);
		if ((ts.tv_sec*1000+ts.tv_usec/1000) > last_timestamp_io+2000) {
			// io timeout
			if (!flag_no_command )
				command_idle();
			flag_no_command = 1;
		} else {
			flag_no_command = 0;
		}

		// Loop step 2 : Parse buffer
		parsecommand();

		// Loop step 3 : Exec command
		if (commandlen > 0){
			if (execcommand())
			    printf("!ERR\n");
			else {
			    printf(".\n");
			}
		}

		// Process drone data
		while (process() == 2)
			usleep(20000);

    }

    ttystate.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

    return 0;
}
