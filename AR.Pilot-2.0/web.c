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
#include <unistd.h>
#include <stdlib.h>

#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/netdevice.h>
#include <arpa/inet.h>

#include "states.h"
#include "command.h"
#include "log.h"
#include "arpilot.h"

#include "web_control.h"

#define LISTENQ           (16)
#define HTTP_PORT		  8080

struct sockaddr_in si_web;
int web_sock;

int web_state = 0;

int vid_sockets[5];
int log_sockets[5];


void send_index(int http_fd)
{
	char buffer[] = {
				  "</HEAD>"
				"<BODY>"
				  "<H1>Control Panel:</H1><P>"
				  "<H1>Please select:</H1><P>"
				  "<A href=command.html>Control</A><P>"
				  "<A href=stat.html>Status</A><P>"
				  "<A href=emer.html>Emergency</A><P>"
				"</BODY>"
			  "</HTML>"};

	send(http_fd,http_header,sizeof(http_header),MSG_NOSIGNAL);
	send(http_fd,css,sizeof(css),MSG_NOSIGNAL);
	send(http_fd,buffer,sizeof(buffer),MSG_NOSIGNAL);
}

void send_status(int http_fd)
{
	char tail[] = {"</BODY>"
			     "</HTML>"};

	char tmp[2048];
	int i;

	send(http_fd,http_header,sizeof(http_header),MSG_NOSIGNAL);
	send(http_fd,css,sizeof(css),MSG_NOSIGNAL);

	sprintf(tmp,
			    "</HEAD>"
			      "<BODY>"
			        "<A href=stat.html>libarpilot status:</A><P>");

	send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

	if(navdata_valid){
		float32_t heading = navdata_unpacked.navdata_demo.psi;
		        if (heading < 0)
		            heading += 360000;

		sprintf(tmp,"<TABLE BORDER=1 WIDTH=90%% CELLPADDING=4 CELLSPACING=3><TD VALIGN=top ALIGN=left WIDTH=300><H2>navdata</H2>"
		            "drone state: %d<BR>"
				    "drone batt: %d<BR>"
				    "drone altitude: %d<BR>"
				    "drone theta: %f<BR>"
				    "drone phi %f<BR>"
				    "drone psi %f<BR>"
				      ,navdata_unpacked.navdata_demo.ctrl_state
				      ,navdata_unpacked.navdata_demo.vbat_flying_percentage
				      ,navdata_unpacked.navdata_demo.altitude
				      ,navdata_unpacked.navdata_demo.theta
				      ,navdata_unpacked.navdata_demo.phi
				      ,heading);

		send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        sprintf(tmp,"<P><H2>drone flight</H2>"
                    "drone_init: %d<BR>"
                    "drone_fly: %d<BR>"
        		    "lockout_control: %d<BR>"
                    "drone_roll: %d<BR>"
                    "drone_pitch: %d<BR>"
                    "drone_gaz: %d<BR>"
                    "drone_yaw: %d<BR>"
                      ,drone_init
                      ,drone_fly
                      ,lockout_control
                      ,drone_roll
                      ,drone_pitch
                      ,drone_gaz
                      ,drone_yaw);

        send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        sprintf(tmp,"<P><H2>drone position</H2>"
                    "pos_lon: %f<BR>"
                    "pos_lat: %f<BR>"
             		"pos_alt: %d<BR>"
        		    "pos_course: %d<BR>"
        	    	"pos_hdop: %d<BR>"
                      ,pos_lon
                      ,pos_lat
                      ,pos_alt
                      ,pos_course
                      ,pos_hdop);

        send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        sprintf(tmp,"<P><H2>video</H2>"
                    "is_recording: %d<BR>"
                      ,is_recording);
        send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        sprintf(tmp,"<P><H2>automations</H2>"
                    "pilot_heading: %d<BR>"
                    "pilot_head: %d<BR>"
                    "pilot_altitude: %d<BR>"
                    "pilot_alti: %d<BR>"
                    "err_drone_altitude: %d<BR>"
                    "err_drone_phi: %d<BR></TD>"
                      ,pilot_heading
                      ,pilot_head
                      ,pilot_altitude
                      ,pilot_alti
                      ,err_drone_altitude
                      ,err_drone_phi);

		send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        sprintf(tmp,"<TD VALIGN=top ALIGN=left><H2>command log</H2><PRE>");
        send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

        for(i=0; i<50; i++){
		    char buffer[LOGLENGTH+1];

		    getlogmsg(i, buffer);

		    sprintf(tmp,"[%02d]: \"%s\"\r\n", i, buffer);
		    send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);
		}

		sprintf(tmp,"</PRE></TD></TABLE>");
        send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);
	}
	else {
		char buffer[] = {"There is no valid navdata.<BR>"};
		send(http_fd,buffer,sizeof(buffer),MSG_NOSIGNAL);
	}

	send(http_fd,tail,sizeof(tail),MSG_NOSIGNAL);
}

void send_emergency(int http_fd)
{
    char buffer[] = {"</HEAD>\n"
                      "<BODY><H1>Please select:</H1><P>"
                      "<H2><A href=cmd.html?cmd=land>--- Inject Land ---</A>"
                      "&lt --- &gt <A href=cmd.html?cmd=emer>--- Set Emergency ---</A></H2><BR>"
                      "</BODY></HTML>"};

    send(http_fd,http_header,sizeof(http_header),MSG_NOSIGNAL);
    send(http_fd,css,sizeof(css),MSG_NOSIGNAL);
    send(http_fd,buffer,sizeof(buffer),MSG_NOSIGNAL);
}


void send_video(int http_fd)
{
	char vidhead[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: video/H264\r\n\r\n"};
	char novid[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: text/html\r\n\r\n"
			         "<HTML>\n<HEAD>\n<TITLE>No more vid slots free</TITLE>\n</HEAD>\n"};

	int i;

	for ( i=0 ; i<5 ; i++) {
		if (!vid_sockets[i]) {
			vid_sockets[i] = http_fd;
			send(http_fd,vidhead,sizeof(vidhead),MSG_NOSIGNAL);

			return;
		}
	}

	send(http_fd,novid,sizeof(novid),MSG_NOSIGNAL);
	close(http_fd);
}

void send_vid_data(char *buffer,int len)
{
	int i;

	for ( i=0 ; i<5 ; i++) {
		if (vid_sockets[i]) {
			if (send(vid_sockets[i], buffer, len, MSG_NOSIGNAL) < 0) {
				close(vid_sockets[i]);
				vid_sockets[i] = 0;
			}
		}
	}
}

void send_log(int http_fd)
{
	char loghead[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: text/html\r\n\r\n"
			          "<HTML>\n<HEAD>\n<TITLE>libarpilot log</TITLE>\n</HEAD>\n"
			          "<BODY>libarpilot log output:<BR><PRE>\r\n"};
	char nolog[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: text/html\r\n\r\n"
			         "<HTML>\n<HEAD>\n<TITLE>No more log slots free</TITLE>\n</HEAD>\n"};

	int i;

	for ( i=0 ; i<5 ; i++) {
		if (!log_sockets[i]) {
		    int a;
			log_sockets[i] = http_fd;
			send(http_fd,loghead,sizeof(loghead),MSG_NOSIGNAL);

			/*send the last 99 log entries if not emty*/
			for (a=99;a>=0;a--) {
			    char buffer[41];
			    char tmp[80];

			    getlogmsg(a, buffer);

			    if (buffer[0]) {
                    sprintf(tmp,"%s\r\n", buffer);
                    send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);
			    }
			}

			return;
		}
	}

	send(http_fd,nolog,sizeof(nolog),MSG_NOSIGNAL);
	close(http_fd);
}

void send_log_data(char *buffer,int len)
{
	int i;

	for ( i=0 ; i<5 ; i++) {
		if (log_sockets[i]) {
			if (send(log_sockets[i],buffer,len,MSG_NOSIGNAL) < 0) {
				close(log_sockets[i]);
				log_sockets[i] = 0;
			}
			else
			{
				char tmp[] = {"\r\n"};
				if (send(log_sockets[i],tmp,sizeof(tmp),MSG_NOSIGNAL) < 0){
	                close(log_sockets[i]);
	                log_sockets[i] = 0;
				}
				//flush(log_sockets[i]);
			}
		}
	}
}

void send_command(int http_fd)
{
    send(http_fd,http_header,sizeof(http_header),MSG_NOSIGNAL);
	send(http_fd,css,sizeof(css),MSG_NOSIGNAL);
    send(http_fd,doc_command,sizeof(doc_command),MSG_NOSIGNAL);
}

void send_xml_status(int http_fd)
{
    char header[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: text/xml\r\n\r\n"};
    char tmp[2048];

    send(http_fd,header,sizeof(header),MSG_NOSIGNAL);

    float32_t heading = navdata_unpacked.navdata_demo.psi;
            if (heading < 0)
                heading += 360000;

    sprintf(tmp,"<drone>\n"
    		    "<status fly=\"%d\" "
                "batt=\"%d\" "
                "altitude=\"%d\" "
                "theta=\"%f\" "
                "phi=\"%f\" "
                "psi=\"%f\"/>\n"
                  ,drone_fly
                  ,navdata_unpacked.navdata_demo.vbat_flying_percentage
                  ,navdata_unpacked.navdata_demo.altitude
                  ,navdata_unpacked.navdata_demo.theta
                  ,navdata_unpacked.navdata_demo.phi
                  ,heading);

    send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);

    sprintf(tmp,"<position lon=\"%f\" "
                "lat=\"%f\" "
                "alt=\"%d\" "
                "course=\"%d\" "
                "hdop=\"%d\"/>\n"
    		    "</drone>\n"
                  ,pos_lon
                  ,pos_lat
                  ,pos_alt
                  ,pos_course
                  ,pos_hdop);

    send(http_fd,tmp,strlen(tmp),MSG_NOSIGNAL);
}

void send_cmd(int http_fd, char *buffer)
{
    char error[] = {"HTTP/1.0 200 OK\r\nServer: ardrone\r\nContent-Type: text/html\r\n\r\n"
                     "<HTML>\n<HEAD>\n<TITLE>libarpilot</TITLE>\n</HEAD>\n"
                      "<BODY><H1>Unknown command</H1><P>"
                      "<A href=command.html>back to commands</A><BR>"
                      "</BODY></HTML>"};

    if (!strncmp(buffer,"cmd=",4)) {
        buffer += 4;

        send_xml_status(http_fd);

        if (!strncmp(buffer,"fwd",3)) {
            command_move(0,-500,0,0);
            return;
        }
        if (!strncmp(buffer,"rev",3)) {
            command_move(0,500,0,0);
            return;
        }
        if (!strncmp(buffer,"left",4)) {
            command_move(-500,0,0,0);
            return;
        }
        if (!strncmp(buffer,"right",5)) {
            command_move(500,0,0,0);
            return;
        }
        if (!strncmp(buffer,"stop",4)) {
        	command_hover();
        	return;
        }
        if (!strncmp(buffer,"up",2)) {
            command_move(0,0,300,0);
            return;
        }
        if (!strncmp(buffer,"down",4)) {
            command_move(0,0,-300,0);
            return;
        }
        if (!strncmp(buffer,"rotl",4)) {
            command_move(0,0,0,-300);
            return;
        }
        if (!strncmp(buffer,"rotr",4)) {
            command_move(0,0,0,300);
            return;
        }
        if (!strncmp(buffer,"start",5)) {
        	command_state(1);
        	return;
        }
        if (!strncmp(buffer,"land",4)) {
        	command_state(0);
        	return;
        }
        if (!strncmp(buffer,"emer",4)) {
            command_state(666);
            return;
        }
        if (!strncmp(buffer,"rec",3)) {
        	if (!is_recording)
        		command_record(1);
        	else
        		command_record(0);
        	return;
        }
        if (!strncmp(buffer,"lock",4)) {
         	if (!lockout_control)
         		lockout_control=1;
         	else
         		lockout_control=0;
         	return;
         }

        return; /*Ignore unknown commands*/
    }

    // Error
    send(http_fd,error,sizeof(error),MSG_NOSIGNAL);
}

void send_404(int http_fd)
{
	char buffer[] = {"HTTP/1.0 404 ERR\r\nServer: ardrone\r\nContent-Type: text/html\r\n\r\n"
			         "<HTML>\n<HEAD>\n<TITLE>Error 404</TITLE>\n</HEAD>\n"};

	send(http_fd,buffer,sizeof(buffer),MSG_NOSIGNAL);
}

int init_web()
{
    if ((web_sock=socket(AF_INET, SOCK_STREAM, 0))==-1) {
        fprintf(stderr, "socket() failed\n");
        return -1;
    }

    memset((char *) &si_web, 0, sizeof(si_web));
    si_web.sin_family = AF_INET;
    si_web.sin_port = htons(HTTP_PORT);
    si_web.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(web_sock, (struct sockaddr *)&si_web, sizeof(si_web))==-1) {
        fprintf(stderr,"bind,web");
        return -1;
    }

    if ( listen(web_sock, LISTENQ) < 0 ) {
        fprintf(stderr,"listen() failed.");
    }

    web_state = 1;

    return 0;
}

int accept_web(int web_sock)
{
	int http_fd;

	if ( (http_fd = accept(web_sock, NULL, NULL)) < 0 ) {
        fprintf(stderr,"accept() failed.");
		return 0;
	}

	return http_fd;
}

int handle_web(int http_fd)
{
	char buffer[4096];
	int len = 0;

    bzero(buffer,4096);
    len = read(http_fd, buffer, 4096);

    if (strncmp(buffer,"\n\n",len) < 0)
    	goto end_close;

    if (strncmp(buffer,"GET ",4))
		goto end_close;

    /* Look for known documents */
    if (!strncmp(buffer,"GET /index.html",15)){
    	send_index(http_fd);
    	goto end_close;
    }

    if (!strncmp(buffer,"GET /stat.html",14)){
    	send_status(http_fd);
    	goto end_close;
    }

    if (!strncmp(buffer,"GET /video.html",15)){
    	send_video(http_fd);
    	goto end_leave;
    }

    if (!strncmp(buffer,"GET /command.html",17)){
        send_command(http_fd);
        goto end_close;
    }

    if (!strncmp(buffer,"GET /log.html",13)){
        send_log(http_fd);
        goto end_leave;
    }

    if (!strncmp(buffer,"GET /cmd.html?",14)){
    	printf("WEB-CMD\n");
        send_cmd(http_fd, buffer+14);
        goto end_close;
    }

    if (!strncmp(buffer,"GET /emer.html",14)){
        send_emergency(http_fd);
        goto end_close;
    }

    /* Send 404 error */
    send_404(http_fd);


end_close:
	close(http_fd);

end_leave:
	return 0;
}

