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
#include "arpilot.h"


struct sockaddr_in si_other,si_me, si_nav, si_vid;
int command_sock;
int slen=sizeof(struct sockaddr_in);
struct ip_mreq nav_group;
int nav_sock;
int vid_sock;

int neterror = 0;

int  vid_state = 0;
int  nav_state = 0;

int add_multicast_members()
{
    int s;
    struct ifconf ifconf;
    struct ifreq ifr[50];
    int domain = AF_INET;
    int ifs;
    int i;

    s = socket(domain, SOCK_STREAM, 0);
    if (s < 0) {
        perror("socket");
        return -1;
    }

    ifconf.ifc_buf = (char *) ifr;
    ifconf.ifc_len = sizeof ifr;

    if (ioctl(s, SIOCGIFCONF, &ifconf) == -1) {
        perror("ioctl");
        return -1;
    }

    ifs = ifconf.ifc_len / sizeof(ifr[0]);

    for (i = 0; i < ifs; i++) {
        char ip[INET_ADDRSTRLEN];
        struct sockaddr_in *s_in = (struct sockaddr_in *) &ifr[i].ifr_addr;

        if (!inet_ntop(domain, &s_in->sin_addr, ip, sizeof(ip))) {
            perror("inet_ntop");
            return -1;
        }

        nav_group.imr_multiaddr.s_addr = inet_addr("224.1.1.1");
        nav_group.imr_interface.s_addr = inet_addr(ip);

        if(setsockopt(nav_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
           (char *)&nav_group, sizeof(nav_group)) < 0) {
            perror("Adding multicast group error");
            return -1;
        }
    }

    close(s);

    return 0;
}

int init_navdata()
{
    //
    //Note: Sending anthing other than the buffer
    //      below makes the drone respond with
    //      multicast navdata. (commented variant)
    //
    struct sockaddr_in si_tmp;
    char buffer[] =  {1,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int len=14;

//  sprintf(buffer,"%d",(int)1);
//  int len=strlen(buffer);

    memset((char *) &si_tmp, 0, sizeof(si_tmp));
    si_tmp.sin_family = AF_INET;
    si_tmp.sin_port = htons(NAVPORT);
    inet_aton(SRV_IP, &si_tmp.sin_addr);

//  printf("xmit: \"%s\", len=%i\n",buffer,len);
    if (sendto(nav_sock, buffer, len, 0, (struct sockaddr *)&si_tmp, slen)==-1) {
		fprintf(stderr,"navdata sendto()\n");
		return -1;
    }

    nav_state = 1;
    return 0;
}

int init_vid()
{
    vid_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (vid_sock < 0) {
        fprintf(stderr,"video socket()\n");
        return -1;
    }

    memset((char *) &si_vid, 0, sizeof(si_vid));
    si_vid.sin_family = AF_INET;
    si_vid.sin_port = htons(5555);
    inet_aton(SRV_IP, &si_vid.sin_addr);

    if (connect(vid_sock,(struct sockaddr *) &si_vid,sizeof(si_vid)) < 0) {
        fprintf(stderr,"video connect()\n");
                return -1;
    }

    vid_state = 1;
    return 0;
}

int reconnect_vid()
{
	close(vid_sock);

	return(init_vid());
}

int setup_net()
{
    // Prepare the UDP socket to send the commands
    if ((command_sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        fprintf(stderr, "socket() failed\n");
        return -1;
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        return -1;
    }

    memset((char *) &si_me, 0, sizeof(si_other));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(MYPORT+100);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(command_sock, (struct sockaddr *)&si_me, sizeof(si_me))==-1) {
        fprintf(stderr,"bind,my");
        return -1;
    }

    if ((nav_sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
		fprintf(stderr, "socket() failed\n");
		return -1;
    }

    memset((char *) &si_nav, 0, sizeof(si_nav));
    si_nav.sin_family = AF_INET;
    si_nav.sin_port = htons(NAVPORT+100);
    si_nav.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(nav_sock, (struct sockaddr *)&si_nav, sizeof(si_nav))==-1) {
        fprintf(stderr,"bind,nav");
    	return -1;
    }

    // Add interfaces to multicast groups
    // only needed when doing multicast
//  if(add_multicast_members() == -1)
//      exit(1);

    return 0;
}

void dump_buffer(char *buffer)
{
	int i;
	for (i=0;i<strlen(buffer);i++)
		if(buffer[i] == '\r')
			buffer[i] = ';';

	printf("xmit: \"%s\", len=%i\n",buffer,(int)strlen(buffer));
}

void udp_send_command(char* buffer)
{
    int len=strlen(buffer);
    if (sendto(command_sock, buffer, len, 0, (struct sockaddr *)&si_other, slen)==-1)
        neterror++;
    else
        neterror = 0;

    if (neterror == 25)
        printf("!WARN,network\n");

    //_buffer(buffer);
}

int shutdown_net()
{
    close(nav_sock);
    close(command_sock);

    return 0;
}
