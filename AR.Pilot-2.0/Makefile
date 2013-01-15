#*
#* This file is part of libarpilot.
#*
#* Copyright (C) 2012  D.Herrendoerfer
#*
#*   libarpilot is free software: you can redistribute it and/or modify
#*   it under the terms of the GNU Lesser Public License as published by
#*   the Free Software Foundation, either version 3 of the License, or
#*   (at your option) any later version.
#*
#*   libarpilot is distributed in the hope that it will be useful,
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#*   GNU Lesser Public License for more details.
#*
#*   You should have received a copy of the GNU Lesser Public License
#*   along with libarpilot.  If not, see <http://www.gnu.org/licenses/>.
#*
 
CFLAGS = -O2 -Wall
CC = gcc
INSTALL = install
TARGET = /usr/sbin

LIB_OBJS=command.o controller.o emitter.o init.o log.o monitor.o navdata.o network.o pilot.o web.o
PROG_OBJS=arpilot.o
LIB=arpilot.a
PROG=arpilot

all: lib arpilot 

.c.o:
	$(CC) -c $(CFLAGS) -o $@ $<

lib: $(LIB_OBJS)
	ar r $(LIB) $(LIB_OBJS)

arpilot: $(PROG_OBJS) lib
	$(CC) $(CFLAGS) $(PROG_OBJS) $(LIB) -o $(PROG)

install: $(PROG)
	$(INSTALL) $(PROG) $(TARGET)/$(PROG)

clean:
	rm -f $(PROG_OBJS) $(LIB_OBJS) $(PROG) $(LIB)
