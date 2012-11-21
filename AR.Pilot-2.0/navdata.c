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
 *
 * Note:
 * The origin of the contents of this file are multiple.
 * Most of the Content was pulled from the AR.Drone programming documentation
 * available on Parrot's home page and several programming examples available
 * on the internet.
 * Together with drone.h this implements the protocol decoder of the AR.Drones
 * navdata protocol to the extend set by the publicly available documentation.
 *
 */


#include "drone.h"
#include "states.h"

unsigned int nav_sequence = NAVDATA_SEQUENCE_DEFAULT-1;
int navdata_valid = 0;
uint8_t navdata_buffer[NAVDATA_BUFFER_SIZE];

navdata_unpacked_t navdata_unpacked;

static void mykonos_navdata_unpack_all(navdata_unpacked_t* navdata_unpacked, navdata_t* navdata, uint32_t* cks)
{
    navdata_cks_t navdata_cks = { 0 };
    navdata_option_t* navdata_option_ptr;

    navdata_option_ptr = (navdata_option_t*) &navdata->options[0];

    memset( navdata_unpacked, 0, sizeof(*navdata_unpacked) );

    navdata_unpacked->mykonos_state   = navdata->mykonos_state;
    navdata_unpacked->vision_defined  = navdata->vision_defined;

    while( navdata_option_ptr != NULL )
        {
            // Check if we have a valid option
            if( navdata_option_ptr->size == 0 )
                {
                    INFO ("One option is not a valid because its size is zero\n");
                    navdata_option_ptr = NULL;
                }
            else
                {
                    switch( navdata_option_ptr->tag )
                        {
                        case NAVDATA_DEMO_TAG:
//                            INFO ("Demo tag %i bytes\n",sizeof(navdata_demo_t));
                            navdata_option_ptr = navdata_unpack( navdata_option_ptr, navdata_unpacked->navdata_demo );
                            break;

                        case NAVDATA_IPHONE_ANGLES_TAG:
//                            INFO ("Angles tag %i bytes\n",sizeof(navdata_iphone_angles_t));
                            navdata_option_ptr = navdata_unpack( navdata_option_ptr, navdata_unpacked->navdata_iphone_angles );
                            break;

                        case NAVDATA_VISION_DETECT_TAG:
//                            INFO ("Vision tag %i bytes\n",sizeof(navdata_vision_detect_t));
                            navdata_option_ptr = navdata_unpack( navdata_option_ptr, navdata_unpacked->navdata_vision_detect );
                            break;

                        case NAVDATA_CKS_TAG:
//                            INFO ("CKS tag\n");
                            navdata_option_ptr = navdata_unpack( navdata_option_ptr, navdata_cks );
                            *cks = navdata_cks.cks;
                            navdata_option_ptr = NULL; // End of structure
                            break;
                        default:
                            INFO ("Tag %d is not a valid navdata option tag\n", (int) navdata_option_ptr->tag);
                            navdata_option_ptr = NULL;
                            break;
                        }
                }
        }
}


int decode_navdata(uint8_t *navbuff,int size)
{
    unsigned int cks = 0;
    unsigned int navdata_cks = 0;
    uint32_t mykonos_state;

    navdata_t* navdata = (navdata_t*) &navbuff[0];

//  printf("recv: %i\n",size);
//  printf("seq : %i\n",navdata->sequence);

    if( navdata->header == NAVDATA_HEADER ) {
        mykonos_state = navdata->mykonos_state;

//      printf("Stat: %ld\n",mykonos_state);

        if( get_mask_from_state(navdata->mykonos_state, MYKONOS_COM_WATCHDOG_MASK) ) {
            INFO ("[NAVDATA] Detect com watchdog\n");
            nav_sequence = NAVDATA_SEQUENCE_DEFAULT-1;

            if( get_mask_from_state(navdata->mykonos_state, MYKONOS_NAVDATA_BOOTSTRAP) == FALSE ) {
                INFO ("[NAVDATA] send watchdog\n");
            }
        }

        if( navdata->sequence > nav_sequence ) {
            if ( get_mask_from_state( mykonos_state, MYKONOS_NAVDATA_DEMO_MASK )) {
                mykonos_navdata_unpack_all(&navdata_unpacked, navdata, &navdata_cks);
                cks = navdata_compute_cks( &navbuff[0], size - sizeof(navdata_cks_t) );
                if( cks == navdata_cks ) {
                    navdata_valid=TRUE;
                }
                else {
                    navdata_valid=FALSE;
                }
            }
            nav_sequence = navdata->sequence;
        }
        else {
            INFO ("[Navdata] Sequence pb : %d (distant) / %d (local)\n", navdata->sequence, nav_sequence);
        }

    }

    return 0;
}
