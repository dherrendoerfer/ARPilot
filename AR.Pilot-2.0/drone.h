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
 * Together with navdata.c this implements the protocol decoder of the AR.Drones
 * navdata protocol to the extend set by the publicly available documentation.
 *
 */


#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef DRONE_H_
#define DRONE_H_

#define INFO(_fmt_, args...)   \
printf(_fmt_, ##args)          \

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef float float32_t;
#define bool_t  int32_t

extern uint32_t mykonos_state;

/* Command constant */
#define BUFLEN 4096
#define NPACK 10
#define PORT 5556
#define MYPORT 5556
#define NAVPORT 5554
#define SRV_IP "192.168.1.1"

/* Navdata constant */
#define NAVDATA_SEQUENCE_DEFAULT  1
#define NAVDATA_PORT              5554
#define NAVDATA_HEADER            0x55667788
#define NAVDATA_BUFFER_SIZE       2048

/* Drone constants*/
#define DRONE_PCMD_FLAG_PROGRESSIVE        0x1
#define DRONE_PCMD_FLAG_COMBINED_YAW       0x2

#define DRONE_REF_FLAG_EMERGENCY         0x100
#define DRONE_REF_FLAG_START             0x200
#define DRONE_REF_FLAG_BASIC        0x11540000



enum {
  NO_CONTROL_MODE = 0,          // Doing nothing
  MYKONOS_UPDATE_CONTROL_MODE,  // Mykonos software update reception (update is done next run)
                                // After event completion, card should power off
  PIC_UPDATE_CONTROL_MODE,      // Mykonos pic software update reception (update is done next run)
                                // After event completion, card should power off
  LOGS_GET_CONTROL_MODE,        // Send previous run's logs
  CFG_GET_CONTROL_MODE,         // Send activ configuration
  ACK_CONTROL_MODE              // Reset command mask in navdata
};

enum {
  MYKONOS_FLY_MASK            = 1 << 0, /*!< FLY MASK : (0) mykonos is landed, (1) mykonos is flying */
  MYKONOS_VIDEO_MASK          = 1 << 1, /*!< VIDEO MASK : (0) video disable, (1) video enable */
  MYKONOS_VISION_MASK         = 1 << 2, /*!< VISION MASK : (0) vision disable, (1) vision enable */
  MYKONOS_CONTROL_MASK        = 1 << 3, /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
  MYKONOS_ALTITUDE_MASK       = 1 << 4, /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
  MYKONOS_USER_FEEDBACK_START = 1 << 5, /*!< USER feedback : Start button state */
  MYKONOS_COMMAND_MASK        = 1 << 6, /*!< Control command ACK : (0) None, (1) one received */
  MYKONOS_TRIM_COMMAND_MASK   = 1 << 7, /*!< Trim command ACK : (0) None, (1) one received */
  MYKONOS_TRIM_RUNNING_MASK   = 1 << 8, /*!< Trim running : (0) none, (1) running */
  MYKONOS_TRIM_RESULT_MASK    = 1 << 9, /*!< Trim result : (0) failed, (1) succeeded */
  MYKONOS_NAVDATA_DEMO_MASK   = 1 << 10, /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
  MYKONOS_NAVDATA_BOOTSTRAP   = 1 << 11, /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
  MYKONOS_MOTORS_BRUSHED      = 1 << 12, /*!< Motors brushed : (0) brushless, (1) brushed */
  MYKONOS_COM_LOST_MASK       = 1 << 13, /*!< Communication Lost : (1) com problem, (0) Com is ok */
  MYKONOS_GYROS_ZERO          = 1 << 14, /*!< Bit means that there's an hardware problem with gyrometers */
  MYKONOS_VBAT_LOW            = 1 << 15, /*!< VBat low : (1) too low, (0) Ok */
  MYKONOS_VBAT_HIGH           = 1 << 16, /*!< VBat high (US mad) : (1) too high, (0) Ok */
  MYKONOS_TIMER_ELAPSED       = 1 << 17, /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
  MYKONOS_NOT_ENOUGH_POWER    = 1 << 18, /*!< Power : (0) Ok, (1) not enough to fly */
  MYKONOS_ANGLES_OUT_OF_RANGE = 1 << 19, /*!< Angles : (0) Ok, (1) out of range */
  MYKONOS_WIND_MASK           = 1 << 20, /*!< Wind : (0) Ok, (1) too much to fly */
  MYKONOS_ULTRASOUND_MASK     = 1 << 21, /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
  MYKONOS_CUTOUT_MASK         = 1 << 22, /*!< Cutout system detection : (0) Not detected, (1) detected */
  MYKONOS_PIC_VERSION_MASK    = 1 << 23, /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
  MYKONOS_ATCODEC_THREAD_ON   = 1 << 24, /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
  MYKONOS_NAVDATA_THREAD_ON   = 1 << 25, /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
  MYKONOS_VIDEO_THREAD_ON     = 1 << 26, /*!< Video thread ON : (0) thread OFF (1) thread ON */
  MYKONOS_ACQ_THREAD_ON       = 1 << 27, /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
  MYKONOS_CTRL_WATCHDOG_MASK  = 1 << 28, /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
  MYKONOS_ADC_WATCHDOG_MASK   = 1 << 29, /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
  MYKONOS_COM_WATCHDOG_MASK   = 1 << 30, /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
  MYKONOS_EMERGENCY_MASK      = 1 << 31  /*!< Emergency landing : (0) no emergency, (1) emergency */
};

/******** Available codecs ********/
enum {
  NULL_CODEC    = 0,
  UVLC_CODEC    = 0x20,       // codec_type value is used for START_CODE
  MJPEG_CODEC,                // not used
  P263_CODEC,                 // not used
  P264_CODEC    = 0x40,
  MP4_360P_CODEC = 0x80,
  H264_360P_CODEC = 0x81,
  MP4_360P_H264_720P_CODEC = 0x82,
  H264_720P_CODEC = 0x83,
  MP4_360P_SLRS_CODEC = 0x84,
  H264_360P_SLRS_CODEC = 0x85,
  H264_720P_SLRS_CODEC = 0x86,
  H264_AUTO_RESIZE_CODEC = 0x87,    // resolution is automatically adjusted according to bitrate
  MP4_360P_H264_360P_CODEC = 0x88,
};


typedef enum _navdata_tag_t {
    NAVDATA_DEMO_TAG = 0,
    NAVDATA_VISION_DETECT_TAG = 16,
    NAVDATA_IPHONE_ANGLES_TAG = 18,
    NAVDATA_CKS_TAG = 0xFFFF
} navdata_tag_t;

typedef struct _matrix33_t
{
    float32_t m11;
    float32_t m12;
    float32_t m13;
    float32_t m21;
    float32_t m22;
    float32_t m23;
    float32_t m31;
    float32_t m32;
    float32_t m33;
} matrix33_t;

typedef struct _vector31_t {
    union {
        float32_t v[3];
        struct
        {
            float32_t x;
            float32_t y;
            float32_t z;
        };
    };
} vector31_t;

typedef struct _navdata_option_t {
    uint16_t  tag;
    uint16_t  size;

    uint8_t   data[];
} __attribute__ ((packed)) navdata_option_t;

typedef struct _navdata_t {
    uint32_t    header;
    uint32_t    mykonos_state;
    uint32_t    sequence;
    int      vision_defined;

    navdata_option_t  options[1];
} __attribute__ ((packed)) navdata_t;

typedef struct _navdata_cks_t {
    uint16_t  tag;
    uint16_t  size;

    // Checksum for all navdatas (including options)
    uint32_t  cks;
} __attribute__ ((packed)) navdata_cks_t;

typedef struct _navdata_demo_t {
  uint16_t    tag;                    /*!< Navdata block ('option') identifier */
  uint16_t    size;                   /*!< set this to the size of this structure */

  uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
  uint32_t    vbat_flying_percentage; /*!< battery voltage filtered (mV) */

  float32_t   theta;                  /*!< UAV's pitch in milli-degrees */
  float32_t   phi;                    /*!< UAV's roll  in milli-degrees */
  float32_t   psi;                    /*!< UAV's yaw   in milli-degrees */

  int32_t     altitude;               /*!< UAV's altitude in centimeters */

  float32_t   vx;                     /*!< UAV's estimated linear velocity */
  float32_t   vy;                     /*!< UAV's estimated linear velocity */
  float32_t   vz;                     /*!< UAV's estimated linear velocity */

  uint32_t    num_frames;             /*!< streamed frame index */
                                      // Not used -> To integrate in video stage.

  // Camera parameters compute by detection
  matrix33_t  detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
  vector31_t  detection_camera_trans; /*!<  Deprecated ! Don't use ! */
  uint32_t        detection_tag_index;    /*!<  Deprecated ! Don't use ! */
  uint32_t        detection_camera_type;  /*!<  Type of tag searched in detection */

  // Camera parameters compute by drone
  matrix33_t  drone_camera_rot;           /*!<  Deprecated ! Don't use ! */
  vector31_t  drone_camera_trans;         /*!<  Deprecated ! Don't use ! */
} __attribute__ ((packed)) navdata_demo_t;

typedef struct _navdata_iphone_angles_t {
    uint16_t   tag;
    uint16_t   size;

    int32_t    enable;
    float32_t  ax;
    float32_t  ay;
    float32_t  az;
    uint32_t   elapsed;
} __attribute__ ((packed)) navdata_iphone_angles_t;

typedef struct _navdata_time_t {
    uint16_t  tag;
    uint16_t  size;

    uint32_t  time;
} __attribute__ ((packed)) navdata_time_t;

#define NB_NAVDATA_DETECTION_RESULTS 4

typedef struct _navdata_vision_detect_t {
        /* !! Change the function 'navdata_server_reset_vision_detect()' if this structure is modified !! */
  uint16_t   tag;
  uint16_t   size;

  uint32_t   nb_detected;
  uint32_t   type[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   xc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   yc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   width[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   height[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   dist[NB_NAVDATA_DETECTION_RESULTS];
  float32_t  orientation_angle[NB_NAVDATA_DETECTION_RESULTS];
  matrix33_t rotation[NB_NAVDATA_DETECTION_RESULTS];
  vector31_t translation[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   camera_source[NB_NAVDATA_DETECTION_RESULTS];
} __attribute__ ((packed)) navdata_vision_detect_t;

typedef struct _navdata_unpacked_t {
    uint32_t  mykonos_state;
    int    vision_defined;

    navdata_demo_t           navdata_demo;
    navdata_iphone_angles_t  navdata_iphone_angles;
    navdata_vision_detect_t  navdata_vision_detect;
} navdata_unpacked_t;

static inline int get_mask_from_state( uint32_t state, uint32_t mask )
{
    return state & mask ? TRUE : FALSE;
}

static inline uint8_t* navdata_unpack_option( uint8_t* navdata_ptr, uint8_t* data, uint32_t size )
{
    memcpy(data, navdata_ptr, size);

//    printf("memcpy: %i bytes\n",size);

    return (navdata_ptr + size);
}

static inline navdata_option_t* navdata_next_option( navdata_option_t* navdata_options_ptr )
{
    uint8_t* ptr;

    ptr  = (uint8_t*) navdata_options_ptr;
    ptr += navdata_options_ptr->size;

    return (navdata_option_t*) ptr;
}

navdata_option_t* navdata_search_option( navdata_option_t* navdata_options_ptr, uint32_t tag );

static inline uint32_t navdata_compute_cks( uint8_t* nv, int32_t size )
{
    int32_t i;
    uint32_t cks;
    uint32_t temp;

    cks = 0;

    for( i = 0; i < size; i++ ) {
            temp = nv[i];
            cks += temp;
        }

    return cks;
}

#define navdata_unpack( navdata_ptr, option ) (navdata_option_t*) navdata_unpack_option( (uint8_t*) \
                        navdata_ptr, (uint8_t*) &option, navdata_ptr->size )



#endif
