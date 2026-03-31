/*  - mission.h
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef FIRMWARE_MISSION_H
#define FIRMWARE_MISSION_H
#include <Arduino.h>

#define FIRMWARE_PATCH 0
#define FIRMWARE_MINOR 0
#define FIRMWARE_MAJOR 0

#define MAX_PAYLOAD_CHUNK 128
#define SPACECRAFT_ID 0x01

#define SC_TM_ID_ACCE_X 0x01
#define SC_TM_ID_ACCE_Y 0x02
#define SC_TM_ID_ACCE_Z 0x03
#define SC_TM_ID_ACCE_TMP 0x04

#define SC_TM_ID_BME_TEMPERATURE 0x05
#define SC_TM_ID_BME_PRESSURE 0x06
#define SC_TM_ID_BME_ALTITUDE 0x07
#define SC_TM_ID_BME_HUMIDITY 0x08

#define SC_TM_ID_SIM_THRUSTER0 0x09
#define SC_TM_ID_SIM_THRUSTER1 0x0A

/* APIDS TC */
// On Board Computer
#define SPP_APID_TC_PING 0x01
#define SPP_APID_TC_RESETC 0x02
#define SPP_APID_TC_SEND_FW 0x03
// EPS
#define SPP_APID_TC_SET_THRUSTER 0x04
// Communications
#define SPP_APID_TC_SET_BEACON_RATE 0x05
#define SPP_APID_TC_BROADCAST_MSG 0x06
/* APIDS TM */
// On Board Computer
#define SPP_APID_TM_PING 0x01
#define SPP_APID_TM_RESETC 0x02
#define SPP_APID_TM_SEND_FW 0x03
// EPS
#define SPP_APID_TM_SET_THRUSTER 0x04
// Communications
#define SPP_APID_TM_SET_BEACON_RATE 0x05
#define SPP_APID_TM_BROADCAST_MSG 0x06
#define SPP_APID_TM_SEND_TM 0x08

#endif // FIRMWARE_MISSION_H