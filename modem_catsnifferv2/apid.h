/**
 * @file apid.h
 * @brief Space Packet Protocol Mission APIDs
 *
 * @author Pwnsat
 * @date 2025-08-7
 */
#ifndef __APID_H
#define __APID_H

/* APID sended from spacecraft */
#define APID_TM_SENSORS 100
#define APID_TM_BME 101
#define APID_TM_MPU 102
// This is the spacecraft sender ack ping

#define APID_TC_SEND_CONFIG 320

/* APID recv from spacecraft */
#define APID_TC_GET_CONFIG 300
#define APID_TC_SET_FREQ 301
// APIDS with response
#define APID_TC_PING_SYNC 200
#define APID_TC_GET_STATUS 400
#define APID_TC_GET_TEMP 401
#define APID_TC_GET_GYRO 402
#define APID_TC_GET_TM 403
// APID TC Recv file
#define APID_TC_FIRMWARE_UPDATE 66
#define APID_TM_IMAGE 10
// APID FOR ROOT stuff
#define APID_TC_SET_ROOT 900

// APID TM for command response
#define APID_TM_PING_ACK 201    // Response <- APID_TC_PING_SYNC
#define APID_TM_SEND_STATUS 450 // Response <- APID_TC_GET_STATUS
#define APID_TM_SEND_TEMP 451   // Response <- APID_TC_GET_TEMP
#define APID_TM_SEND_GYRO 452   // Response <- APID_TC_GET_GYRO
#define APID_TM_SEND_TM 453     // Response <- APID_TC_GET_TM
#define APID_TM_FILE_ACK 453    // Response <- APID_TC_FIRMWARE_UPDATE

#endif