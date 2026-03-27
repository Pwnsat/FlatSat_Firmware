/*  - rdownlink.h
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef FIRMWARE_RDOWNLINK_H
#define FIRMWARE_RDOWNLINK_H
#include <Arduino.h>

#define DOWNLINK_FREQ (912)
#define DOWNLINK_BW (250)
#define DOWNLINK_SF (7)
#define DOWNLINK_CR (5)

typedef void (*radioPacketReceivedCb)(uint8_t *buffer, uint16_t buffer_len);

void downlinkRadioConfigure(void);
void downlinkRadioRegisterCb(radioPacketReceivedCb recv_cb);
void downlinkRadioCheckPacketReceived(void);
#endif // FIRMWARE_RDOWNLINK_H