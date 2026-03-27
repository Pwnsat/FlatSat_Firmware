/*  - usbCDC.h
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef FIRMWARE_USBCDC_H
#define FIRMWARE_USBCDC_H
#include "frame.h"

void obcSetupUSB(void);
void obcConfigureCore0(void);
void obcConfigureCore1(void);
void obcWriteFrame(spacecan_frame_t *f);
#endif // FIRMWARE_USBCDC_H
