/*  - usbCDC.cpp
 *
 * firmware - By astrobyte 18/03/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include "usbCDC.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#define BOARD_VERSION 0x0020 // Current version board 03/2026

Adafruit_USBD_CDC USBSpaceCan;

void obcSetupUSB(void) {
  TinyUSBDevice.setID(0x239a, 0xcafe);
  TinyUSBDevice.setManufacturerDescriptor("Pwnsat");
  TinyUSBDevice.setProductDescriptor("Pwnsat - Flatsat");
  TinyUSBDevice.setSerialDescriptor("fsat");
  TinyUSBDevice.setDeviceVersion(BOARD_VERSION);
}

static void obcResetUSBStack(void) {
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

static void obcWaitConnectionCore0(void) {
  while (!Serial) {
    delay(1000);
  }
}

static void obcWaitConnectionCore1(void) {
  while (!USBSpaceCan) {
    delay(1000);
  }
}

void obcConfigureCore0(void) {
  obcSetupUSB();

  Serial.begin(921600);
  obcWaitConnectionCore0();

  if (CFG_TUD_CDC < 2) {
    Serial.printf(
        "[WARNING] CFG_TUD_CDC must be at least 2, current value is %u\r\n",
        CFG_TUD_CDC);
    return;
  }
  Serial.printf("[INFO] USB Device configured successfully\r\n");
}

void obcConfigureCore1(void) {
  obcSetupUSB();
  USBSpaceCan.begin(921600);
  obcResetUSBStack();
  obcWaitConnectionCore1();
  USBSpaceCan.print("[INFO] SPACECAN Bus Interface\r\n");
}

void obcWriteFrame(spacecan_frame_t *f) {
  uint8_t sync = 0xAA;

  USBSpaceCan.write(sync);

  USBSpaceCan.write((uint8_t)(f->can_id & 0xFF));
  USBSpaceCan.write((uint8_t)((f->can_id >> 8) & 0xFF));

  USBSpaceCan.write(f->dlc);
  USBSpaceCan.write(f->buffer, f->dlc);
}