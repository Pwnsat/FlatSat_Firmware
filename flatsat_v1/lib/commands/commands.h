#ifndef __COMMANDS_H
#define __COMMANDS_H
#include <commands_list.h>
#include <apid.h>
#include <stdint.h>

void commandHandler(uint8_t* buffer, uint16_t buffer_len);
#endif