/* lib/task - task.h
 * DESCRIPTION
 *
 * coreFlight - LoRa Packet Analyzer
 * By astrobyte 13/09/25.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef COREFLIGHT_TASK_H
#define COREFLIGHT_TASK_H
#include <stdint.h>

#define TASK_LIMIT

typedef enum { STOPPED, RUNNING } task_state_t;
typedef enum { ROOT = 0, SYSTEM, USER } task_priority_t;

typedef struct {
  uint16_t time_s;
  task_priority_t priority;
  const char *name;
  const char *description;
  task_state_t state;
} task_t;
void task_show_list(void);
void task_create(const char *name, const char *description, uint16_t time_s,
                 task_priority_t priority);

#endif // COREFLIGHT_TASK_H