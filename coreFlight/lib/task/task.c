/* lib/task - task.c
 * DESCRIPTION
 *
 * coreFlight - LoRa Packet Analyzer
 * By astrobyte 13/09/25.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "task.h"
#include <stdio.h>

static task_t task_list[10];
static uint16_t task_count = 0;

void task_show_list(void) {
  for (int i = 0; i < task_count; i++) {
    printf("%d\t|%s\t|%s\t|%d\t|%d\t|%d\t|", i, task_list[i].name,
           task_list[i].description, task_list[i].time_s, task_list[i].priority,
           task_list[i].state);
  }
}

void task_create(const char *name, const char *description, uint16_t time_s,
                 task_priority_t priority) {
  task_list[task_count++] =
      (task_t){time_s, priority, name, description, STOPPED};
}