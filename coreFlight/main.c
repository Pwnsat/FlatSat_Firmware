#include "lib/filesystem/filesystem.h"
#include "lib/task/task.h"
#include <stdio.h>

int main(void) {
  char buffer[512] = {0};
  filesystem_list_files("/", buffer);
  printf("%s\n", buffer);
  task_create("STATUS", "Send the system status", 1, 0);
  task_show_list();
  return 0;
}