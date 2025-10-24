/* lib/filesystem - filesystem.c
 * DESCRIPTION
 *
 * coreFlight - LoRa Packet Analyzer
 * By astrobyte 13/09/25.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "filesystem.h"

#include "dirent.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>

void filesystem_list_files(const char *path, char *buffer) {
  DIR *dir = opendir(path);
  if (dir == NULL) {
    printf("[SYS-FILESYSTEM] Error opening dir: %s", path);
    return;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    sprintf(buffer, "%s%s;", buffer, entry->d_name);
  }
  closedir(dir);
}