#include "util.h"

#include <stdio.h>
#include <stdlib.h>

u16 convertToLittleEndian16(u16 value) {
  u16 result = ((value & 0xFF) << 8) | ((value >> 8) & 0x00FF);
  return result;
}

// helper functions
u32 convertToLittleEndian(u32 value) {
  u32 result = ((value & 0xFF) << 24) | ((value & 0xFF00) << 8) |
               ((value >> 8) & 0xFF00) | ((value >> 24) & 0xFF);
  return result;
}

void mapArraySection(u8 *source, u8 *destination, int sourceStart,
                     int destinationStart, int length) {
  int i;
  for (i = 0; i < length; i++) {
    destination[destinationStart + i] = source[sourceStart + i];
  }
}

u32 djb2_hash(
    const char *str,
    u32 array_size) { // for stuff like the texture mappings from string to
                      // index in the video_state texture array.
  u32 hash = 5381;    // A recommended initial value

  while (*str != '\0') {
    hash = ((hash << 5) + hash) + *str; // DJB2 algorithm
    str++;
  }

  return hash % array_size;
}

char *read_file_as_string(
    const char *filename) { // returns a malloced string, trust the caller to
                            // free it when they're done.
  FILE *file = fopen(filename, "rb"); // Open the file in binary mode
  if (file == NULL) {
    fprintf(stderr, "Failed to open file: %s\n", filename);
    return NULL;
  }

  // Seek to the end of the file to determine its size
  fseek(file, 0, SEEK_END);
  long file_size = ftell(file);

  // Allocate memory for the string, leave one byte of room for the NUL
  char *string = (char *)malloc((file_size + 1) * sizeof(char));
  if (string == NULL) {
    fprintf(stderr, "Memory allocation failed.\n");
    fclose(file);
    return NULL;
  }

  // Seek back to the beginning of the file
  fseek(file, 0, SEEK_SET);

  // Read the file contents into the allocated memory
  size_t bytes_read = fread(string, sizeof(char), file_size, file);
  if (bytes_read != file_size) {
    fprintf(stderr, "Failed to read file: %s\n", filename);
    free(string);
    fclose(file);
    return NULL;
  }

  // Append null terminator at the end of the string
  string[file_size] = '\0';

  fclose(file);

  return string;
}
