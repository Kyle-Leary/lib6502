#include "api/6502_defines.h"

// for functions that return strings that must be freed.
// put the expression that returns a string in this macro, and it will be pulled
// out and freed automatically.
// think of this conceptually as a "do" block in ruby, or something similar.
// we'll "do" the second bit with the context of the expr string.
//
// will expose the result of "expr" to the passed codeblock in the second arg.
// use the value in to_free by name. literally call it "to_free".
#define BASE_USE_THEN_FREE(expr, block, type)                                  \
  {                                                                            \
    type to_free = (expr);                                                     \
    if (to_free != NULL) {                                                     \
      block;                                                                   \
      free(to_free);                                                           \
    } else {                                                                   \
      fprintf(stderr,                                                          \
              "Failed to get the pointer in the BASE_USE_THEN_FREE macro.");   \
    }                                                                          \
  }

#define STR_USE_THEN_FREE(expr, block) BASE_USE_THEN_FREE(expr, block, char *)

#define CONST_STR_USE_THEN_FREE(expr, block)                                   \
  BASE_USE_THEN_FREE(expr, block, const char *)

u16 convertToLittleEndian16(u16 value);
u32 convertToLittleEndian(u32 value);
void mapArraySection(u8 *source, u8 *destination, int sourceStart,
                     int destinationStart, int length);
u32 djb2_hash(const char *str, u32 array_size);
char *read_file_as_string(const char *filename);
