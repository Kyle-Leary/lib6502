#pragma once

#include "6502_defines.h"
#include "cpu_mapper.h"
#include <stdio.h>
#include <sys/types.h>

typedef union { // args can either be some u16 entity,
  // or a u8 entity. pass this to get around the weirdness.
  u8 immediate;
  u16 address;
} Args;

// enums and defines
typedef enum AddrMode { // the addressing mode for each instruction.

  Implicit, // no args
  Immediate,
  Abs,
  AbsX,
  AbsY,
  ZP,
  ZPX,
  ZPY,
  Relative,
  Indirect,
  IndexedIndirect,
  IndirectIndexed,
  ADDRMODE_COUNT,

} AddrMode;

typedef enum StatusBit {
  Carry = (1 << 0),
  Zero = (1 << 1),
  Interrupt = (1 << 2),
  Decimal = (1 << 3),
  Break = (1 << 4),
  Unused = (1 << 5), // "unused, often set to 1."
  Overflow = (1 << 6),
  Negative =
      (1 << 7), // (1 << 7) == 0b1000 0000, it's just flipping the nth bit.
} StatusBit;

typedef struct CPUState {
  u16 pc;
  u8 sp; // is the sp 8 bit?

  u8 a;
  u8 x;
  u8 y;
  u8 status;

  u8 shutting_down; // for BRK.
} CPUState;

// this is substate and the emu.h file has to access it. therefore, we need to
// expose constructors here.
CPUState *make_cpu_state();
void clean_cpu_state(CPUState *state);

void cpu_tick(CPUState *cs, Memmap *map);
void cpu_single_run(
    CPUState *cs, Memmap *map, u8 *instruction,
    uint instruction_size); // execute a single instruction. a helper for the
                            // greater EmuState function.
void cpu_debug_print(CPUState *cs);
