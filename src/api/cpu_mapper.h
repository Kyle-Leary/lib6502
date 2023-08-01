#pragma once

// handles the memory access between the Memmap structure and the cpu, making
// sure that the right addresses and mirrors are imposed. not doing anything
// clever for the mirroring right now.

#include "6502_defines.h"

// the actual memory mapping of all the components in the NES. the cpu requires
// access to a pointer to the memmap and the cpu's state.
typedef struct Memmap {
  // the first page (256 bytes) of the wram is the zero page!!
  u8 wram[KB(2)]; // 0x0000 - 0x07FF, mirrors at 0x0800, 0x1000,
                  // 0x1800

  u8 ppu_reg[8];     // 0x2000 - 0x2007, mirrors 0x2008 - 0x3FFF
  u8 apu_io_reg[32]; // 0x4000 - 0x401F

  // 0x4020 - 0x7FFF is cartridge-mapped etc space for the nes. it holds
  // expansion slots on the cart and sram, so basically anything bonus that the
  // cart might provide.

  // static ram, usually the save RAM on a cartridge. pretty large. so this
  // isn't really a part of the nes, it's an optional cartridge feature.
  u8 sram[KB(8)]; // 0x6000 - 0x7FFF

  // two banks of prg-rom could be loaded at a time.
  // these are pointers because we want to use the pointer from the rom
  // structure and avoid mallocing extra bytes.
  u8 *lower_prg; // 0x8000 - 0xBFFF, 16kb
  u8 *upper_prg; // 0xC000 - 0xFFFF, 16kb
} Memmap;

u8 cpuread(Memmap *m, u16 address); // read a byte of memory from the memmap.
void cpuwrite(Memmap *m, u16 address,
              u8 to_write); // write a byte of memory to the memmap.
