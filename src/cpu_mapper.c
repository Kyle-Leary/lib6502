#include "cpu_mapper.h"
#include <stdio.h>

#define START_WRAM 0x0000
#define END_WRAM 0x07FF
#define WRAM_SIZE END_WRAM - START_WRAM
#define END_WRAM_MIRROR 0x1800
#define WRAM_MIRROR_SIZE END_WRAM_MIRROR - START_WRAM

#define START_PPU_REG 0x2000
#define END_PPU_REG 0x2007
#define PPU_REG_SIZE END_PPU_REG - START_PPU_REG
#define END_PPU_REG_MIRROR 0x3FFF
#define PPU_REG_MIRROR_SIZE END_PPU_REG_MIRROR - START_PPU_REG

#define START_APU_REG 0x4000
#define END_APU_REG 0x401F

// sram and lower_prg (and the upper prg) lead into eachother, they're
// contiguous.
#define START_SRAM 0x6000
#define END_SRAM 0x7FFF
#define START_LOWER_PRG 0x8000
#define END_LOWER_PRG 0xBFFF
#define START_UPPER_PRG 0xC000
#define END_UPPER_PRG 0xFFFF

// i would've prefered some C magic pointer shenanigans for the memory mapper in
// the 6502, but this is ok too.
u8 cpuread(Memmap *m, u16 address) {
  if (address <= END_WRAM_MIRROR) {
    return m->wram[address % WRAM_SIZE]; // get the mirrored address wrt to the
                                         // REAL size of the segment, up until
                                         // when the mirror of wram stops.
  } else if (address <= END_PPU_REG_MIRROR) {
    return m->ppu_reg[address % PPU_REG_SIZE];
  } else if (address <= END_APU_REG) {
    return m->apu_io_reg[address];
  } else if (address <= END_SRAM) {
    return m->sram[address];
  } else if (address <= END_LOWER_PRG) {
    return m->lower_prg[address];
  } else if (address <= END_UPPER_PRG) {
    return m->upper_prg[address];
  } else {
    fprintf(stderr,
            "Invalid address in the memory mapper, the CPU tried to access "
            "0x%04X.\n",
            address);

    return 0; // return a dummy. whatever.
  }
}

// TODO: find a better (but still fast) way to reuse the mapper code, maybe
// macro-ify it?
void cpuwrite(Memmap *m, u16 address, u8 to_write) {
  if (address <= END_WRAM_MIRROR) {
    m->wram[address % WRAM_SIZE] =
        to_write; // get the mirrored address wrt to the
                  // REAL size of the segment, up until
                  // when the mirror of wram stops.
  } else if (address <= END_PPU_REG_MIRROR) {
    m->ppu_reg[address % PPU_REG_SIZE] = to_write;
  } else if (address <= END_APU_REG) {
    m->apu_io_reg[address] = to_write;
  } else if (address <= END_SRAM) {
    m->sram[address] = to_write;
  } else if (address <= END_LOWER_PRG) {
    m->lower_prg[address] = to_write;
  } else if (address <= END_UPPER_PRG) {
    m->upper_prg[address] = to_write;
  } else {
    fprintf(stderr,
            "Invalid address in the memory mapper, the CPU tried to write the "
            "byte 0x%02X to address 0x%04X.\n",
            to_write, address);
  }
}
