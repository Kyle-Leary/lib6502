#include "cpu_mapper.h"
#include <stdio.h>

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
