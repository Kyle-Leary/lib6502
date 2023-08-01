#pragma once

#include "6502_defines.h"
#include "cpu.h"
#include <sys/types.h>

// module that holds the components together, and connects them,
// the components being the cpu, ppu and apu of the NES.

// size of the ines header
#define HEADER_SIZE 0xF

// size of each of the types of ROM banks on an nes cartridge.
#define PRG_BANK_SIZE KB(16)
#define CHR_BANK_SIZE KB(8)

typedef struct ROMState {
  u8 *prg_banks[PRG_BANK_SIZE]; // array of banks in the cartridge itself.

  u8 *chr_banks[CHR_BANK_SIZE]; // easy bank switching, just change around the
                                // pointer to the active prg or chr in memory.
                                // the chr uses the ppu address space, not the
                                // cpu. they're different chips that occupy
                                // completely different worlds.

  u8 num_prg_banks; // from the ines header, a u8
  u8 num_chr_banks;

  u32 rom_prg_offset; // the offsets of each section in the ROM file. parsing
                      // helper variables, really, but they might be useful
                      // later.
  u32 rom_chr_offset;

  u32 rom_prg_size; // the overall size of the prg and chr rom segments in the
                    // ines rom file.
  u32 rom_chr_size;

  u8 flags; // byte 6 of the header
} ROMState;

// our overall stateful object for the emulator core.
// cleaning this should clean EVERYTHING else.
typedef struct EmuState {
  CPUState *cpu_state;

  ROMState
      *rom; // actual ROM data from the file itself. nothing directly physical.

  Memmap *map;
} EmuState;

// management of the client's external emulator state.
EmuState *emu_init();
void emu_update(EmuState *emu_state, u8 *is_running);
void emu_clean(EmuState *emu_state);

// then, the public execution types on the EmuState.
void execute_instruction(
    EmuState *emu_state, u8 *instruction,
    uint instruction_size); // literally take in a single opcode and just go
                            // through that, without any overhead of a file or
                            // full ROM.
void emu_load(
    EmuState *emu_state,
    const char *rom_file); // run a proper NES rom, directly from a filepath.
void emu_prg_load(
    EmuState *emu_state,
    u8 *prg_rom_array); // load a test executable for the cpu into the first
                        // bank of the prg-rom, and just start executing.
