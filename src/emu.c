#include "emu.h"
#include "cpu.h"
#include "stdlib.h"
#include "util.h"

#include <stdio.h>

#define ines_magic 0x4E45531A

static ROMState *make_rom_state() {
  // all data is zero initially, no rom banks have been loaded.
  ROMState *r = (ROMState *)calloc(sizeof(ROMState), 1);
  return r;
}

static void clean_rom_state(ROMState *state) { free(state); }

static Memmap *make_memmap() {
  Memmap *m = (Memmap *)calloc(
      sizeof(Memmap),
      1); // again, zero init the pointers (and data!!!) to be safe.
  return m;
}

static void clean_memmap(Memmap *state) { free(state); }

static EmuState *make_emu_state() {
  EmuState *state = (EmuState *)calloc(
      sizeof(EmuState), 1); // significantly, all pointers inside this struct
                            // will be initialized to NULL.
  state->cpu_state = make_cpu_state();
  state->rom = make_rom_state();
  state->map = make_memmap();
  return state;
}

static void clean_emu_state(EmuState *state) {
  clean_cpu_state(state->cpu_state);
  clean_rom_state(state->rom);
  clean_memmap(state->map);
}

EmuState *emu_init() { return make_emu_state(); }

void emu_update(EmuState *emu_state, u8 *is_running) {
  if (emu_state->cpu_state
          ->shutting_down) // short circuit if the emulation has brk'd.
    return;

  // pass pointers, the cpu emulation ticker should be able to modify the
  // emulator's memory and cpu state.
  cpu_tick(emu_state->cpu_state, emu_state->map);
}

void emu_clean(EmuState *emu_state) { clean_emu_state(emu_state); }

void execute_instruction(EmuState *emu_state, u8 *instruction,
                         uint instruction_size) {
  // run through a single instruction, overwriting the program counter at a
  // specific point.
  cpu_single_run(emu_state->cpu_state, emu_state->map, instruction,
                 instruction_size);
}

// main public interface-y functions for the emu. ideally, we don't want the
// frontend to call the cpu, apu or ppu directly at all.
void emu_load(EmuState *emu_state, const char *rom_file_path) {
  // update general state variables for the restart of the emulator.
  emu_state->cpu_state->shutting_down = 0; // we're not brk'd anymore.

  // then, load the file and parse the rom into the right data structures
  // through the while loop byte iterator. this is kind of psychotic, but it's
  // really cool.
  FILE *rom_file =
      fopen(rom_file_path, "r"); // why use a string in the second arg? why not
                                 // just make it an enum? whatever.

  // Find the rom_file size
  fseek(rom_file, 0, SEEK_END);
  long rom_file_size = ftell(rom_file);
  rewind(rom_file);

  // ulimit -s says my stack limit is 8192kib. that seems rather high, but i
  // guess it's OK to use 512kib on the stack? i'm still hesitant to. (512kib is
  // the supposed size limit of a real NES cart. we might as well not limit
  // ourselves, right? this could potentially handle much larger, it makes less
  // assumptions about the shape of the rom data. and it's faster! (?))

  u8 *array_magic = (u8 *)ines_magic;

  // Read the file byte by byte, compare in a state machine instead of stack
  // allocing and THEN comparing to the saved member. it should be faster and
  // more portable to do the comparison with the read operation, since it
  // doesn't require monster stack allocation.
  int byte;
  long index = 0;

  //  FILE STRUCTURE: the ines header, then the prg-rom banks, then the chr-rom
  //  banks. we'll parse out the header information, then the banks into the
  //  proper malloced arrays right into the EmuState ROMState substructure.
  while ((byte = fgetc(rom_file)) != EOF) {
    if (index > 0xF) { // we're past the header. parse the rest of the data (prg
                       // and chr rom from the cart) into the right banks.
      if (index <
          emu_state->rom
              ->rom_prg_offset) { // if we're still in the prg-rom section
                                  // of the ines file, parse into the
                                  // prg-rom bank table in the ROMState.
        u8 which_bank =
            (index - emu_state->rom->rom_prg_offset) /
            emu_state->rom
                ->rom_prg_size; // which prg-rom bank are we writing to?
        u32 bank_index =
            (index - emu_state->rom->rom_prg_offset) %
            emu_state->rom->rom_prg_size; // where are we in that bank?

        emu_state->rom->prg_banks[which_bank][bank_index] = byte;

      } else if (index < emu_state->rom->rom_chr_offset) {
        u8 which_bank =
            (index - emu_state->rom->rom_chr_offset) /
            emu_state->rom
                ->rom_chr_size; // which chr-rom bank are we writing to?
        u32 bank_index =
            (index - emu_state->rom->rom_chr_offset) %
            emu_state->rom->rom_chr_size; // where are we in that bank?

        emu_state->rom->chr_banks[which_bank][bank_index] = byte;

      } else {
        break;
      } // we're done parsing, the rest is probably garbage. break!

      index++;
      continue;
    }

    if (index < 4 && array_magic[index] != byte) {
      fprintf(stderr,
              "INES magic doesn't match up with the file at index %d. Actual "
              "magic byte: %02X; Your magic byte: %02X",
              (int)index, array_magic[index], byte);
    } else {
      index++;
      // don't run the rest of the state machine for the first four bytes
      continue;
    }

    // then match each index individually.
    switch (index) {
    case 4: // num of PRG-rom banks.
      // TODO: skip the trainer in the offset pointer if it's present.
      emu_state->rom->num_prg_banks = byte;
      // setup the helper offset variables.
      emu_state->rom->rom_prg_offset = HEADER_SIZE;
      emu_state->rom->rom_prg_size = PRG_BANK_SIZE * byte;
      // malloc all at once, so do the allocation when we've already found the
      // num_chr_banks in the fifth iteration of this loop.
      break;
    case 5: // num of CHR-rom banks.
      emu_state->rom->num_chr_banks = byte;
      // the chr is offset by the ines header and all the prg-rom banks that
      // came before it.
      emu_state->rom->rom_chr_offset =
          emu_state->rom->rom_chr_offset +
          PRG_BANK_SIZE *
              emu_state->rom
                  ->num_prg_banks; // we've already calculated the offset of the
                                   // prg rom, they're parsed in order.
      emu_state->rom->rom_chr_offset = CHR_BANK_SIZE * byte;

      // then, actually malloc the array that's going to hold the PRG_ROM.
      u8 n_prg = emu_state->rom->num_prg_banks;
      u8 n_chr = emu_state->rom->num_chr_banks;

      u8 *base_ptr = (u8 *)malloc(
          (PRG_BANK_SIZE * n_prg) +
          (CHR_BANK_SIZE *
           n_chr)); // malloc a big block, then use the loop to divvy up the
                    // block into the right places in the prg and chr bank
                    // memory in the ROMState structure.

      for (int i = 0; i < n_prg; i++) {
        emu_state->rom->prg_banks[i] =
            base_ptr; // it's already expecting an array of size PRG_BANK_SIZE,
                      // so we give it that and nothing more.
        base_ptr += PRG_BANK_SIZE;
      }

      for (int i = 0; i < n_chr; i++) {
        emu_state->rom->chr_banks[i] = base_ptr;
        base_ptr += CHR_BANK_SIZE;
      }
      break;
    case 6:
      emu_state->rom->flags = byte;
      break;
      // TODO: parse the rest? most of the other stuff is just extra/unused.
    default:
      break;
    }
    index++;
  }

  printf(
      "Parsed header. Found a rom with proper magic.\n [PRG-ROM] size of %d * "
      "16kb, with an offset of %08X into the ines rom.\n [CHR-ROM] size of %d "
      "* "
      "8kb, with an offset of %08X into the ines rom.\n",
      emu_state->rom->num_prg_banks, emu_state->rom->rom_prg_offset,
      emu_state->rom->num_chr_banks, emu_state->rom->rom_chr_offset);

  // now, map the rom structure pointers into the right places on the banks in
  // the memmap.

  // prg banks go in the cpumap, chr goes in the ppumap
  emu_state->map->lower_prg =
      emu_state->rom
          ->prg_banks[0]; // i assume this is the default. TODO: is it?
  emu_state->map->upper_prg = emu_state->rom->prg_banks[1];

  // TODO: does this cart use sram? if so, init the sram section. (is there an
  // init process? it's already zeroed out in the mmap?)

  // TODO: read from the correct banks.
  emu_state->cpu_state->pc = 0x8000;
}

void emu_prg_load(EmuState *emu_state, u8 *prg_rom_array) {}
