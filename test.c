#include "6502_defines.h"
#include "emu.h"

#include <assert.h>

#define TEST_INIT es = emu_init();
#define TEST_END emu_clean(es);

int main(int argc, char *argv[]) {
  EmuState *es;

  {
    TEST_INIT
    u8 buf[] = {0xe8}; // increment X register
    execute_instruction(es, buf, 1);
    assert(es->cpu_state->x == 0x01);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xa9, 0xae}; // load ae immediate into accumulator
    execute_instruction(es, buf, 2);
    assert(es->cpu_state->a == 0xae);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xe6, 0x10}; // increment memory at zero page address
    es->map->wram[0x10] = 0x10;
    execute_instruction(es, buf, 2);
    assert(es->map->wram[0x10] == 0x11);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0x85, 0x10}; // store accumulator in memory at zero page address
    es->cpu_state->a = 0xab;
    execute_instruction(es, buf, 2);
    assert(es->map->wram[0x10] == 0xab);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xa5, 0x10}; // load from memory at zero page address into accumulator
    es->map->wram[0x10] = 0xcc;
    execute_instruction(es, buf, 2);
    assert(es->cpu_state->a == 0xcc);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xc6, 0x10}; // decrement memory at zero page address
    es->map->wram[0x10] = 0x10;
    execute_instruction(es, buf, 2);
    assert(es->map->wram[0x10] == 0x0f);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xca}; // decrement X register
    es->cpu_state->x = 0x02;
    execute_instruction(es, buf, 1);
    assert(es->cpu_state->x == 0x01);
    TEST_END
  }

}
