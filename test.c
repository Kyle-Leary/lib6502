#include "6502_defines.h"
#include "emu.h"

#include <assert.h>

#define TEST_INIT es = emu_init();
#define TEST_END emu_clean(es);

int main(int argc, char *argv[]) {
  EmuState *es;

  {
    TEST_INIT
    u8 buf[] = {0xe8};
    execute_instruction(es, buf, 1);
    assert(es->cpu_state->x == 0x01);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {
        0xe8}; // write inx at the program counter (0x00 as a wram index)
    execute_instruction(es, buf, 1);
    assert(es->cpu_state->pc == 0x01);
    execute_instruction(es, buf, 1);
    assert(es->cpu_state->pc == 0x02);
    u8 buftwo[] = {0xa9, 0xae};
    execute_instruction(es, buftwo, 2);
    printf("0x%02X\n", es->cpu_state->pc);
    assert(es->cpu_state->pc == 0x04);
    TEST_END
  }

  {
    TEST_INIT
    u8 buf[] = {0xa9, 0xae}; // load ae immediate into accumulator
    printf("%d\n", es->cpu_state->pc);
    execute_instruction(es, buf, 2);
    assert(es->cpu_state->a == 0xae);
    TEST_END
  }
}
