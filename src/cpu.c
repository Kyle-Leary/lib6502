#include "cpu.h"
#include "api/6502_defines.h"
#include "api/cpu.h"
#include "cpu_mapper.h"
#include "emu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

CPUState *make_cpu_state() {
  CPUState *state = (CPUState *)malloc(sizeof(CPUState));

  state->pc = 0; // to be read from the reset vector, when that gets setup.

  state->sp = 0xFF; // top of the stack, at $01FF, goes down from the 8-bit
                    // position there until 0, where it overflows?

  state->a = 0; // accumulator
  state->x = 0; // gp registers
  state->y = 0;

  state->status = 0b00100000; // flip the unused bit? does it matter?

  state->shutting_down = 0; // brk register, not real

  return state;
}

void clean_cpu_state(CPUState *state) {}

void cpu_debug_print(CPUState *cs) {
  // Print CPUState info
  printf("  CPUState:\n");
  if (cs != NULL) {
    printf("    PC: 0x%04X\n", cs->pc);
    printf("    A: 0x%02X\n", cs->a);
    printf("    X: 0x%02X\n", cs->x);
    printf("    Y: 0x%02X\n", cs->y);
    printf("    Status: 0b%08B\n", cs->status);
    printf("    Shutting down: %s\n", cs->shutting_down ? "Yes" : "No");
  } else {
    printf("    CPUState is NULL\n");
  }
}

// TODO: find better place for overall debug print of the EMUState, maybe an EMU
// module that submodules the PPU, APU and CPU? how will submodules work with
// this architecture?
void debug_print(EmuState *state) {
  if (state == NULL) {
    printf("EmuState is NULL\n");
    return;
  }

  cpu_debug_print(state->cpu_state);
}

/// STATUS HELPERS

// flip the passed statusbit
void toggle_status(CPUState *cs, StatusBit s) {
  cs->status ^= s; // since the enum IS the bit value, this works.
} // XOR is toggling.

void unset_status(CPUState *cs, StatusBit s) {
  cs->status &= ~s;
} // AND with the complement to disable.

void set_status(CPUState *cs, StatusBit s) { cs->status |= s; }

void force_status(CPUState *cs, StatusBit s, u8 is_on) {
  if (is_on)
    set_status(cs, s);
  else
    unset_status(cs, s);
}

u8 is_status_set(StatusBit s, CPUState *cs) {
  return ((cs->status & s) != 0);
} // the is_set function, but for the status register and its special enum
  // layout.

// 1 if set, 0 if not set.
// is which_bit on byte set?
u8 is_set(u8 which_bit, u8 byte) {
  return ((byte & (1 << which_bit)) != 0);
} // if it's nonzero at all, that's evidence that the bit is set.

void handle_neg(CPUState *cs, u8 target) {
  force_status(cs, Negative, is_set(7, target));
}

void handle_zero(CPUState *cs, u8 target) {
  force_status(cs, Zero, target == 0);
}

// set zero if the target is zero,
// and set negative if bit 7 (leftmost) of target is set.
void neg_and_zero(CPUState *cs, u8 target) {
  handle_neg(cs, target);
  handle_zero(cs, target);
}

// pass the handle_carry() the old byte that needs to be operated on.
void handle_carry(CPUState *cs, u8 target) {
  force_status(
      cs, Carry,
      is_set(7, target)); // Carry matches the 7th bit of the target value.
}

// the status setup used in the cmp opcodes.
void handle_compare(CPUState *cs, u8 reg, u8 compare_memory) {
  force_status(cs, Negative,
               (reg - compare_memory) <
                   0); // set the zero, unset it if it's not true. i'm going
                       // with the assumption that the status flag should be
                       // UNSET if the condition DOESN'T hold.

  force_status(cs, Zero,
               (reg == compare_memory)); // if they're equal, set the zero flag.
  force_status(cs, Carry,
               (reg >= compare_memory)); // if they're equal, set the zero flag.
}

/// bit helpers
/// note: none of these methods take pointers. you're expected to handle the
/// return value.
// general u8 bit toggling function.
u8 toggle_bit(u8 target, u8 bit) { return (target ^= (1 << bit)); }

// is the bit in the target set?
u8 is_bit(u8 target, u8 bit) {
  return ((target &= (1 << bit)) >> bit);
} // shift it right, get rid of the extra bits around it with an AND, then shift
  // the result back into the ones position.
u8 set_bit(u8 target, u8 bit) { return (target |= (1 << bit)); }
u8 unset_bit(u8 target, u8 bit) {
  return (target &= ~(1 << bit));
} // unset is the complement of setting?

u8 force_bit(u8 target,  // pass the target, will be returned changed.
             u8 bit,     // pass the bit index, from the right to the left.
             u8 is_on) { // force a specific value for a specific bit.
  if (is_on)
    target = set_bit(target, bit);
  else
    target = unset_bit(target, bit);
  return target;
}

/// RAM rw helper functions.
// this is the CPU reading a memory address in the cpu address space, so we need
// to read through the cpu mapper functions. maybe introduce a more
// sophisticated system? we're passing through the mapping functions way more
// than we actually need, this is super slow.
// these only need access to the memory map. try to limit the access of state
// through function arguments.
u8 read_8(Memmap *map, u16 address) { return cpuread(map, address); }
u16 read_16(Memmap *map, u16 address) {
  // the lower byte goes higher
  // the higher byte is interpreted as the lower byte of the u16
  return ((u16)read_8(map, address) | ((u16)read_8(map, address + 1)) << 8);
}

void write_8(Memmap *map, u16 address, u8 value) {
  cpuwrite(map, address, value);
}
void write_16(Memmap *map, u16 address, u16 value) {
  write_8(map, address,
          (u8)(value & 0xFF)); // safe cast, don't trust C with this. it might
                               // cast the u8 with a mask, it might not.
  write_8(map, address + 1, (u8)(value << 8));
}

/// stack helpers
#define CHG_SP(times) cpu_state->sp += times
#define FULL_SP_ADDRESS (0x0100 | (u16)cpu_state->sp)

void stack_push_8(Memmap *map, CPUState *cpu_state, u8 value) {
  write_8(map, FULL_SP_ADDRESS, value);
  CHG_SP(-1);
} // move down the stack.
void stack_push_16(Memmap *map, CPUState *cpu_state, u16 value) {
  stack_push_8(map, cpu_state,
               (u8)(value & 0xff)); // lsb comes first, get the lower byte
  stack_push_8(map, cpu_state,
               (u8)(value >> 8)); // THEN msb, get the higher byte.
}

u8 stack_pull_8(Memmap *map, CPUState *cpu_state) {
  u8 ret_value = read_8(map, FULL_SP_ADDRESS);
  CHG_SP(1);
  return ret_value;
} // move up the stack
// pull the msb first, then the lsb. just pushing in reverse.
u16 stack_pull_16(Memmap *map, CPUState *cpu_state) {
  return ((u16)stack_pull_8(map, cpu_state) |
          ((u16)stack_pull_8(map, cpu_state))
              << 8); // msb into the lower byte, then lsb into the higher byte.
}

/// cpu opcode definitions
///
// opcode helpers
//
// so it has come to this
#define CS cpu_state
#define PC cpu_state->pc
#define SP cpu_state->sp
#define A cpu_state->a
#define X cpu_state->x
#define Y cpu_state->y
#define STATUS cpu_state->status

// the idea that most instruction variance is "should i deref the arg, or not?"
// it's mostly just imm or not imm, so abstract the behavior into a macro.
// only the case with byte arguments, a deref can only yield a single byte in
// most useful cases.
#define MAYBE_DEREF_BYTE                                                       \
  u8 final_arg = 0;                                                            \
  switch (mode) {                                                              \
  case Immediate:                                                              \
    final_arg = arg;                                                           \
    break;                                                                     \
  default:                                                                     \
    final_arg = read_8(map, arg);                                              \
    break;                                                                     \
  }
// now the final_arg is dereffed and defined in the greater function scope.

// compare against reg, all the cmp instructions are remarkably similar.
// the big difference is that the X and Y versions have many less addressing
// modes. they should still work the same, this is generic enough with address
// handling.
void cmp_wrapper(CPUState *cpu_state, Memmap *map, u16 arg, AddrMode mode,
                 u8 reg) {
  MAYBE_DEREF_BYTE
  handle_compare(CS, reg, final_arg);
}

// helper for defining the function headers.
// unfortunately, i think it IS required in some circumstances to switch over
// the addressing mode in the actual function itself. the behavior across
// opcodes isn't totally generic.
#define INST(name)                                                             \
  void name(CPUState *cpu_state, Memmap *map, u16 arg, AddrMode mode)

// logic functions.
// CPU instructions.
INST(adc) { // add with carry
  MAYBE_DEREF_BYTE

  // add with the Carry status bit as well.
  A += final_arg + is_status_set(Carry, CS); // actually do the addition

  // then just set status registers.

  u16 temp = (u16)A + (u16)final_arg + (u16)is_status_set(Carry, CS);
  u8 is_overflow = temp > 0xFF; // is there's an overflow?

  force_status(CS, Carry, is_overflow); // set the carry if there's an overflow.

  // Overflow flag: Set if the sign bit is incorrect (two positives give a
  // negative, or two negatives give a positive)
  force_status(CS, Overflow, (~(A ^ final_arg) & (A ^ temp) & 0x80) != 0);
  // (the "sign bit" is the 7th bit, if we interpret the byte as a signed int.)

  neg_and_zero(CS, A);
}

INST(and) {
  MAYBE_DEREF_BYTE
  A &= final_arg;
  neg_and_zero(CS, A);
} // and with the passed in byte of memory (passes in a pointer)

/* about asl and lsr, gpt says:
    *For unsigned integer types, a right shift operation fills the vacant
positions on the left (the most significant bits) with zeros. This means that
when you right shift a value, including the 7th bit, the vacant bit positions
will be filled with zeros.

    *However, for signed integer types, a right shift operation can behave
differently based on the implementation-defined behavior called "sign
extension." If the signed integer has a negative value (i.e., the most
significant bit is set to 1), then most C implementations will perform an
arithmetic right shift. In this case, the vacant bit positions on the left
will be filled with ones if the original value was negative.


so since we're using unsigned chars for everything, the lsr will set the 7 bit
to zero, like we want. we need to manually set the 0th bit to zero in the asl,
since C doesn't do that by default.
  */

INST(asl) { // arithmetic left shift.
  u8 old_value = 0;
  u8 target = 0;
  // the shift ops are more annoying ones where we can either get the acc or a
  // pointer.
  switch (mode) {
  case Implicit:
    old_value = A;
    A <<= 1;
    A = unset_bit(A, 0); // manually unset the 0th bit of A.
    target = A;
    break;
  default:
    target = read_8(map, arg);
    old_value = target;
    target >>= 1;
    target = unset_bit(target, 0); // manually unset the 0th bit of A.
    write_8(map, arg, target);
    break;
  }

  force_status(
      CS, Carry,
      is_bit(old_value,
             7)); // set to the status of the old 7th bit of the target.
  neg_and_zero(CS, target);
}

INST(bit) { // only mutates the status registers.
  u8 final_arg = read_8(
      map,
      arg); // no direct mode, just ZP and ABS, this deref should be fine.
  u8 result = A & final_arg;
  force_status(CS, Zero,
               result); // if the result is zero, the Zero flag is set.
  force_status(CS, Overflow, is_bit(final_arg, 6)); // overflow = M_6
  force_status(CS, Negative, is_bit(final_arg, 7)); // neg = M_7
}

#define BRANCH PC += arg
// all branching instructions take in offsets to save bytes on the opcode.
INST(bne) { // branch if not equal. (if the zero flag is clear)
  if (!is_status_set(Zero, CS))
    BRANCH;
}
INST(beq) { // opposite of ne, eq.
  if (is_status_set(Zero, CS))
    BRANCH;
}
INST(bpl) { // branch if positive
  if (!is_status_set(Negative, CS))
    BRANCH;
}
INST(bmi) { // branch if minus
  if (is_status_set(Negative, CS))
    BRANCH;
}
INST(bvc) {
  if (!is_status_set(Overflow, CS))
    BRANCH;
}
INST(bvs) {
  if (is_status_set(Overflow, CS))
    BRANCH;
}
INST(bcc) {
  if (!is_status_set(Carry, CS))
    BRANCH;
}
INST(bcs) {
  if (is_status_set(Carry, CS))
    BRANCH;
}

// halt all execution.
INST(brk) { cpu_state->shutting_down = 1; }

// clear status ops
INST(clc) { set_status(CS, Carry); }
INST(cld) { set_status(CS, Decimal); }
INST(cli) { set_status(CS, Interrupt); }
INST(clv) { set_status(CS, Overflow); }

INST(cmp) { cmp_wrapper(cpu_state, map, arg, mode, A); }
INST(cpx) { cmp_wrapper(cpu_state, map, arg, mode, X); }
INST(cpy) { cmp_wrapper(cpu_state, map, arg, mode, Y); }

INST(dec) {
  u8 mem = read_8(map, arg); // only a byte.
  mem--;
  write_8(map, arg, mem); // sync the memory once we've updated it. normal
                          // CPUs would pull from memory batch into the cache.
                          // i wonder if the 6502 has caching of any kind?
} // decrement from memory position, it already passes in the address
  // to us. no switching for this instruction, easy.
INST(dex) {
  X--;
  neg_and_zero(CS, X);
}
INST(dey) {
  Y--;
  neg_and_zero(CS, Y);
}

INST(eor) { // boolean logic ops can take in Immediate OR normal dereffed
            // arguments. this is very similar to AND.
  MAYBE_DEREF_BYTE
  A ^= final_arg;
  neg_and_zero(CS, A);
}

INST(inc) {
  u8 mem = read_8(map, arg);
  mem++;
  write_8(map, arg, mem);
}
INST(inx) {
  X++;
  neg_and_zero(CS, X);
}
INST(iny) {
  Y++;
  neg_and_zero(CS, Y);
}

INST(jmp) {
  printf("Jumping to address %02x\n", arg);
  PC = arg;
}
INST(jsr) {
  stack_push_16(map, cpu_state, PC);
  PC = arg;
} // takes in a raw $c0c0 abs address, forces the program counter to
  // jump to it AFTER dumping PC to the stack. to be used with rts.

// load the addressed value, set flags.
INST(lda) { // all of these are also immediate-optional. most of the time in the
            // docs, "operating on a byte of memory" and a potential Immediate
            // mode version of the opcode means that it uses the MAYBE_DEREF
            // pattern.
  MAYBE_DEREF_BYTE
  A = final_arg;
  neg_and_zero(CS, A);
}
INST(ldx) {
  MAYBE_DEREF_BYTE
  X = final_arg;
  neg_and_zero(CS, X);
}
INST(ldy) {
  MAYBE_DEREF_BYTE
  Y = final_arg; // loading a byte, specifically.
  // these are 8-bit registers.
  neg_and_zero(CS, Y);
}

INST(lsr) { // shift everything in the arg one place to the right.
  u8 old_value = 0;
  u8 target = 0;
  // the shift ops are more annoying ones where we can either get the acc or a
  // pointer.
  switch (mode) {
  case Implicit:
    old_value = A;
    A >>= 1;
    target = A;
    break;
  default:
    target = read_8(map, arg);
    old_value = target;
    target >>= 1;
    write_8(map, arg, target);
    break;
  }

  force_status(CS, Carry,
               old_value &
                   1); // set to the status of the old 0th bit of the target.
  neg_and_zero(CS, target);
}

INST(nop) {} // do nothing!

INST(ora) {
  MAYBE_DEREF_BYTE
  A |= final_arg;
  neg_and_zero(CS, A);
} // just a funny name for OR. works just like EOR and AND.

INST(pha) { stack_push_8(map, cpu_state, A); } // push acc and status bytes.
INST(php) { stack_push_8(map, cpu_state, STATUS); }

INST(pla) {
  A = stack_pull_8(map, cpu_state);
  neg_and_zero(CS, A);
} // pull acc and status bytes, the same except we have to update the neg and
  // zero status registers HERE, but not on the push ops. why?
INST(plp) {
  STATUS = stack_pull_8(map, cpu_state);
} // don't set it here. we're setting it from the stack on all STATUS bits.

INST(rol) {
  u8 target = 0;
  u8 old_target = 0;
  u8 old_carry = (is_status_set(Carry, CS) & 0x01);
  switch (mode) { // either use a memory address, or the accumulator in
                  // implicit/Implicit mode.
  case Implicit:
    old_target = A;
    A <<= 1; // shift then equals. weird.
    target = A;

    handle_neg(CS, A);
    force_status(CS, Carry, A & (1 << 7)); // force the carry to be the seventh
                                           // bit of the accumulator.
    break;
  default:
    target = read_8(map, arg);
    old_target = target;
    target <<= 1;
    write_8(map, arg, target); // write back the new value, update memory.

    handle_neg(CS, target);
    force_status(CS, Carry, target & (1 << 7));
    break;
  }

  target = force_bit(target, 0, old_carry);
  handle_zero(CS,
              A); // it says to handle zero on A no matter what addressing
                  // mode. TODO: does this part of the docs have a typo?
                  // this might be wrong?
}

INST(ror) { // just shift RIGHT this time, and use the 0th bit of the result as
            // the new forced Carry value.
  u8 target = 0;
  u8 old_target = 0;
  u8 old_carry = (is_status_set(Carry, CS) & 0x01);
  switch (mode) {
  case Implicit:
    old_target = A;
    A >>= 1;
    target = A;

    handle_neg(CS, A);
    force_status(CS, Carry, A & (1 << 0));
    break;
  default:
    target = read_8(map, arg);
    old_target = target;
    target >>= 1;
    write_8(map, arg, target);

    handle_neg(CS, target);
    force_status(CS, Carry, target & (1 << 0));
    break;
  }

  // do as much as we can generically.
  target = force_bit(target, 0, old_carry);
  handle_zero(CS, A);
}

INST(rti) { // returning from interrupt, pull the status THEN the PC from the
            // stack.
  STATUS = stack_pull_16(map, cpu_state);
  PC = stack_pull_16(map, cpu_state);
} // so actually, this answers one of my questions. no, it doesn't matter when
  // the reset vector happens in processing, since the nes cpu will just return
  // to the same point that it was before right after the vsync interrupt.
INST(rts) {
  PC = stack_pull_16(map, cpu_state);
} // set the stack pointer. return from the subroutine that we called, really
  // simple on the nes.

INST(sbc) {
  MAYBE_DEREF_BYTE

  // add with the Carry status bit as well.
  A -= final_arg - (1 - is_status_set(Carry, CS)); // actually do the addition

  // be super safe here. let it underflow, so make the integer type that holds
  // the temporary calculations signed.
  int temp = (int)A - (int)final_arg - ((int)1 - (int)is_status_set(Carry, CS));
  u8 is_underflow = (temp) < 0x00; // is there's an overflow?

  force_status(CS, Carry,
               !is_underflow); // CLEAR the carry if there's an underflow,
                               // instead of setting it.

  // TODO: might have fucked this up, i don't really understand the sign bit.
  force_status(CS, Overflow, ((A ^ temp) & (A ^ final_arg) & 0x80) != 0);

  neg_and_zero(CS, A);
}

INST(sec) { set_status(CS, Carry); } // set status flags
INST(sed) { set_status(CS, Decimal); }
INST(sei) { set_status(CS, Interrupt); }

INST(sta) { write_8(map, arg, A); } // store ops.
INST(stx) { write_8(map, arg, X); }
INST(sty) {
  write_8(map, arg, Y);
} // all of these just pass some sort of target address, i think. they don't
  // take in an Immediate type, so we should be fine just raw dereffing the arg.

INST(tax) {
  X = A;
  neg_and_zero(CS, X);
} // X = A, A -> X
INST(tay) {
  Y = A;
  neg_and_zero(CS, Y);
} // Y = A, A -> Y, and etc...
INST(tsx) {
  X = STATUS;
  neg_and_zero(CS, X);
}
INST(txa) {
  A = X;
  neg_and_zero(CS, A);
}
INST(txs) { SP = X; } // X -> SP, don't set any flags.
INST(tya) {
  A = Y;
  neg_and_zero(CS, A);
}

// get the byte and word pointed to by the program counter. this is a proper
// dereference op.
#define PC_BYTE read_8(map, cs->pc)
#define PC_WORD read_16(map, cs->pc)

#define PC_BYTEO(offset) read_8(map, cs->pc + offset)
#define PC_WORDO(offset) read_16(map, cs->pc + offset)
// dont make INC a for loop wtf is wrong with you
#define INC(times) cs->pc += times

// takes in an instruction function and a mode.
// this handles the addressing mode abstractions, giving the function
// the right args and incrementing the pc the right amount to the next
// instruction.
void call(void (*function)(CPUState *, Memmap *, u16, AddrMode), AddrMode mode,
          CPUState *cs, Memmap *map) {
  // arguments are either 8 bits or 16 bits, give the instruction function a 16
  // bit value that can be optionally casted down by the function.
  u16 arg = 0;

  // increment to either the first arg byte or the next instruction.
  INC(1);

  switch (mode) {
  case Implicit:
    // no arg, do nothing
    break;

  case Immediate:
    arg = PC_BYTE;
    INC(1);
    break;

  case Abs: // $0200 -> load the value at the address 0x0200 in RAM.
    arg = PC_WORD;
    INC(2);
    break;
  case AbsX:
    arg = PC_WORDO(cs->x);
    INC(2);
    break;
  case AbsY:
    arg = PC_WORDO(cs->y);
    INC(2);
    break;

  case ZP: // like absolute, but on the zero page.
    arg = PC_BYTE;
    INC(1);
    break;
  case ZPX:
    arg = PC_BYTEO(cs->x);
    INC(1);
    break;
  case ZPY:
    arg = PC_BYTEO(cs->y);
    INC(1);
    break;

  case Relative:
    arg = cs->pc +
          read_8(
              map,
              PC_BYTE); // offset from the current position, used for branching.
    INC(1);
    break;

  case Indirect:
    arg = read_16(map, PC_WORD);
    INC(2);
    break;
  case IndexedIndirect:
    arg = read_16(map, PC_BYTEO(cs->x)); // read the program counter's zero page
                                         // address pointer with an offset of X.
    INC(1);
    break;
  case IndirectIndexed:
    // add y AFTER the dereference, rather than adding cs->x to the pointer
    // itself.
    arg = read_16(map, PC_BYTE) + cs->y;
    INC(1);
    break;

  default:
    printf("Invalid addressing mode.");
    break;
  }

  function(cs, map, arg, mode);
}

// all an instruction, and the cpu in general needs is a pointer to the
// memory mapping scheme and the cpustate, eg the registers. it needs to
// write to memory, and it needs to read instructions and update its own
// state.
#define C(fn, addrmode) call(fn, addrmode, cs, map)

// handlers and etc logic
void handle_instruction(Memmap *map, CPUState *cs) {
  u8 base_instruction =
      read_8(map, cs->pc); // read the instruction at the program counter in the
                           // active prg-rom banks. mapping handles this for us.

  if (IS_DEBUG)
    printf("Running 0x%02X...\n", base_instruction);

  // then, specify the full opcode table, including the different addressing
  // modes.
  switch (base_instruction) {
  case 0x00:
    C(brk, Implicit);
    break;
  case 0x01:
    C(ora, IndexedIndirect);
    break;
  case 0x05:
    C(ora, ZP);
    break;
  case 0x06:
    C(asl, ZP);
    break;
  case 0x08:
    C(php, Implicit);
    break;
  case 0x09:
    C(ora, Immediate);
    break;
  case 0x0A:
    C(asl, Implicit);
    break;
  case 0x0D:
    C(ora, Abs);
    break;
  case 0x0E:
    C(asl, Abs);
    break;
  case 0x10:
    C(bpl, Relative);
    break;
  case 0x11:
    C(ora, IndirectIndexed);
    break;
  case 0x15:
    C(ora, ZPX);
    break;
  case 0x16:
    C(asl, ZPX);
    break;
  case 0x18:
    C(clc, Implicit);
    break;
  case 0x19:
    C(ora, AbsY);
    break;
  case 0x1D:
    C(ora, AbsX);
    break;
  case 0x1E:
    C(asl, AbsX);
    break;
  case 0x20:
    C(jsr, Abs);
    break;
  case 0x21:
    C(and, IndexedIndirect);
    break;
  case 0x24:
    C(bit, ZP);
    break;
  case 0x25:
    C(and, ZP);
    break;
  case 0x26:
    C(rol, ZP);
    break;
  case 0x28:
    C(plp, Implicit);
    break;
  case 0x29:
    C(and, Immediate);
    break;
  case 0x2A:
    C(rol, Implicit);
    break;
  case 0x2C:
    C(bit, Abs);
    break;
  case 0x2D:
    C(and, Abs);
    break;
  case 0x2E:
    C(rol, Abs);
    break;
  case 0x30:
    C(bmi, Relative);
    break;
  case 0x31:
    C(and, IndirectIndexed);
    break;
  case 0x35:
    C(and, ZPX);
    break;
  case 0x36:
    C(rol, ZPX);
    break;
  case 0x38:
    C(sec, Implicit);
    break;
  case 0x39:
    C(and, AbsY);
    break;
  case 0x3D:
    C(and, AbsX);
    break;
  case 0x3E:
    C(rol, AbsX);
    break;
  case 0x40:
    C(rti, Implicit);
    break;
  case 0x41:
    C(eor, IndexedIndirect);
    break;
  case 0x45:
    C(eor, ZP);
    break;
  case 0x46:
    C(lsr, ZP);
    break;
  case 0x48:
    C(pha, Implicit);
    break;
  case 0x49:
    C(eor, Immediate);
    break;
  case 0x4A:
    C(lsr, Implicit);
    break;
  case 0x4C:
    C(jmp, Abs);
    break;
  case 0x4D:
    C(eor, Abs);
    break;
  case 0x4E:
    C(lsr, Abs);
    break;
  case 0x50:
    C(bvc, Relative);
    break;
  case 0x51:
    C(eor, IndirectIndexed);
    break;
  case 0x55:
    C(eor, ZPX);
    break;
  case 0x56:
    C(lsr, ZPX);
    break;
  case 0x58:
    C(cli, Implicit);
    break;
  case 0x59:
    C(eor, AbsY);
    break;
  case 0x5D:
    C(eor, AbsX);
    break;
  case 0x5E:
    C(lsr, AbsX);
    break;
  case 0x60:
    C(rts, Implicit);
    break;
  case 0x61:
    C(adc, IndexedIndirect);
    break;
  case 0x65:
    C(adc, ZP);
    break;
  case 0x66:
    C(ror, ZP);
    break;
  case 0x68:
    C(pla, Implicit);
    break;
  case 0x69:
    C(adc, Immediate);
    break;
  case 0x6A:
    C(ror, Implicit);
    break;
  case 0x6C:
    C(jmp, Indirect);
    break;
  case 0x6D:
    C(adc, Abs);
    break;
  case 0x6E:
    C(ror, Abs);
    break;
  case 0x70:
    C(bvs, Relative);
    break;
  case 0x71:
    C(adc, IndirectIndexed);
    break;
  case 0x75:
    C(adc, ZPX);
    break;
  case 0x76:
    C(ror, ZPX);
    break;
  case 0x78:
    C(sei, Implicit);
    break;
  case 0x79:
    C(adc, AbsY);
    break;
  case 0x7D:
    C(adc, AbsX);
    break;
  case 0x7E:
    C(ror, AbsX);
    break;
  case 0x81:
    C(sta, IndexedIndirect);
    break;
  case 0x84:
    C(sty, ZP);
    break;
  case 0x85:
    C(sta, ZP);
    break;
  case 0x86:
    C(stx, ZP);
    break;
  case 0x88:
    C(dey, Implicit);
    break;
  case 0x8A:
    C(txa, Implicit);
    break;
  case 0x8C:
    C(sty, Abs);
    break;
  case 0x8D:
    C(sta, Abs);
    break;
  case 0x8E:
    C(stx, Abs);
    break;
  case 0x90:
    C(bcc, Relative);
    break;
  case 0x91:
    C(sta, IndirectIndexed);
    break;
  case 0x94:
    C(sty, ZPX);
    break;
  case 0x95:
    C(sta, ZPX);
    break;
  case 0x96:
    C(stx, ZPY);
    break;
  case 0x98:
    C(tya, Implicit);
    break;
  case 0x99:
    C(sta, AbsY);
    break;
  case 0x9A:
    C(txs, Implicit);
    break;
  case 0x9D:
    C(sta, AbsX);
    break;
  case 0xA0:
    C(ldy, Immediate);
    break;
  case 0xA1:
    C(lda, IndexedIndirect);
    break;
  case 0xA2:
    C(ldx, Immediate);
    break;
  case 0xA4:
    C(ldy, ZP);
    break;
  case 0xA5:
    C(lda, ZP);
    break;
  case 0xA6:
    C(ldx, ZP);
    break;
  case 0xA8:
    C(tay, Implicit);
    break;
  case 0xA9:
    C(lda, Immediate);
    break;
  case 0xAA:
    C(tax, Implicit);
    break;
  case 0xAC:
    C(ldy, Abs);
    break;
  case 0xAD:
    C(lda, Abs);
    break;
  case 0xAE:
    C(ldx, Abs);
    break;
  case 0xB0:
    C(bcs, Relative);
    break;
  case 0xB1:
    C(lda, IndirectIndexed);
    break;
  case 0xB4:
    C(ldy, ZPX);
    break;
  case 0xB5:
    C(lda, ZPX);
    break;
  case 0xB6:
    C(ldx, ZPY);
    break;
  case 0xB8:
    C(clv, Implicit);
    break;
  case 0xB9:
    C(lda, AbsY);
    break;
  case 0xBA:
    C(tsx, Implicit);
    break;
  case 0xBC:
    C(ldy, AbsX);
    break;
  case 0xBD:
    C(lda, AbsX);
    break;
  case 0xBE:
    C(ldx, AbsY);
    break;
  case 0xC0:
    C(cpy, Immediate);
    break;
  case 0xC1:
    C(cmp, IndexedIndirect);
    break;
  case 0xC4:
    C(cpy, ZP);
    break;
  case 0xC5:
    C(cmp, ZP);
    break;
  case 0xC6:
    C(dec, ZP);
    break;
  case 0xC8:
    C(iny, Implicit);
    break;
  case 0xC9:
    C(cmp, Immediate);
    break;
  case 0xCA:
    C(dex, Implicit);
    break;
  case 0xCC:
    C(cpy, Abs);
    break;
  case 0xCD:
    C(cmp, Abs);
    break;
  case 0xCE:
    C(dec, Abs);
    break;
  case 0xD0:
    C(bne, Relative);
    break;
  case 0xD1:
    C(cmp, IndirectIndexed);
    break;
  case 0xD5:
    C(cmp, ZPX);
    break;
  case 0xD6:
    C(dec, ZPX);
    break;
  case 0xD8:
    C(cld, Implicit);
    break;
  case 0xD9:
    C(cmp, AbsY);
    break;
  case 0xDD:
    C(cmp, AbsX);
    break;
  case 0xDE:
    C(dec, AbsX);
    break;
  case 0xE0:
    C(cpx, Immediate);
    break;
  case 0xE1:
    C(sbc, IndexedIndirect);
    break;
  case 0xE4:
    C(cpx, ZP);
    break;
  case 0xE5:
    C(sbc, ZP);
    break;
  case 0xE6:
    C(inc, ZP);
    break;
  case 0xE8:
    C(inx, Implicit);
    break;
  case 0xE9:
    C(sbc, Immediate);
    break;
  case 0xEA:
    C(nop, Implicit);
    break;
  case 0xEC:
    C(cpx, Abs);
    break;
  case 0xED:
    C(sbc, Abs);
    break;
  case 0xEE:
    C(inc, Abs);
    break;
  case 0xF0:
    C(beq, Relative);
    break;
  case 0xF1:
    C(sbc, IndirectIndexed);
    break;
  case 0xF5:
    C(sbc, ZPX);
    break;
  case 0xF6:
    C(inc, ZPX);
    break;
  case 0xF8:
    C(sed, Implicit);
    break;
  case 0xF9:
    C(sbc, AbsY);
    break;
  case 0xFD:
    C(sbc, AbsX);
    break;
  case 0xFE:
    C(inc, AbsX);
    break;
  case 0xFF: // print debug, not real or used opcode.
    C(nop, Implicit);
    if (IS_DEBUG)
      cpu_debug_print(cs); // just print the cpu state.
    break;
  default:
    printf("Invalid opcode detected (%02X).\n", base_instruction);
    break;
  }
}

// override the instruction at the pc, just write it over lazily.
static void override_instruction(CPUState *cs, Memmap *map, u8 *instruction,
                                 uint instruction_size) {
  for (int i = 0; i < instruction_size; i++) {
    write_8(map, cs->pc, instruction[i]);
  }
}

void cpu_single_run(CPUState *cs, Memmap *map, u8 *instruction,
                    uint instruction_size) {
  override_instruction(cs, map, instruction, instruction_size);
  handle_instruction(map, cs);
}

// handle stopping the cpu outside, in the emulator core module. this is just a
// dumb function that ticks through the state when we ask it to.
void cpu_tick(CPUState *cs, Memmap *map) { handle_instruction(map, cs); }
