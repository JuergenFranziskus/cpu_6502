# CPU_6502

This is just a little somewhat okay emulator for the MOS6502 CPU.  
It is written as a Rust library to be easily used as part of other, more
specific emulation systems.

## Features
- Instruction-stepped, cycle-ticked

    The emulator can only be stepped forward by whole instructions,  
    but accesses the memory bus in the same order and amount as the real CPU.

- Disabling of BCD implementation

    The emulator can be set to ignore the Decimal flag,
    in which case it will simply always execute the binary operation.



## Correctness
The bulk of the implementation was copied over from an
NES emulator I had been writing.  
There, it passed various test programs for correctness and cycle accuracy.  
The code now present in this repository has not been tested since then.

## Project Status / Issues
The Emulator is pretty much in a finished state.  
Some of the less useful illegal opcodes of the 6502 remain unimplemented.  
Unit tests are not yet implemented.  
It currently crashes when BCD mode is used.  
Performance could probably be improved.
These things may be fixed at some point, but probably not.
