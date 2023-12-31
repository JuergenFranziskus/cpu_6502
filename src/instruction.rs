use std::fmt::Display;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Op {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,

    // Illegal operations
    ALR,
    ANC,
    ANE,
    ARR,
    DCP,
    ISC,
    LAS,
    LAX,
    LXA,
    RLA,
    RRA,
    SAX,
    SBX,
    SHA,
    SHX,
    SHY,
    SLO,
    SRE,
    TAS,
    JAM,
}
impl Op {
    pub fn reads_operand(self) -> bool {
        use Op::*;
        self.is_rmw()
            || matches!(
                self,
                LDA | LDX | LDY | EOR | AND | ORA | ADC | SBC | CMP | CPX | CPY | BIT | LAX | NOP
            )
    }
    pub fn writes_operand(self) -> bool {
        use Op::*;
        self.is_rmw() || matches!(self, STA | STX | STY | SAX)
    }
    pub fn is_rmw(self) -> bool {
        use Op::*;
        matches!(
            self,
            ASL | LSR | ROL | ROR | INC | DEC | SLO | SRE | RLA | RRA | ISC | DCP | SHA | SHX | SHY
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AddrMode {
    Implied,
    Immediate,
    Relative,
    Accumulator,
    Zero,
    ZeroX,
    ZeroY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    XIndirect,
    IndirectY,
}
impl Display for AddrMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let str = match self {
            Self::Implied => "",
            Self::Immediate => "#",
            Self::Relative => "#",
            Self::Accumulator => "A",
            Self::Zero => "[#]",
            Self::ZeroX => "[#+X]",
            Self::ZeroY => "[#+Y]",
            Self::Absolute => "[$]",
            Self::AbsoluteX => "[$+X]",
            Self::AbsoluteY => "[$+Y]",
            Self::Indirect => "[[$]]",
            Self::XIndirect => "[[#+X]]",
            Self::IndirectY => "[[#] + Y]",
        };

        Display::fmt(str, f)
    }
}

pub fn decode(opcode: u8) -> (Op, AddrMode) {
    DECODE_TABLE[opcode as usize]
}

use AddrMode::*;
use Op::*;
static DECODE_TABLE: [(Op, AddrMode); 256] = [
    (BRK, Implied),
    (ORA, XIndirect),
    (JAM, Implied),
    (SLO, XIndirect),
    (NOP, Zero),
    (ORA, Zero),
    (ASL, Zero),
    (SLO, Zero),
    (PHP, Implied),
    (ORA, Immediate),
    (ASL, Accumulator),
    (ANC, Immediate),
    (NOP, Absolute),
    (ORA, Absolute),
    (ASL, Absolute),
    (SLO, Absolute),
    (BPL, Relative),
    (ORA, IndirectY),
    (JAM, Implied),
    (SLO, IndirectY),
    (NOP, ZeroX),
    (ORA, ZeroX),
    (ASL, ZeroX),
    (SLO, ZeroX),
    (CLC, Implied),
    (ORA, AbsoluteY),
    (NOP, Implied),
    (SLO, AbsoluteY),
    (NOP, AbsoluteX),
    (ORA, AbsoluteX),
    (ASL, AbsoluteX),
    (SLO, AbsoluteX),
    (JSR, Absolute),
    (AND, XIndirect),
    (JAM, Implied),
    (RLA, XIndirect),
    (BIT, Zero),
    (AND, Zero),
    (ROL, Zero),
    (RLA, Zero),
    (PLP, Implied),
    (AND, Immediate),
    (ROL, Accumulator),
    (ANC, Immediate),
    (BIT, Absolute),
    (AND, Absolute),
    (ROL, Absolute),
    (RLA, Absolute),
    (BMI, Relative),
    (AND, IndirectY),
    (JAM, Implied),
    (RLA, IndirectY),
    (NOP, ZeroX),
    (AND, ZeroX),
    (ROL, ZeroX),
    (RLA, ZeroX),
    (SEC, Implied),
    (AND, AbsoluteY),
    (NOP, Implied),
    (RLA, AbsoluteY),
    (NOP, AbsoluteX),
    (AND, AbsoluteX),
    (ROL, AbsoluteX),
    (RLA, AbsoluteX),
    (RTI, Implied),
    (EOR, XIndirect),
    (JAM, Implied),
    (SRE, XIndirect),
    (NOP, Zero),
    (EOR, Zero),
    (LSR, Zero),
    (SRE, Zero),
    (PHA, Implied),
    (EOR, Immediate),
    (LSR, Accumulator),
    (ALR, Immediate),
    (JMP, Absolute),
    (EOR, Absolute),
    (LSR, Absolute),
    (SRE, Absolute),
    (BVC, Relative),
    (EOR, IndirectY),
    (JAM, Implied),
    (SRE, IndirectY),
    (NOP, ZeroX),
    (EOR, ZeroX),
    (LSR, ZeroX),
    (SRE, ZeroX),
    (CLI, Implied),
    (EOR, AbsoluteY),
    (NOP, Implied),
    (SRE, AbsoluteY),
    (NOP, AbsoluteX),
    (EOR, AbsoluteX),
    (LSR, AbsoluteX),
    (SRE, AbsoluteX),
    (RTS, Implied),
    (ADC, XIndirect),
    (JAM, Implied),
    (RRA, XIndirect),
    (NOP, Zero),
    (ADC, Zero),
    (ROR, Zero),
    (RRA, Zero),
    (PLA, Implied),
    (ADC, Immediate),
    (ROR, Accumulator),
    (ARR, Immediate),
    (JMP, Indirect),
    (ADC, Absolute),
    (ROR, Absolute),
    (RRA, Absolute),
    (BVS, Relative),
    (ADC, IndirectY),
    (JAM, Implied),
    (RRA, IndirectY),
    (NOP, ZeroX),
    (ADC, ZeroX),
    (ROR, ZeroX),
    (RRA, ZeroX),
    (SEI, Implied),
    (ADC, AbsoluteY),
    (NOP, Implied),
    (RRA, AbsoluteY),
    (NOP, AbsoluteX),
    (ADC, AbsoluteX),
    (ROR, AbsoluteX),
    (RRA, AbsoluteX),
    (NOP, Immediate),
    (STA, XIndirect),
    (NOP, Immediate),
    (SAX, XIndirect),
    (STY, Zero),
    (STA, Zero),
    (STX, Zero),
    (SAX, Zero),
    (DEY, Implied),
    (NOP, Immediate),
    (TXA, Implied),
    (ANE, Immediate),
    (STY, Absolute),
    (STA, Absolute),
    (STX, Absolute),
    (SAX, Absolute),
    (BCC, Relative),
    (STA, IndirectY),
    (JAM, Implied),
    (SHA, IndirectY),
    (STY, ZeroX),
    (STA, ZeroX),
    (STX, ZeroY),
    (SAX, ZeroY),
    (TYA, Implied),
    (STA, AbsoluteY),
    (TXS, Implied),
    (TAS, AbsoluteY),
    (SHY, AbsoluteX),
    (STA, AbsoluteX),
    (SHX, AbsoluteY),
    (SHA, AbsoluteY),
    (LDY, Immediate),
    (LDA, XIndirect),
    (LDX, Immediate),
    (LAX, XIndirect),
    (LDY, Zero),
    (LDA, Zero),
    (LDX, Zero),
    (LAX, Zero),
    (TAY, Implied),
    (LDA, Immediate),
    (TAX, Implied),
    (LXA, Immediate),
    (LDY, Absolute),
    (LDA, Absolute),
    (LDX, Absolute),
    (LAX, Absolute),
    (BCS, Relative),
    (LDA, IndirectY),
    (JAM, Implied),
    (LAX, IndirectY),
    (LDY, ZeroX),
    (LDA, ZeroX),
    (LDX, ZeroY),
    (LAX, ZeroY),
    (CLV, Implied),
    (LDA, AbsoluteY),
    (TSX, Implied),
    (LAS, AbsoluteY),
    (LDY, AbsoluteX),
    (LDA, AbsoluteX),
    (LDX, AbsoluteY),
    (LAX, AbsoluteY),
    (CPY, Immediate),
    (CMP, XIndirect),
    (NOP, Immediate),
    (DCP, XIndirect),
    (CPY, Zero),
    (CMP, Zero),
    (DEC, Zero),
    (DCP, Zero),
    (INY, Implied),
    (CMP, Immediate),
    (DEX, Implied),
    (SBX, Immediate),
    (CPY, Absolute),
    (CMP, Absolute),
    (DEC, Absolute),
    (DCP, Absolute),
    (BNE, Relative),
    (CMP, IndirectY),
    (JAM, Implied),
    (DCP, IndirectY),
    (NOP, ZeroX),
    (CMP, ZeroX),
    (DEC, ZeroX),
    (DCP, ZeroX),
    (CLD, Implied),
    (CMP, AbsoluteY),
    (NOP, Implied),
    (DCP, AbsoluteY),
    (NOP, AbsoluteX),
    (CMP, AbsoluteX),
    (DEC, AbsoluteX),
    (DCP, AbsoluteX),
    (CPX, Immediate),
    (SBC, XIndirect),
    (NOP, Immediate),
    (ISC, XIndirect),
    (CPX, Zero),
    (SBC, Zero),
    (INC, Zero),
    (ISC, Zero),
    (INX, Implied),
    (SBC, Immediate),
    (NOP, Implied),
    (SBC, Immediate),
    (CPX, Absolute),
    (SBC, Absolute),
    (INC, Absolute),
    (ISC, Absolute),
    (BEQ, Relative),
    (SBC, IndirectY),
    (JAM, Implied),
    (ISC, IndirectY),
    (NOP, ZeroX),
    (SBC, ZeroX),
    (INC, ZeroX),
    (ISC, ZeroX),
    (SED, Implied),
    (SBC, AbsoluteY),
    (NOP, Implied),
    (ISC, AbsoluteY),
    (NOP, AbsoluteX),
    (SBC, AbsoluteX),
    (INC, AbsoluteX),
    (ISC, AbsoluteX),
];
