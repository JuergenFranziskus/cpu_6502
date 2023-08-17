#![feature(bigint_helper_methods)]
use instruction::{decode, AddrMode, Op};

pub mod instruction;

pub struct Cpu {
    meta: Meta,

    a: u8,
    x: u8,
    y: u8,
    sp: u8,
    pc: u16,
    flags: Flags,

    op: Op,
    addr_mode: AddrMode,
}
impl Cpu {
    pub fn new() -> Self {
        Self {
            meta: Meta::init(),
            a: 0,
            x: 0,
            y: 0,
            sp: 0,
            pc: 0,
            flags: Flags::init(),
            op: Op::BRK,
            addr_mode: AddrMode::Implied,
        }
    }

    pub fn exec(&mut self, bus: &mut impl Bus) {
        self.configure_instruction(bus);
        if self.meta.jammed() {
            return;
        };
        let (addr, val) = self.eval_addr_mode(bus);
        self.exec_op(addr, val, bus);
    }

    fn configure_instruction(&mut self, bus: &mut impl Bus) {
        if self.meta.rst_pending() {
            self.configure_rst(bus);
        } else if self.meta.nmi_pending() && !self.meta.jammed() {
            self.configure_nmi(bus);
        } else if self.meta.irq_pending() && !self.flags.irq_disable() && !self.meta.jammed() {
            self.configure_irq(bus);
        } else {
            if self.meta.jammed() {
                self.do_read(true, true, self.pc, bus);
            } else {
                self.fetch_instruction(bus);
            }
        }
    }
    fn configure_rst(&mut self, bus: &mut impl Bus) {
        self.op = Op::BRK;
        self.addr_mode = AddrMode::Implied;
        self.meta.set_nmi_pending(false);
        self.flags = Flags::init();
        self.meta.set_break_mode(BreakMode::Rst);
        self.meta.set_jammed(false);
        self.read_brk(self.pc, bus);
    }
    fn configure_nmi(&mut self, bus: &mut impl Bus) {
        self.op = Op::BRK;
        self.addr_mode = AddrMode::Implied;
        self.meta.set_nmi_pending(false);
        self.meta.set_break_mode(BreakMode::Nmi);
        self.read_brk(self.pc, bus);
    }
    fn configure_irq(&mut self, bus: &mut impl Bus) {
        self.op = Op::BRK;
        self.addr_mode = AddrMode::Implied;
        self.meta.set_break_mode(BreakMode::Irq);
        self.read_brk(self.pc, bus);
    }
    fn fetch_instruction(&mut self, bus: &mut impl Bus) {
        self.meta.set_break_mode(BreakMode::Brk);
        let byte = self.read_opcode(bus);
        let (op, addr_mode) = decode(byte);
        self.op = op;
        self.addr_mode = addr_mode;
    }

    fn eval_addr_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        match self.addr_mode {
            AddrMode::Implied => self.exec_implied_mode(bus),
            AddrMode::Accumulator => self.exec_accumulator_mode(bus),
            AddrMode::Immediate => self.exec_immediate_mode(bus),
            AddrMode::Relative => self.exec_relative_mode(bus),
            AddrMode::Zero => self.exec_zero_mode(bus),
            AddrMode::ZeroX => self.exec_zero_index_mode(self.x, bus),
            AddrMode::ZeroY => self.exec_zero_index_mode(self.y, bus),
            AddrMode::Absolute => self.exec_absolute_mode(bus),
            AddrMode::AbsoluteX => self.exec_absolute_index_mode(self.x, bus),
            AddrMode::AbsoluteY => self.exec_absolute_index_mode(self.y, bus),
            AddrMode::Indirect => self.exec_indirect_mode(bus),
            AddrMode::XIndirect => self.exec_xindirect_mode(bus),
            AddrMode::IndirectY => self.exec_indirect_y_mode(bus),
        }
    }
    fn exec_implied_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        self.read(self.pc, bus);
        (0, 0)
    }
    fn exec_accumulator_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        self.read(self.pc, bus);
        (0, self.a)
    }
    fn exec_immediate_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let val = self.read_arg_byte(bus);
        (0, val)
    }
    fn exec_relative_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let val = self.read_arg_byte(bus);
        (0, val)
    }
    fn exec_zero_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let addr = self.read_arg_byte(bus) as u16;

        let val = if self.op.reads_operand() {
            self.read(addr, bus)
        } else {
            0
        };

        if self.op.is_rmw() {
            self.write(addr, val, bus);
        }

        (addr, val)
    }
    fn exec_zero_index_mode(&mut self, index: u8, bus: &mut impl Bus) -> (u16, u8) {
        let addr = self.read_arg_byte(bus);
        let _ = self.read(addr as u16, bus);
        let addr = addr.wrapping_add(index) as u16;

        let val = if self.op.reads_operand() {
            self.read(addr, bus)
        } else {
            0
        };
        if self.op.is_rmw() {
            self.write(addr, val, bus);
        }

        (addr, val)
    }
    fn exec_absolute_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let low = self.read_arg_byte(bus) as u16;
        let high = self.read_arg_byte(bus) as u16;
        let addr = low | high << 8;
        let val = if self.op.reads_operand() {
            self.read(addr, bus)
        } else {
            0
        };
        if self.op.is_rmw() {
            self.write(addr, val, bus);
        }

        (addr, val)
    }
    fn exec_absolute_index_mode(&mut self, index: u8, bus: &mut impl Bus) -> (u16, u8) {
        let low = self.read_arg_byte(bus);
        let high = self.read_arg_byte(bus);
        let (low, carry) = low.overflowing_add(index);
        let wrong_address = (low as u16) | (high as u16) << 8;
        let high = if carry { high.wrapping_add(1) } else { high };
        let wrong_value = self.read(wrong_address, bus);

        let addr = (low as u16) | (high as u16) << 8;

        let read = self.op.reads_operand();
        let write = self.op.writes_operand();
        let rmw = self.op.is_rmw();

        if rmw {
            let val = self.read(addr, bus);
            self.write(addr, val, bus);
            (addr, val)
        } else if read {
            if !carry {
                return (addr, wrong_value);
            };
            let val = self.read(addr, bus);
            (addr, val)
        } else if write {
            (addr, wrong_value)
        } else {
            unreachable!()
        }
    }
    fn exec_indirect_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let low = self.read_arg_byte(bus);
        let high = self.read_arg_byte(bus);
        let low_inc = low.wrapping_add(1);

        let addr_low = self.read((low as u16) | (high as u16) << 8, bus);
        let addr_high = self.read((low_inc as u16) | (high as u16) << 8, bus);

        let addr = (addr_low as u16) | (addr_high as u16) << 8;
        (addr, 0)
    }
    fn exec_xindirect_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let offset = self.read_arg_byte(bus);
        self.read(offset as u16, bus);

        let ptr = offset.wrapping_add(self.x);
        let low = self.read(ptr as u16, bus);
        let high = self.read(ptr.wrapping_add(1) as u16, bus);
        let addr = (low as u16) | (high as u16) << 8;

        let val = if self.op.reads_operand() {
            self.read(addr, bus)
        } else {
            0
        };
        if self.op.is_rmw() {
            self.write(addr, val, bus);
        }

        (addr, val)
    }
    fn exec_indirect_y_mode(&mut self, bus: &mut impl Bus) -> (u16, u8) {
        let zero_ptr = self.read_arg_byte(bus);
        let low = self.read(zero_ptr as u16, bus);
        let high = self.read(zero_ptr.wrapping_add(1) as u16, bus);
        let (low, carry) = low.overflowing_add(self.y);
        let wrong_addr = (low as u16) | (high as u16) << 8;
        let high = if carry { high.wrapping_add(1) } else { high };
        let addr = (low as u16) | (high as u16) << 8;
        let wrong_val = self.read(wrong_addr, bus);

        let read = self.op.reads_operand();
        let write = self.op.writes_operand();
        let rmw = self.op.is_rmw();

        if rmw {
            let val = self.read(addr, bus);
            self.write(addr, val, bus);
            (addr, val)
        } else if read {
            if !carry {
                return (addr, wrong_val);
            };
            let val = self.read(addr, bus);
            (addr, val)
        } else if write {
            (addr, wrong_val)
        } else {
            unreachable!()
        }
    }

    fn exec_op(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        use Op::*;
        match self.op {
            ADC => self.exec_adc(val),
            AND => self.exec_and(val),
            ASL => self.exec_asl(addr, val, bus),
            BCC => self.exec_branch(!self.flags.carry(), val, bus),
            BCS => self.exec_branch(self.flags.carry(), val, bus),
            BEQ => self.exec_branch(self.flags.zero(), val, bus),
            BIT => self.exec_bit(val),
            BMI => self.exec_branch(self.flags.negative(), val, bus),
            BNE => self.exec_branch(!self.flags.zero(), val, bus),
            BPL => self.exec_branch(!self.flags.negative(), val, bus),
            BRK => self.exec_brk(bus),
            BVC => self.exec_branch(!self.flags.overflow(), val, bus),
            BVS => self.exec_branch(self.flags.overflow(), val, bus),
            CLC => self.flags.set_carry(false),
            CLD => self.flags.set_decimal(false),
            CLV => self.flags.set_overflow(false),
            CMP => self.exec_cmp(self.a, val),
            CPX => self.exec_cmp(self.x, val),
            CPY => self.exec_cmp(self.y, val),
            DEC => self.exec_dec(addr, val, bus),
            DEX => self.exec_dex(),
            DEY => self.exec_dey(),
            EOR => self.exec_eor(val),
            INC => self.exec_inc(addr, val, bus),
            INX => self.exec_inx(),
            INY => self.exec_iny(),
            JAM => self.exec_jam(),
            JMP => self.pc = addr,
            JSR => self.exec_jsr(addr, bus),
            LDA => self.exec_lda(val),
            LDX => self.exec_ldx(val),
            LDY => self.exec_ldy(val),
            LSR => self.exec_lsr(addr, val, bus),
            NOP => (),
            ORA => self.exec_ora(val),
            PHA => self.exec_pha(bus),
            PHP => self.exec_php(bus),
            PLA => self.exec_pla(bus),
            PLP => self.exec_plp(bus),
            ROL => self.exec_rol(addr, val, bus),
            ROR => self.exec_ror(addr, val, bus),
            RTI => self.exec_rti(bus),
            RTS => self.exec_rts(bus),
            SBC => self.exec_sbc(val),
            SEC => self.flags.set_carry(true),
            SED => self.flags.set_decimal(true),
            SEI => self.flags.set_irq_disable(true),
            STA => self.exec_sta(addr, bus),
            STX => self.exec_stx(addr, bus),
            STY => self.exec_sty(addr, bus),
            TXS => self.sp = self.x,
            TAX => self.exec_tax(),
            TAY => self.exec_tay(),
            TSX => self.exec_tsx(),
            TXA => self.exec_txa(),
            TYA => self.exec_tya(),

            DCP => self.exec_dcp(addr, val, bus),
            ISC => self.exec_isc(addr, val, bus),
            LAX => self.exec_lax(val),
            SAX => self.exec_sax(addr, bus),
            RLA => self.exec_rla(addr, val, bus),
            RRA => self.exec_rra(addr, val, bus),
            SLO => self.exec_slo(addr, val, bus),
            SRE => self.exec_sre(addr, val, bus),
            op => todo!("Operation {op:?} is not implemented"),
        }
    }
    fn exec_adc(&mut self, val: u8) {
        if self.meta.decimal_enabled() && self.flags.decimal() {
            todo!("ADC using BCD mode is not yet implemented");
        }

        self.do_adc(val);
    }
    fn exec_and(&mut self, val: u8) {
        self.a &= val;
        self.set_common_flags(self.a);
    }
    fn exec_asl(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        self.flags.set_carry(val & 128 != 0);
        let res = val << 1;
        self.set_common_flags(res);
        self.write_rmw_result(addr, res, bus);
    }
    fn exec_bit(&mut self, val: u8) {
        let negative = val & 128 != 0;
        let overflow = val & 64 != 0;
        self.flags.set_negative(negative);
        self.flags.set_overflow(overflow);
        self.flags.set_zero(self.a & val == 0)
    }
    fn exec_branch(&mut self, c: bool, val: u8, bus: &mut impl Bus) {
        if !c {
            return;
        };

        self.read(self.pc, bus);
        let (low, carry) = self.pc_low().overflowing_add_signed(val as i8);
        self.pc = u16::from_le_bytes([low, self.pc_high()]);
        if !carry {
            return;
        };

        self.read(self.pc, bus);
        let pch = self.pc_high().wrapping_add(1);
        self.pc = u16::from_le_bytes([low, pch]);
    }
    fn exec_brk(&mut self, bus: &mut impl Bus) {
        let mode = self.meta.break_mode();
        if mode.increment_pc() {
            self.increment_pc()
        };

        self.write_brk(self.sp(), self.pc_high(), bus);
        self.decrement_sp();
        self.write_brk(self.sp(), self.pc_low(), bus);
        self.decrement_sp();
        self.write_brk(self.sp(), self.flags.to_pushable_bits(mode.b_flag()), bus);
        self.decrement_sp();

        let vector = mode.vector();
        let low = self.read_brk(vector, bus);
        let high = self.read_brk(vector + 1, bus);
        self.pc = (low as u16) | (high as u16) << 8;
    }
    fn exec_cmp(&mut self, a: u8, b: u8) {
        let (res, carry) = a.overflowing_sub(b);
        self.flags.set_carry(!carry);
        self.set_common_flags(res);
    }
    fn exec_dec(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let val = val.wrapping_sub(1);
        self.set_common_flags(val);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_dex(&mut self) {
        self.x = self.x.wrapping_sub(1);
        self.set_common_flags(self.x);
    }
    fn exec_dey(&mut self) {
        self.y = self.y.wrapping_sub(1);
        self.set_common_flags(self.y);
    }
    fn exec_eor(&mut self, val: u8) {
        self.a ^= val;
        self.set_common_flags(self.a);
    }
    fn exec_inc(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let val = val.wrapping_add(1);
        self.set_common_flags(val);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_inx(&mut self) {
        self.x = self.x.wrapping_add(1);
        self.set_common_flags(self.x)
    }
    fn exec_iny(&mut self) {
        self.y = self.y.wrapping_add(1);
        self.set_common_flags(self.y)
    }
    fn exec_jam(&mut self) {
        self.meta.set_jammed(true);
    }
    fn exec_jsr(&mut self, addr: u16, bus: &mut impl Bus) {
        self.read(self.sp as u16 + 0x100, bus);
        let pc = self.pc.wrapping_sub(1);
        let pcl = pc as u8;
        let pch = (pc >> 8) as u8;
        self.push(pch, bus);
        self.push(pcl, bus);
        self.pc = addr;
    }
    fn exec_lda(&mut self, val: u8) {
        self.a = val;
        self.set_common_flags(val);
    }
    fn exec_ldx(&mut self, val: u8) {
        self.x = val;
        self.set_common_flags(val);
    }
    fn exec_ldy(&mut self, val: u8) {
        self.y = val;
        self.set_common_flags(val);
    }
    fn exec_lsr(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        self.flags.set_carry(val & 1 != 0);
        let res = val >> 1;
        self.set_common_flags(res);
        self.write_rmw_result(addr, res, bus);
    }
    fn exec_ora(&mut self, val: u8) {
        self.a |= val;
        self.set_common_flags(self.a);
    }
    fn exec_pha(&mut self, bus: &mut impl Bus) {
        self.push(self.a, bus);
    }
    fn exec_php(&mut self, bus: &mut impl Bus) {
        let bits = self.flags.to_pushable_bits(true);
        self.push(bits, bus);
    }
    fn exec_pla(&mut self, bus: &mut impl Bus) {
        self.read(self.sp as u16 + 0x100, bus);
        self.sp = self.sp.wrapping_add(1);

        self.a = self.read(self.sp as u16 + 0x100, bus);
        self.set_common_flags(self.a);
    }
    fn exec_plp(&mut self, bus: &mut impl Bus) {
        self.read(self.sp as u16 + 0x100, bus);
        self.sp = self.sp.wrapping_add(1);

        let bits = self.read(self.sp as u16 + 0x100, bus);
        self.flags = Flags::from_pushable_bits(bits);
    }
    fn exec_rol(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let new_carry = val & 128 != 0;
        let old_carry = if self.flags.carry() { 1 } else { 0 };
        let res = (val << 1) | old_carry;

        self.flags.set_carry(new_carry);
        self.set_common_flags(res);
        self.write_rmw_result(addr, res, bus);
    }
    fn exec_ror(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let new_carry = val & 1 != 0;
        let old_carry = if self.flags.carry() { 128 } else { 0 };
        let res = (val >> 1) | old_carry;

        self.flags.set_carry(new_carry);
        self.set_common_flags(res);
        self.write_rmw_result(addr, res, bus);
    }
    fn exec_rti(&mut self, bus: &mut impl Bus) {
        self.read(self.sp(), bus);
        self.increment_sp();

        self.flags = Flags::from_pushable_bits(self.read(self.sp(), bus));
        self.increment_sp();
        let pcl = self.read(self.sp(), bus);
        self.increment_sp();
        let pch = self.read(self.sp(), bus);

        self.pc = (pcl as u16) | (pch as u16) << 8;
    }
    fn exec_rts(&mut self, bus: &mut impl Bus) {
        self.read(self.sp(), bus);
        self.increment_sp();

        let pcl = self.read(self.sp(), bus);
        self.increment_sp();
        let pch = self.read(self.sp(), bus);

        self.pc = (pcl as u16) | (pch as u16) << 8;
        self.read_arg_byte(bus);
    }
    fn exec_sbc(&mut self, val: u8) {
        if self.meta.decimal_enabled() && self.flags.decimal() {
            todo!("SBC using BCD mode is not yet implemented");
        }
        self.do_sbc(val)
    }
    fn exec_sta(&mut self, addr: u16, bus: &mut impl Bus) {
        self.write(addr, self.a, bus);
    }
    fn exec_stx(&mut self, addr: u16, bus: &mut impl Bus) {
        self.write(addr, self.x, bus);
    }
    fn exec_sty(&mut self, addr: u16, bus: &mut impl Bus) {
        self.write(addr, self.y, bus);
    }
    fn exec_tax(&mut self) {
        self.x = self.a;
        self.set_common_flags(self.x);
    }
    fn exec_tay(&mut self) {
        self.y = self.a;
        self.set_common_flags(self.y);
    }
    fn exec_tsx(&mut self) {
        self.x = self.sp;
        self.set_common_flags(self.x);
    }
    fn exec_txa(&mut self) {
        self.a = self.x;
        self.set_common_flags(self.a);
    }
    fn exec_tya(&mut self) {
        self.a = self.y;
        self.set_common_flags(self.a);
    }

    fn exec_dcp(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let r = val.wrapping_sub(1);

        let (cmp_r, carry) = self.a.overflowing_sub(r);
        self.flags.set_carry(!carry);
        self.set_common_flags(cmp_r);
        self.write_rmw_result(addr, r, bus);
    }
    fn exec_isc(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let val = val.wrapping_add(1);
        self.do_sbc(val);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_lax(&mut self, val: u8) {
        self.a = val;
        self.x = val;
        self.set_common_flags(val);
    }
    fn exec_sax(&mut self, addr: u16, bus: &mut impl Bus) {
        let val = self.a & self.x;
        self.write(addr, val, bus);
    }
    fn exec_rla(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let val_low = if self.flags.carry() { 1 } else { 0 };
        self.flags.set_carry(val & 128 != 0);
        let val = (val << 1) | val_low;

        self.a &= val;
        self.set_common_flags(self.a);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_rra(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        let val_low = if self.flags.carry() { 128 } else { 0 };
        self.flags.set_carry(val & 1 != 0);
        let val = (val >> 1) | val_low;

        self.do_adc(val);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_slo(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        self.flags.set_carry(val & 128 != 0);
        let val = val << 1;
        self.a |= val;
        self.set_common_flags(self.a);
        self.write_rmw_result(addr, val, bus);
    }
    fn exec_sre(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        self.flags.set_carry(val & 1 != 0);
        let val = val >> 1;

        self.a ^= val;
        self.set_common_flags(self.a);
        self.write_rmw_result(addr, val, bus);
    }

    fn do_adc(&mut self, val: u8) {
        let (res, carry) = self.a.carrying_add(val, self.flags.carry());

        let (_, overflow) = (self.a as i8).overflowing_add(val as i8);

        self.a = res;
        self.set_common_flags(self.a);
        self.flags.set_carry(carry);
        self.flags.set_overflow(overflow);
    }
    fn do_sbc(&mut self, val: u8) {
        let (res, borrow) = self.a.borrowing_sub(val, !self.flags.carry());

        let (_, overflow) = (self.a as i8).borrowing_sub(val as i8, !self.flags.carry());

        self.flags.set_overflow(overflow);
        self.flags.set_carry(!borrow);
        self.set_common_flags(res);
        self.a = res;
    }

    fn write_rmw_result(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        match self.addr_mode {
            AddrMode::Accumulator => self.a = val,
            _ => self.write(addr, val, bus),
        }
    }

    fn set_common_flags(&mut self, val: u8) {
        self.flags.set_zero(val == 0);
        self.flags.set_negative(val >= 128);
    }

    fn do_read(&mut self, sync: bool, poll: bool, addr: u16, bus: &mut impl Bus) -> u8 {
        bus.set_halt(false);
        loop {
            if poll {
                self.poll_interrupts(bus)
            };
            bus.set_address(addr);
            bus.set_sync(sync);
            bus.set_read(true);
            bus.cycle(self);
            let ready = !bus.not_ready();
            if ready {
                return bus.data();
            }
            bus.set_halt(true);
        }
    }
    fn do_write(&mut self, poll: bool, addr: u16, val: u8, bus: &mut impl Bus) {
        if poll {
            self.poll_interrupts(bus);
        }
        bus.set_halt(false);
        bus.set_read(false);
        bus.set_address(addr);
        bus.set_data(val);
        bus.cycle(self);
    }

    fn push(&mut self, val: u8, bus: &mut impl Bus) {
        self.write(self.sp(), val, bus);
        self.decrement_sp();
    }

    fn read(&mut self, addr: u16, bus: &mut impl Bus) -> u8 {
        self.do_read(false, true, addr, bus)
    }
    fn write(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        self.do_write(true, addr, val, bus);
    }
    fn read_brk(&mut self, addr: u16, bus: &mut impl Bus) -> u8 {
        self.do_read(false, false, addr, bus)
    }
    fn write_brk(&mut self, addr: u16, val: u8, bus: &mut impl Bus) {
        if self.meta.break_mode().suppress_writes() {
            self.do_read(false, false, addr, bus);
        } else {
            self.do_write(false, addr, val, bus);
        }
    }
    fn read_opcode(&mut self, bus: &mut impl Bus) -> u8 {
        let val = self.do_read(true, true, self.pc, bus);
        self.increment_pc();
        val
    }
    fn read_arg_byte(&mut self, bus: &mut impl Bus) -> u8 {
        let val = self.do_read(false, true, self.pc, bus);
        self.increment_pc();
        val
    }
    fn increment_pc(&mut self) {
        self.pc = self.pc.wrapping_add(1);
    }
    fn decrement_sp(&mut self) {
        self.sp = self.sp.wrapping_sub(1);
    }
    fn increment_sp(&mut self) {
        self.sp = self.sp.wrapping_add(1);
    }

    fn poll_interrupts(&mut self, bus: &mut impl Bus) {
        self.meta.set_irq_pending(bus.irq());
        let nmi = bus.nmi() && !self.meta.last_nmi();
        self.meta.set_nmi_pending(nmi);
        self.meta.set_last_nmi(bus.nmi());
        self.meta.set_rst_pending(bus.rst());
    }

    pub fn a(&self) -> u8 {
        self.a
    }
    pub fn x(&self) -> u8 {
        self.x
    }
    pub fn y(&self) -> u8 {
        self.y
    }
    pub fn flags(&self) -> Flags {
        self.flags
    }
    pub fn sp(&self) -> u16 {
        0x100 | self.sp as u16
    }
    pub fn pc(&self) -> u16 {
        self.pc
    }
    pub fn pc_low(&self) -> u8 {
        self.pc as u8
    }
    pub fn pc_high(&self) -> u8 {
        (self.pc >> 8) as u8
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Meta(u8);
impl Meta {
    pub fn init() -> Self {
        let mut ret = Self(0);
        ret.set_rst_pending(true);
        ret
    }

    pub fn jammed(self) -> bool {
        self.0 & (1 << Self::JAMMED) != 0
    }
    pub fn last_nmi(self) -> bool {
        self.0 & (1 << Self::LAST_NMI) != 0
    }
    pub fn irq_pending(self) -> bool {
        self.0 & (1 << Self::IRQ_SCHEDULED) != 0
    }
    pub fn nmi_pending(self) -> bool {
        self.0 & (1 << Self::NMI_SCHEDULED) != 0
    }
    pub fn break_mode(self) -> BreakMode {
        let bits = (self.0 >> Self::BREAK_MODE) & 0x3;
        match bits {
            0 => BreakMode::Brk,
            1 => BreakMode::Irq,
            2 => BreakMode::Nmi,
            3 => BreakMode::Rst,
            _ => unreachable!(),
        }
    }
    pub fn rst_pending(self) -> bool {
        self.0 & (1 << Self::RST_PENDING) != 0
    }
    pub fn decimal_enabled(self) -> bool {
        self.0 & (1 << Self::DECIMAL_ENABLED) != 0
    }

    pub fn set_jammed(&mut self, jammed: bool) {
        let mask = 1 << Self::JAMMED;
        self.0 &= !mask;
        self.0 |= if jammed { mask } else { 0 };
    }
    pub fn set_last_nmi(&mut self, last_nmi: bool) {
        let mask = 1 << Self::LAST_NMI;
        self.0 &= !mask;
        self.0 |= if last_nmi { mask } else { 0 };
    }
    pub fn set_irq_pending(&mut self, irq_scheduled: bool) {
        let mask = 1 << Self::IRQ_SCHEDULED;
        self.0 &= !mask;
        self.0 |= if irq_scheduled { mask } else { 0 };
    }
    pub fn set_nmi_pending(&mut self, nmi_scheduled: bool) {
        let mask = 1 << Self::NMI_SCHEDULED;
        self.0 &= !mask;
        self.0 |= if nmi_scheduled { mask } else { 0 };
    }
    pub fn set_break_mode(&mut self, mode: BreakMode) {
        let bits = mode as u8;
        let mask = 0x3 << Self::BREAK_MODE;
        self.0 &= !mask;
        self.0 |= bits << Self::BREAK_MODE;
    }
    pub fn set_rst_pending(&mut self, start: bool) {
        let mask = 1 << Self::RST_PENDING;
        self.0 &= !mask;
        self.0 |= if start { mask } else { 0 };
    }
    pub fn set_decimal_enabled(&mut self, enabled: bool) {
        let mask = 1 << Self::DECIMAL_ENABLED;
        self.0 &= !mask;
        self.0 |= if enabled { mask } else { 0 };
    }

    pub fn interrupt_pending(self) -> bool {
        self.irq_pending() || self.nmi_pending() || self.rst_pending()
    }

    const JAMMED: u8 = 0;
    const LAST_NMI: u8 = 1;
    const IRQ_SCHEDULED: u8 = 2;
    const NMI_SCHEDULED: u8 = 3;
    const BREAK_MODE: u8 = 4;
    const RST_PENDING: u8 = 6;
    const DECIMAL_ENABLED: u8 = 7;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BreakMode {
    Brk,
    Irq,
    Nmi,
    Rst,
}
impl BreakMode {
    pub fn suppress_writes(self) -> bool {
        matches!(self, Self::Rst)
    }
    pub fn increment_pc(self) -> bool {
        matches!(self, Self::Brk)
    }

    pub fn b_flag(&self) -> bool {
        matches!(self, Self::Brk)
    }

    pub fn vector(&self) -> u16 {
        match self {
            Self::Nmi => 0xFFFA,
            Self::Rst => 0xFFFC,
            Self::Irq | Self::Brk => 0xFFFE,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Flags(u8);
impl Flags {
    pub fn init() -> Self {
        let mut ret = Self(0);
        ret.set_irq_disable(true);
        ret
    }

    pub fn carry(self) -> bool {
        (self.0 >> Self::CARRY) & 1 != 0
    }
    pub fn zero(self) -> bool {
        (self.0 >> Self::ZERO) & 1 != 0
    }
    pub fn irq_disable(self) -> bool {
        (self.0 >> Self::IRQ_DISABLE) & 1 != 0
    }
    pub fn decimal(self) -> bool {
        (self.0 >> Self::DECIMAL) & 1 != 0
    }
    pub fn overflow(self) -> bool {
        (self.0 >> Self::OVERFLOW) & 1 != 0
    }
    pub fn negative(self) -> bool {
        (self.0 >> Self::NEGATIVE) & 1 != 0
    }

    pub fn set_carry(&mut self, carry: bool) {
        let bit = 1 << Self::CARRY;
        self.0 &= !bit;
        self.0 |= bit * carry as u8;
    }
    pub fn set_zero(&mut self, zero: bool) {
        let bit = 1 << Self::ZERO;
        self.0 &= !bit;
        self.0 |= bit * zero as u8;
    }
    pub fn set_irq_disable(&mut self, irq_disable: bool) {
        let bit = 1 << Self::IRQ_DISABLE;
        self.0 &= !bit;
        self.0 |= bit * irq_disable as u8;
    }
    pub fn set_decimal(&mut self, decimal: bool) {
        let bit = 1 << Self::DECIMAL;
        self.0 &= !bit;
        self.0 |= bit * decimal as u8;
    }
    pub fn set_overflow(&mut self, overflow: bool) {
        let bit = 1 << Self::OVERFLOW;
        self.0 &= !bit;
        self.0 |= bit * overflow as u8;
    }
    pub fn set_negative(&mut self, negative: bool) {
        let bit = 1 << Self::NEGATIVE;
        self.0 &= !bit;
        self.0 |= bit * negative as u8;
    }

    pub fn bits(&self) -> u8 {
        self.0
    }

    pub fn to_pushable_bits(self, brk: bool) -> u8 {
        let brk = if brk { 1 << Self::BREAK } else { 0 };
        let set = 1 << Self::SET;
        self.0 | brk | set
    }
    pub fn from_pushable_bits(bits: u8) -> Self {
        let mask = 0b11001111;
        Self(bits & mask)
    }

    const CARRY: u8 = 0;
    const ZERO: u8 = 1;
    const IRQ_DISABLE: u8 = 2;
    const DECIMAL: u8 = 3;
    const BREAK: u8 = 4;
    const SET: u8 = 5;
    const OVERFLOW: u8 = 6;
    const NEGATIVE: u8 = 7;
}


pub trait Bus {
    fn data(&self) -> u8;
    fn rst(&self) -> bool;
    fn nmi(&self) -> bool;
    fn irq(&self) -> bool;
    fn not_ready(&self) -> bool;

    fn set_data(&mut self, data: u8);
    fn set_address(&mut self, addr: u16);
    fn set_read(&mut self, read: bool);
    fn set_sync(&mut self, sync: bool);
    fn set_halt(&mut self, halt: bool);


    fn cycle(&mut self, cpu: &Cpu);
}
