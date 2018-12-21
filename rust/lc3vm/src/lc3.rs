pub mod op {
    pub const BR:   u16 = 0x0;  // branch
    pub const ADD:  u16 = 0x1;  // add
    pub const LD:   u16 = 0x2;  // load
    pub const ST:   u16 = 0x3;  // store
    pub const JSR:  u16 = 0x4;  // jump register
    pub const AND:  u16 = 0x5;  // bitwise and
    pub const LDR:  u16 = 0x6;  // load register
    pub const STR:  u16 = 0x7;  // store register
    pub const RTI:  u16 = 0x8;  // unused
    pub const NOT:  u16 = 0x9;  // bitwise not
    pub const LDI:  u16 = 0xA;  // load indirect
    pub const STI:  u16 = 0xB;  // store indirect
    pub const JMP:  u16 = 0xC;  // jump
    pub const RES:  u16 = 0xD;  // reserved (unused)
    pub const LEA:  u16 = 0xE;  // load effective address
    pub const TRAP: u16 = 0xF;  // trap
}

mod Reg {
    pub const R0: u16 = 0;
    pub const R1: u16 = 1;
    pub const R2: u16 = 2;
    pub const R3: u16 = 3;
    pub const R4: u16 = 4;
    pub const R5: u16 = 5;
    pub const R6: u16 = 6;
    pub const R7: u16 = 7;
}

mod Flg {
    pub const POS: u16 = 1;
    pub const ZRO: u16 = 2;
    pub const NEG: u16 = 4;
}

mod Trap {
    pub const GETC:  u16 = 0x20;
    pub const OUT:   u16 = 0x21;
    pub const PUTS:  u16 = 0x22;
    pub const IN:    u16 = 0x23;
    pub const PUTSP: u16 = 0x24;
    pub const HALT:  u16 = 0x25;
}

const MEM_SIZE:usize = 1 + std::u16::MAX as usize;

pub struct LC3Vm {
    mem:     [u16; MEM_SIZE],
    reg:     [u16; 8],
    pc:      u16,
    cnd:     u16,
    running: bool,
}

mod ISA {
    use super::op;

    fn regf(reg: u16, start_bit: u16) -> u16 {
        (reg & 0b111) << start_bit
    }

    fn opc(op: u16) -> u16 {
        op >> 12
    }

    pub fn add(dr: u16, sr1: u16, sr2: u16) -> u16 {
        opc(op::ADD) | regf(dr, 9) | regf(sr1, 6) | (1 << 5) | sr2 & 0b111
    }

    pub fn addi(dr: u16, sr1: u16, imm5: u16) -> u16 {
        opc(op::ADD) | regf(dr, 9) | regf(sr1, 6) | imm5 & 0x1F
    }

    pub fn and(dr: u16, sr1: u16, sr2: u16) -> u16 {
        opc(op::AND) | regf(dr, 9) | regf(sr1, 6) | (1 << 5) | sr2 & 0b111
    }

    pub fn andi(dr: u16, sr1: u16, imm5: u16) -> u16 {
        opc(op::AND) | regf(dr, 9) | regf(sr1, 6) | imm5 & 0x1F
    }

    pub fn lea(dr:u16, off9: u16) -> u16 {
        opc(op::LEA) | regf(dr, 9) | off9 & 0x1FF
    }

    pub fn ld(dr:u16, off9:u16) -> u16 {
        opc(op::LD) | regf(dr, 9) | off9 & 0x1FF
    }
}

mod Programs {
    use super::ISA::*;
    use super::Reg::*;
    fn example_program() {
        let _ = [
            lea(  R0, 0),
            add(  R1, R0, R2),
            andi( R1, R0, 7),
            and(  R0, R0, 0), // clear R0
            ld(   R0, 0xFFFF),
        ];
    }
}


fn bits(word: u16, start: u16, len: u32) -> u16 {
    let mask = 2u16.pow(len) - 1;
    mask & (word >> start)
}


fn bit(word:u16, n:u16) -> u16 {
    bits(word, n, 1)
}

fn sign_extend(word: u16, bitlen: u16) -> u16 {
    let msb = bits(word, bitlen - 1, 1);
    match msb {
        0 => word,

        _ =>  {
            let mask = 0xFFFF << bitlen;
            word | mask
        }
    }
}

fn sextbits(word:u16, start:u16, len:u16) -> u16 {
    sign_extend(bits(word, start, len as u32), len)
}

impl LC3Vm {
    const PC_START: u16 = 0x3000;
    const MMIO_START:usize = 0xFE00;
    const MMIO_KBDSR:usize = 0xFE00;
    const MMIO_KBDDR:usize = 0xFE01;

    pub fn new() -> LC3Vm {
        LC3Vm {
            mem:     [0; MEM_SIZE],
            reg:     [0; 8],
            pc:      0,
            cnd:     0,
            running: false,
        }
    }

    fn run(&mut self) {
        self.pc = LC3Vm::PC_START;
        while self.running {
            self.step()
        }
    }

    fn step(&mut self) {
        let instr_word = self.mem[self.pc as usize];
        let op         = bits(instr_word, 11, 4);
        let args       = bits(instr_word, 0, 12);

        match op {
            op::BR   => self.br(args),
            op::ADD  => self.add(args),
            op::LD   => self.ld(args),
            op::ST   => self.st(args),
            op::JSR  => self.jsr(args),
            op::AND  => self.and(args),
            op::LDR  => self.ldr(args),
            op::STR  => self.str(args),
            op::RTI  => self.rti(args),
            op::NOT  => self.not(args),
            op::LDI  => self.ldi(args),
            op::STI  => self.sti(args),
            op::JMP  => self.jmp(args),
            op::RES  => self.res(args),
            op::LEA  => self.lea(args),
            op::TRAP => self.trap(args),
            _ => panic!("Impossible opcode!")
        }
    }


    // Instructions
    fn add_and(&mut self, args: u16, op: fn(u16, u16) -> u16) {
        let dr   = bits(args, 9, 3) as usize;
        let sr1  = bits(args, 6, 3) as usize;
        let mode = bit(args, 5);
        let op2  = match mode {
            0 => self.reg[bits(args, 0, 3) as usize],
            _ => sextbits(args, 0, 5),
        };

        self.reg[dr] = op(self.reg[sr1], op2);

        self.update_flags(self.reg[dr]);
    }

    fn br(&mut self, args:u16) {
        // TODO
        let nzp = bits(args, 9, 3);

        if nzp & self.cnd != 0 {
            self.pc += sextbits(args, 0, 9);
        }
    }


    fn add(&mut self, args: u16) {
        self.add_and(args, |x, y| x + y);
    }

    fn ld(&mut self, args: u16) {
        let dr   = bits(args, 9, 3) as usize;
        let off9 = sextbits(args, 0, 9);

        let addr = self.mem_read(self.pc + off9);
        let val  = self.mem_read(addr);

        self.reg[dr] = val;
        self.update_flags(val);
    }

    fn st(&mut self, args: u16) {
        let dr   = bits(args, 9, 3) as usize;
        let off9 = sextbits(args, 0, 9);
        let val  = self.reg[dr];

        self.mem_write(self.pc + off9, val);
    }

    fn jsr(&mut self, args: u16) {
        let mode = bit(args, 11);

        self.reg[7] = self.pc;

        self.pc = match mode {
            0 => self.reg[bits(args, 6, 3) as usize],
            _ => self.pc + sextbits(args, 0, 11)
        }

    }

    fn and(&mut self, args: u16) {
        self.add_and(args, { |x,y| x & y });
    }

    fn ldr(&mut self, args: u16) {
        let dr = bits(args, 9, 3) as usize;
        let br = bits(args, 6, 3) as usize;
        let off6 = sextbits(args, 0, 6);

        self.reg[dr] = self.mem_read(self.reg[br] + off6);
    }

    fn str(&mut self, args: u16) {
        let sr = bits(args, 9, 3) as usize;
        let br = bits(args, 6, 3) as usize;
        let off6 = sextbits(args, 0, 6);
        self.mem_write(self.reg[br] + off6, self.reg[sr]);
    }

    fn rti(&mut self, args: u16) {
        panic!("RTI called from non trap contexst");
    }

    fn not(&mut self, args: u16) {
        let dr = bits(args, 9, 3) as usize;
        let sr = bits(args, 6, 3) as usize;

        self.reg[dr] = !self.reg[sr];
    }

    fn ldi(&mut self, args: u16) {
        let dr   = bits(args, 9, 3) as usize;
        let off9 = bits(args, 0, 9);
        let addr = self.mem_read(self.pc + off9);
        let val  = self.mem_read(addr);
        self.reg[dr] = val;
        self.update_flags(val);
    }

    fn sti(&mut self, args: u16) {
        let sr   = bits(args, 9, 3) as usize;
        let off9 = bits(args, 0, 9);
        let addr = self.mem_read(self.pc + off9);
        self.mem_write(addr, self.reg[sr]);
    }

    fn jmp(&mut self, args: u16) {
        let br = bits(args, 6, 3) as usize;
        self.pc = self.reg[br]
    }

    fn res(&mut self, args: u16) {
        panic!("Invalid opcode")
    }

    fn lea(&mut self, args: u16) {
        let dr = bits(args, 9, 3);
        self.reg[dr as usize] = self.pc;
    }

    fn trap(&mut self, args: u16) {
        let trapv8 = bits(args, 0, 8);
        match trapv8 {
            Trap::GETC  => self.trap_getc(),
            Trap::OUT   => self.trap_out(),
            Trap::IN    => self.trap_in(),
            Trap::PUTS  => self.trap_puts(),
            Trap::PUTSP => self.trap_putsp(),
            _           => panic!("unknown trapvector")
        }
    }

    // traps

    fn trap_getc(&mut self) {
        let ch = ncurses::getch();
        match ch {
            -1 => panic!("error reading char"),
            _  => {
                self.reg[0] = ch as u8 as u16;
            }
        }
    }

    fn trap_out(&mut self) {
        use std::io::Write;
        let ascii = self.reg[0] as u8 as char;
        print!("{}", ascii);
        std::io::stdout().flush();
    }

    fn trap_in(&mut self) {
        println!("Enter a character");
        self.trap_getc();
    }

    fn trap_puts(&mut self) {
        use std::io::Write;

        let i = self.reg[0] as usize;
        while self.mem[i] &0xFF != 0 {
            print!("{}", self.mem[i] as u8 as char);
        }
        std::io::stdout().flush().ok();
    }

    fn trap_putsp(&mut self) {
        use std::io::Write;

        let i = self.reg[0] as usize;
        loop {
            let ch1 = self.mem[i] >> 8;
            match ch1 {
                0  => break,
                ch => {
                    print!("{}", ch);
                }
            }

            let ch2 = self.mem[i] & 0xFF;
            match ch2 {
                0 => break,
                ch => {
                    print!("{}", ch);
                }
            }

            print!("{}", self.mem[i] as u8 as char);
        }
        std::io::stdout().flush().ok();
    }

    fn update_flags(&mut self, val: u16) -> u16 {
        match val {
            0          => Flg::ZRO,
            n if n > 0 => Flg::POS,
            _          => Flg::NEG
        }
    }

    fn update_kbd(&mut self) {
        // TODO: check input buffer, fill 0xFE01 with char, 0xFE00 with 1 if input buffer not empty
    }

    // Memory and MMIO
    fn mmio(&mut self, addr:u16) {
        match addr {
            0xFE00 => self.update_kbd(),
            _      => ()
        }
    }

    fn mem_read(&mut self, addr:u16) -> u16 {
        if addr > 0xFE00 {
            self.mmio(addr)
        }

        self.mem[addr as usize]
    }

    fn mem_write(&mut self, addr:u16, value:u16) {
        self.mem[addr as usize] = value;
    }

    fn load_image_file(&mut self, path:&str) {
        use byteorder::{BigEndian, ReadBytesExt};
        use std::io::Read;
        let f = std::fs::File::open(path).ok().unwrap();

        let mut rd = std::io::BufReader::new(f);

        let size = rd.read_u16::<BigEndian>().ok().unwrap() as usize;

        for i in 0..(size - 1) {
            let word = rd.read_u16::<BigEndian>();
            match word {
                Err(e) => break,
                Ok(w)  => {self.mem[LC3Vm::PC_START as usize + i] = w;},
            }
        }
    }
}
