extern crate nix;
extern crate rustyline;

use std::num::Wrapping;
use std::sync::Arc;
// use std::sync::atomic;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use crate::kbd;


pub struct LC3Vm {
    mem:     [Word; MEM_SIZE],
    reg:     [Word; 8],
    pc:      Word,
    cnd:     u16,
    running: bool,
    debug:   Arc<AtomicBool>,
    rl:      rustyline::Editor<()>,
}

impl LC3Vm {
    const PC_START: u16 = 0x3000;
    const MMIO_START:usize = 0xFE00;
    const MMIO_KBDSR:usize = 0xFE00;
    const MMIO_KBDDR:usize = 0xFE01;

    pub fn new() -> LC3Vm {
        LC3Vm {
            mem:     [Wrapping(0); MEM_SIZE],
            reg:     [Wrapping(0); 8],
            pc:      Wrapping(0),
            cnd:     0,
            running: false,
            debug:   Arc::new(AtomicBool::new(false)),
            rl:      rustyline::Editor::new(),
        }
    }

    pub fn run(&mut self) {
        self.pc = Wrapping(LC3Vm::PC_START);
        self.running = true;
        while self.running {
            self.step();
            if self.debug.load(Ordering::Relaxed) {
                self.debug_prompt();
            }
        }
    }


    fn step(&mut self) {
        let instr_word = self.mem[self.pc.0 as usize];
        let op         = bits(instr_word.0, 12, 4);
        self.pc += Wrapping(1);

        match op {
            op::BR   => self.br(instr_word),
            op::ADD  => self.add(instr_word),
            op::LD   => self.ld(instr_word),
            op::ST   => self.st(instr_word),
            op::JSR  => self.jsr(instr_word),
            op::AND  => self.and(instr_word),
            op::LDR  => self.ldr(instr_word),
            op::STR  => self.str(instr_word),
            op::RTI  => self.rti(instr_word),
            op::NOT  => self.not(instr_word),
            op::LDI  => self.ldi(instr_word),
            op::STI  => self.sti(instr_word),
            op::JMP  => self.jmp(instr_word),
            op::RES  => self.res(instr_word),
            op::LEA  => self.lea(instr_word),
            op::TRAP => self.trap(instr_word),
            _ => panic!("Impossible opcode!")
        }
    }


    // Instructions
    fn add_and(&mut self, args: Word, op: fn(Word, Word) -> Word) {
        let dr   = bits(args.0, 9, 3) as usize;
        let sr1  = bits(args.0, 6, 3) as usize;
        let mode = bit(args.0, 5);
        let op2  = match mode {
            0 => self.reg[bits(args.0, 0, 3) as usize],
            _ => Wrapping(sextbits(args.0, 0, 5)),
        };

        self.reg[dr] = op(self.reg[sr1], op2);

        self.cnd = self.update_flags(self.reg[dr]);
    }

    fn br(&mut self, args:Word) {
        let nzp = bits(args.0, 9, 3);

        if nzp & self.cnd != 0 {
            self.pc += Wrapping(sextbits(args.0, 0, 9));
        }
    }


    fn add(&mut self, args: Word) {
        self.add_and(args, |x, y| x + y);
    }

    fn ld(&mut self, args: Word) {
        let dr   = bits(args.0, 9, 3) as usize;
        let off9 = Wrapping(sextbits(args.0, 0, 9));

        let addr = self.pc + off9;
        let val  = self.mem_read(addr.0);

        self.reg[dr] = val;
        self.cnd = self.update_flags(val);
    }

    fn st(&mut self, args: Word) {
        let dr   = bits(args.0, 9, 3) as usize;
        let off9 = Wrapping(sextbits(args.0, 0, 9));
        let val  = self.reg[dr];

        self.mem_write((self.pc + off9).0, val.0);
    }

    fn jsr(&mut self, args: Word) {
        let mode = bit(args.0, 11);

        self.reg[7] = self.pc;

        self.pc = match mode {
            0 => self.reg[bits(args.0, 6, 3) as usize],
            _ => self.pc + Wrapping(sextbits(args.0, 0, 11))
        }

    }

    fn and(&mut self, args: Word) {
        self.add_and(args, { |x,y| x & y });
    }

    fn ldr(&mut self, args: Word) {
        let dr = bits(args.0, 9, 3) as usize;
        let br = bits(args.0, 6, 3) as usize;
        let off6 = Wrapping(sextbits(args.0, 0, 6));

        self.reg[dr] = self.mem_read((self.reg[br] + off6).0);
        self.cnd = self.update_flags(self.reg[dr])
    }

    fn str(&mut self, args: Word) {
        let sr = bits(args.0, 9, 3) as usize;
        let br = bits(args.0, 6, 3) as usize;
        let off6 = Wrapping(sextbits(args.0, 0, 6));
        self.mem_write((self.reg[br] + off6).0, self.reg[sr].0);
    }

    fn rti(&mut self, _args: Word) {
        panic!("RTI called from non trap contexst");
    }

    fn not(&mut self, args: Word) {
        let dr = bits(args.0, 9, 3) as usize;
        let sr = bits(args.0, 6, 3) as usize;

        let val = !self.reg[sr];
        self.reg[dr] = val;
        self.cnd = self.update_flags(val);
    }

    fn ldi(&mut self, args: Word) {
        let dr   = bits(args.0, 9, 3) as usize;
        let off9 = Wrapping(sextbits(args.0, 0, 9));
        let addr = self.mem_read((self.pc + off9).0);
        let val  = self.mem_read(addr.0);
        self.reg[dr] = val;
        self.cnd = self.update_flags(val);
    }

    fn sti(&mut self, args: Word) {
        let sr   = bits(args.0, 9, 3) as usize;
        let off9 = Wrapping(sextbits(args.0, 0, 9));
        let addr = self.mem_read((self.pc + off9).0);
        self.mem_write(addr.0, self.reg[sr].0);
    }

    fn jmp(&mut self, args: Word) {
        let br = bits(args.0, 6, 3) as usize;
        self.pc = self.reg[br]
    }

    fn res(&mut self, _args: Word) {
        panic!("Invalid opcode")
    }

    fn lea(&mut self, args: Word) {
        let dr = bits(args.0, 9, 3);
        let off9 = Wrapping(sextbits(args.0, 0, 9));

        let val = self.pc + off9;
        self.reg[dr as usize] = val;
        self.cnd = self.update_flags(val)
    }

    fn trap(&mut self, args: Word) {
        let trapv8 = bits(args.0, 0, 8);
        match trapv8 {
            trap::GETC  => self.trap_getc(),
            trap::OUT   => self.trap_out(),
            trap::IN    => self.trap_in(),
            trap::PUTS  => self.trap_puts(),
            trap::PUTSP => self.trap_putsp(),
            trap::HALT  => self.trap_halt(),
            _           => panic!("unknown trapvector")
        }
    }

    // traps
    fn trap_getc(&mut self) {
        let ch = kbd::getch();
        self.reg[0].0 = ch as u16;
        self.update_flags(self.reg[0]);
    }

    fn trap_out(&mut self) {
        use std::io::Write;
        let ascii = self.reg[0].0 as u8 as char;
        print!("{}", ascii);
        std::io::stdout().flush().ok();
    }

    fn trap_in(&mut self) {
        println!("Enter a character");
        self.trap_getc();
    }

    fn trap_puts(&mut self) {
        use std::io::Write;
        let baseaddr = self.reg[0].0 as usize;

        let charv: Vec<char> = self.mem[baseaddr..]
            .iter()
            .take_while(|w| ((**w).0 & 0xFF) != 0)
            .map(|w| (*w).0 as u8 as char)
            .collect();

        let s: String = charv
            .into_iter()
            .collect();

        print!("{}", s);
        std::io::stdout().flush().ok();
    }

    fn trap_putsp(&mut self) {
        use std::io::Write;
        let baseaddr = self.reg[0].0 as usize;

        let s:String = self.mem[baseaddr..].into_iter()
            .take_while(|w| (**w).0 != 0u16)
            .flat_map(|w| {
                let c1 = (w.0 & 0xFF) as u8 as char;
                let c2 = (w.0 >> 8)   as u8 as char;
                vec![c1, c2]
            })
            .collect();

        print!("{}", s);
        std::io::stdout().flush().ok();
    }

    fn trap_halt(&mut self) {
       self.running = false;
    }

    fn update_flags(&self, val: Wrapping<u16>) -> u16 {
        match val {
            Wrapping(0)                    => flags::ZRO,
            Wrapping(n) if bit(n, 15) == 0 => flags::POS,
            _                              => flags::NEG
        }
    }

    fn update_kbd(&mut self) {
        if kbd::check_input() {
            let b = kbd::getch();
            self.mem[LC3Vm::MMIO_KBDSR] = Wrapping(1 << 15);
            self.mem[LC3Vm::MMIO_KBDDR] = Wrapping(b as u16);
        } else {
            self.mem[LC3Vm::MMIO_KBDSR] = Wrapping(0);
        }
    }

    // Memory and MMIO.0
    fn mmio(&mut self, addr:u16) {
        match addr as usize {
            LC3Vm::MMIO_KBDSR => self.update_kbd(),

            _ => ()
        }
    }

    fn mem_read(&mut self, addr:u16) -> Wrapping<u16> {
        if addr >= LC3Vm::MMIO_START as u16 {
            self.mmio(addr)
        }

        self.mem[addr as usize]
    }

    fn mem_write(&mut self, addr:u16, value:u16) {
        self.mem[addr as usize] = Wrapping(value);
    }

    pub fn load_image_file(&mut self, path:&str) {
        use byteorder::{BigEndian, ReadBytesExt};
        let f = std::fs::File::open(path).expect("Could not open file");

        let mut rd = std::io::BufReader::new(f);

        let origin = rd.read_u16::<BigEndian>().ok().unwrap();

        println!("Loading file {} @ x{:04X}\r", path, origin);

        for addr in origin..std::u16::MAX {
            match rd.read_u16::<BigEndian>() {
                Ok(w)  => {
                    println!("x{:04X} {:04X}\r", addr, w);
                    self.mem[addr as usize] = Wrapping(w);
                },
                Err(_e) => {
                    break;
                },
            }
        }
        println!("{}", "-".repeat(80));
    }

    // Debug

    // pub fn toggle_debug(&self) -> () {
    //     self.debug.fetch_or(true, Ordering::SeqCst);
    // }


    fn print_registers(&self) {
        println!("{}", "-".repeat(80));
        for i in 0..4 {
            println!("R{} = x{:04x}    R{} = x{:04x}", i, self.reg[i], 4 + i, self.reg[4+i]);
        }
        println!("PC   = x{:04x}", self.pc );
        println!("       nzp");
        println!("COND = {:03b}", self.cnd );
    }

    fn print_mem(&self) {
        for i in (self.pc.0 - (10))..(self.pc.0 + 30) {
            println!("{}  x{:04x} x{:04x}", if i == self.pc.0 { ">"} else { " "}, i, self.mem[i as usize % MEM_SIZE]);
        }
    }

    pub fn getdebug(&self) -> Arc<AtomicBool> {
        self.debug.clone()
    }

    pub fn debug_prompt(&mut self) {
        println!("Debug");
        loop {
            let readline = self.rl.readline("lc3db> ");
            match readline {
                Ok(line) => {
                    match line.as_ref() {
                        "q" => std::process::exit(1),
                        "c" => {
                            self.debug.store(false, Ordering::SeqCst);
                            break;
                        },
                        "n" => break,
                        "p" => self.print_registers(),
                        "pm" => self.print_mem(),
                        _ => (),
                    }
                }
                Err(e) => {
                    println!("error {}", e);
                }
            }
        }
    }

}

type Word = Wrapping<u16>;

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

#[allow(dead_code)]
mod reg {
    pub const R0: u16 = 0;
    pub const R1: u16 = 1;
    pub const R2: u16 = 2;
    pub const R3: u16 = 3;
    pub const R4: u16 = 4;
    pub const R5: u16 = 5;
    pub const R6: u16 = 6;
    pub const R7: u16 = 7;
}

mod flags {
    pub const POS: u16 = 1;
    pub const ZRO: u16 = 2;
    pub const NEG: u16 = 4;
}

mod trap {
    pub const GETC:  u16 = 0x20;
    pub const OUT:   u16 = 0x21;
    pub const PUTS:  u16 = 0x22;
    pub const IN:    u16 = 0x23;
    pub const PUTSP: u16 = 0x24;
    pub const HALT:  u16 = 0x25;
}

const MEM_SIZE:usize = 1 + std::u16::MAX as usize;

#[allow(dead_code)]
mod isa {
    use super::op;

    fn regf(reg: u16, start_bit: u16) -> u16 {
        (reg & 0b111) << start_bit
    }

    fn opc(op: u16) -> u16 {
        op << 12
    }

    pub fn add(dr: u16, sr1: u16, sr2: u16) -> u16 {
        opc(op::ADD) | regf(dr, 9) | regf(sr1, 6)  | sr2 & 0b111
    }

    pub fn addi(dr: u16, sr1: u16, imm5: u16) -> u16 {
        opc(op::ADD) | regf(dr, 9) | regf(sr1, 6) | (1 << 5) | imm5 & 0x1F
    }

    pub fn and(dr: u16, sr1: u16, sr2: u16) -> u16 {
        opc(op::AND) | regf(dr, 9) | regf(sr1, 6)  | sr2 & 0b111
    }

    pub fn andi(dr: u16, sr1: u16, imm5: u16) -> u16 {
        opc(op::AND) | regf(dr, 9) | regf(sr1, 6) | (1 << 5) | imm5 & 0x1F
    }

    pub fn lea(dr:u16, off9: u16) -> u16 {
        opc(op::LEA) | regf(dr, 9) | off9 & 0x1FF
    }

    pub fn ld(dr:u16, off9:u16) -> u16 {
        opc(op::LD) | regf(dr, 9) | off9 & 0x1FF
    }
}

#[allow(dead_code)]
mod programs {
    use super::isa::*;
    use super::reg::*;
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

#[cfg(test)]
mod tests {

    use crate::lc3::isa;
    use std::num::Wrapping;
    #[test]
    fn add_immediate_works() {
        let mut vm = crate::lc3::LC3Vm::new();
        vm.mem[0x3000] = Wrapping(isa::addi(0, 0, -5i16 as u16));
        vm.pc = Wrapping(0x3000);
        vm.step();
        assert_eq!(vm.reg[0], Wrapping(-5i16 as u16));
        assert_eq!(vm.pc, Wrapping(0x3001));
        assert_eq!(vm.cnd, 0b100);
    }

    #[test]
    fn add_works() {
        let mut vm = crate::lc3::LC3Vm::new();
        vm.reg[0] = Wrapping(10);
        vm.reg[1] = Wrapping(-1i16 as u16);
        vm.mem[0x3000] = Wrapping(isa::add(2, 1, 0));
        vm.pc = Wrapping(0x3000);
        vm.step();
        assert_eq!(vm.reg[2], Wrapping(9));
        assert_eq!(vm.pc, Wrapping(0x3001));
        assert_eq!(vm.cnd, 0b001);
    }

    #[test]
    fn and_immediate_works() {
        let mut vm = crate::lc3::LC3Vm::new();
        vm.reg[0].0 = 0b1100011;
        vm.mem[0x3000] = Wrapping(isa::andi(0, 0, 0b11));
        vm.pc = Wrapping(0x3000);
        vm.step();
        assert_eq!(vm.reg[0], Wrapping(0b11));
        assert_eq!(vm.pc, Wrapping(0x3001));
        assert_eq!(vm.cnd, 0b001);
    }

    #[test]
    fn and_works() {
        let mut vm = crate::lc3::LC3Vm::new();
        vm.reg[0].0 = 0b1011101;
        vm.reg[1].0 = 0b1100011;
        vm.mem[0x3000] = Wrapping(isa::and(2, 1, 0));
        vm.pc = Wrapping(0x3000);
        vm.step();
        assert_eq!(vm.reg[2], Wrapping(0b1000001));
        assert_eq!(vm.pc, Wrapping(0x3001));
        assert_eq!(vm.cnd, 0b001);
    }

    #[test]
    fn add_works2() {
        let mut vm = crate::lc3::LC3Vm::new();
        vm.reg[0].0 = 0x79;
        vm.reg[1].0 = 0xFF87;
        vm.mem[0x3000] = Wrapping(isa::add(2, 1, 0));
        vm.pc = Wrapping(0x3000);
        vm.step();
        assert_eq!(vm.reg[2], Wrapping(0));
        assert_eq!(vm.pc, Wrapping(0x3001));
        assert_eq!(vm.cnd, 0b010);
    }
}
