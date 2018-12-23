extern crate nix;

use nix::sys::termios;
use std::thread;
use std::sync::mpsc;
use std::num::Wrapping;

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

pub struct LC3Vm {
    mem:     [Word; MEM_SIZE],
    reg:     [Word; 8],
    pc:      Word,
    cnd:     u16,
    running: bool,
    kbd:     Kbd
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
            kbd:     Kbd::new()
        }
    }

    pub fn run(&mut self) {
        self.pc = Wrapping(LC3Vm::PC_START);
        self.running = true;
        while self.running {
            self.step()
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

        // println!("nzp={} cnd={}", nzp, self.cnd);
        if nzp & self.cnd != 0 {
            self.pc += Wrapping(sextbits(args.0, 0, 9));
            // println!("branching!");
        } else {
            // println!("passing");
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

    fn rti(&mut self, args: Word) {
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

    fn res(&mut self, args: Word) {
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
            Trap::GETC  => self.trap_getc(),
            Trap::OUT   => self.trap_out(),
            Trap::IN    => self.trap_in(),
            Trap::PUTS  => self.trap_puts(),
            Trap::PUTSP => self.trap_putsp(),
            Trap::HALT  => self.trap_halt(),
            _           => panic!("unknown trapvector")
        }
    }

    // traps

    fn initialize_kbd(&mut self) {
        use nix::sys::termios;
        use std::io::Read;
        let term_orig = termios::tcgetattr(0).unwrap();
        let mut term = termios::tcgetattr(0).unwrap();
        term.local_flags.remove(termios::LocalFlags::ICANON);
        term.local_flags.remove(termios::LocalFlags::ISIG);
        term.local_flags.remove(termios::LocalFlags::ECHO);
        termios::tcsetattr(0, termios::SetArg::TCSADRAIN, &term).unwrap();
    }

    fn getch() {
    }

    fn trap_getc(&mut self) {
        let ch = self.kbd.getch();
        self.reg[0].0 = ch as u16;
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
            Wrapping(0)          => Flg::ZRO,
            Wrapping(n) if n > 0 => Flg::POS,
            _                    => Flg::NEG
        }
    }

    fn kbd_thread(&mut self) {
        while self.running {
            // let c = getch();
            let c = 0;

            // lock memory
            self.mem[LC3Vm::MMIO_KBDSR as usize] = Wrapping(1 << 15);
            self.mem[LC3Vm::MMIO_KBDDR as usize] = Wrapping(c as u8 as u16);
            // unlock
        }

    }

    fn update_kbd(&mut self) {
        let char = self.kbd.getch_timeout(0);
        match char {
            Ok(b) => {
                self.mem[LC3Vm::MMIO_KBDSR as usize] = Wrapping(1 << 15);
                self.mem[LC3Vm::MMIO_KBDDR as usize] = Wrapping(b as u16);
            },
            Err(_e) => {
                self.mem[LC3Vm::MMIO_KBDSR as usize] = Wrapping(0);
            },
        }
    }

    // Memory and MMIO.0
    fn mmio(&mut self, addr:u16) {
        match addr {
            0xFE00 => self.update_kbd(),
            _      => ()
        }
    }

    fn mem_read(&mut self, addr:u16) -> Wrapping<u16> {
        if addr >= 0xFE00 {
            self.mmio(addr)
        }

        self.mem[addr as usize]
    }

    fn mem_write(&mut self, addr:u16, value:u16) {
        self.mem[addr as usize] = Wrapping(value);
    }

    pub fn load_image_file(&mut self, path:&str) {
        use byteorder::{BigEndian, ReadBytesExt};
        let f = std::fs::File::open(path).ok().unwrap();

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
    }
    pub fn debug(&mut self) {

    }
}

struct Kbd {
    rx:mpsc::Receiver<u8>
}

impl Kbd {

    pub fn new() -> Kbd {
        let (tx, rx) = mpsc::channel();

        thread::spawn(move ||{
            loop {
                let b = Kbd::do_getch();
                tx.send(b).ok();
            }
        });

        Kbd {
            rx: rx
        }
    }

    fn do_getch() -> u8 {
        use std::io::Read;
        // Querying original as a separate, since `Termios` does not implement copy
        let orig_term = termios::tcgetattr(0).unwrap();
        let mut term = termios::tcgetattr(0).unwrap();
        // Unset canonical mode, so we get characters immediately
        term.local_flags.remove(termios::LocalFlags::ICANON);
        // Don't generate signals on Ctrl-C and friends
        // term.local_flags.remove(termios::LocalFlags::ISIG);
        // Disable local echo
        term.local_flags.remove(termios::LocalFlags::ECHO);
        termios::tcsetattr(0, termios::SetArg::TCSADRAIN, &term).unwrap();

        let byte = std::io::stdin().bytes().next().unwrap().ok().unwrap();

        termios::tcsetattr(0, termios::SetArg::TCSADRAIN, &orig_term).unwrap();
        byte
    }

    pub fn getch(&mut self) -> u8 {
        self.rx.recv().ok().unwrap()
    }

    pub fn getch_timeout(&mut self, timeout: u64) -> Result<u8, mpsc::RecvTimeoutError> {
        self.rx.recv_timeout(std::time::Duration::from_millis(timeout))
    }
}
