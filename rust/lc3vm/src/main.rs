// instructions opcodes

mod Op {
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

struct LC3Vm {
    mem:     [u16; MEM_SIZE],
    reg:     [u16; 8],
    pc:      u16,
    cnd:     u16,
    running: bool,
}

fn bits(word: u16, start: u16, len: u32) -> u16 {
    let mask = 2u16.pow(len) - 1;
    mask & (word >> start)
}

fn sign_extend(word: u16, bitlen: u16) -> u16 {
    let msb = bits(word, bitlen - 1, 1);
    if msb == 1 {
        let mask = 0xFFFF << bitlen;
        word | mask
    } else {
        word
    }
}

impl LC3Vm {
    const PC_START: u16 = 0x3000;

    fn new() -> LC3Vm {
        LC3Vm {
            mem:     [0; MEM_SIZE],
            reg:     [0; 8],
            pc:      0,
            cnd:     0,
            running: false,
        }
    }

    fn br(&mut self, args: u16) {
        // TODO
    }

    fn add_and(&mut self, args: u16, op: fn(u16, u16) -> u16) {
        let dr  = bits(args, 9, 3) as usize;
        let sr1 = bits(args, 6, 3) as usize;
        let mode = bits(args, 5, 1);

        if mode == 1 {
            let imm5 = sign_extend( bits(args, 0, 5), 5 );
            self.reg[dr] = op( self.reg[sr1], imm5);
        } else {
            let sr2 = bits(args, 3, 0) as usize;
            self.reg[dr] = op(self.reg[sr1 as usize], self.reg[sr2]);
        }
    }

    fn add(&mut self, args: u16) {
        self.add_and(args, |x, y| x + y);
    }

    fn ld(&mut self, args: u16) {
        // TODO
    }

    fn st(&mut self, args: u16) {
        // TODO
    }

    fn jsr(&mut self, args: u16) {

    }

    fn and(&mut self, args: u16) {
        self.add_and(args, { |x,y| x & y });
    }

    fn ldr(&mut self, args: u16) {

    }

    fn str(&mut self, args: u16) {

    }

    fn rti(&mut self, args: u16) {

    }

    fn not(&mut self, args: u16) {

    }

    fn ldi(&mut self, args: u16) {

    }

    fn sti(&mut self, args: u16) {

    }

    fn jmp(&mut self, args: u16) {

    }

    fn res(&mut self, args: u16) {

    }

    fn lea(&mut self, args: u16) {
        let dr = bits(args, 9, 3);
        self.reg[dr as usize] = self.pc;
    }

    fn trap(&mut self, args: u16) {

    }

    fn step(&mut self) {
        let instr_word = self.mem[self.pc as usize];
        let op         = bits(instr_word, 11, 4);
        let args       = bits(instr_word, 0, 12);

        match op {
            Op::BR   => self.br(args),
            Op::ADD  => self.add(args),
            Op::LD   => self.ld(args),
            Op::ST   => self.st(args),
            Op::JSR  => self.jsr(args),
            Op::AND  => self.and(args),
            Op::LDR  => self.ldr(args),
            Op::STR  => self.str(args),
            Op::RTI  => self.rti(args),
            Op::NOT  => self.not(args),
            Op::LDI  => self.ldi(args),
            Op::STI  => self.sti(args),
            Op::JMP  => self.jmp(args),
            Op::RES  => self.res(args),
            Op::LEA  => self.lea(args),
            Op::TRAP => self.trap(args),
            _ => panic!
        }
    }

    fn run(&mut self) {
        self.pc = LC3Vm::PC_START;
        while self.running {
            self.step()
        }
    }

}

fn main() {
    println!("Hello, world!");
}
