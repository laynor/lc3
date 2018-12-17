module Vm
  module ISA
    extend self
    include Utils

    def fmtsigned(val, bits)
      imm5 = sign_extend(val, bits).to_i16
    end

    def decode(iw)
      opc, args = bitsplit(iw, [4, 12])
      binopmode = bit(iw, 5)
      opc = Op.new(opc.to_i)

      case opc
      when Op::ADD, Op::AND
        case binopmode
        when 0
          dr, sr1, _, sr2 = bitsplit(args, [3, 3, 3, 3])
          "#{opc} R#{dr}, R#{sr1}, R#{sr2}"
        else 1
          dr, sr1, _, imm5 = bitsplit(args, [3, 3, 1, 5])
          "#{opc} R#{dr}, R#{sr1}, ##{fmtsigned(imm5, 5)}"
        end

      when Op::NOT
        dr, sr, _ = bitsplit(args, [3, 3, 6])
        "#{opc} R#{dr}, R#{sr}"

      when Op::TRAP
        _, trapv = bitsplit(args, [4, 8])
        "#{opc} ##{Trap.new(trapv.to_i)}"

      when Op::BR
        nzp, off9 = bitsplit(args, [3, 9])
        flag = Flg.new(nzp.to_i)
        "#{opc} #{flag} ##{fmtsigned(off9, 9)}"

      when Op::LD, Op::LDI, Op::ST, Op::STI, Op::LEA
        dr, off9 = bitsplit(args, [3, 9])
        "#{opc} R#{dr} ##{fmtsigned(off9, 9)}"

      when Op::LDR, Op::STR
        dr, sr, off6 = bitsplit(args, [3, 3, 6])
        "#{opc} R#{dr}, R#{sr} ##{fmtsigned(off6, 6)}"

      when Op::JSR
        mode, off11 = bitsplit(args, [1, 11])
        if mode == 1
          "#{opc} #{fmtsigned(off11, 11)}"
        else
          _, baseR, _ = bitsplit(args, [3, 3, 6])
          "#{opc}R R#{baseR}"
        end

      when Op::JMP
        _, baseR, _ = bitsplit(args, [3, 3, 6])
        if baseR == 0b111
          "RET"
        else
          "#{opc} R#{baseR}"
        end

      when Op::RTI # 13
        "#{opc}"

      when Op::RES
        "#{opc}"
      end
    end

    def add(dr, sr1, sr2) 
      (Op::ADD << 12) | (dr << 9) | (sr1 << 6) | (sr2 << 0) 
    end

    def addi(dr, sr1, imm)
      #       opcode dr  sr1 m imm5
      (Op::ADD << 12) | (dr << 9) | (sr1 << 6) | 1 << 5 | (imm & 0b11111)
    end 

    def and(dr, sr1, sr2)
      (Op::AND << 12) | (dr << 9) | (sr1 << 6) | sr2
    end

    def andi(dr, sr1, imm)
      (Op::AND << 12) | (dr << 9) | (sr1 << 6) | 1 << 5 | (imm & 0b11111)
    end

    def rti
      Op::RTI << 12
    end

    def ld(dr, label)
      (Op::LD << 12) | (dr << 9) | label
    end

    def ldi(dr, off9)
      (Op::LDI << 12) | (dr << 9) | (off9 & 0b1_1111_1111)
    end 

    def ldr(dr, baseR, off6)
      (Op::LDR << 12) | (dr << 9) | (baseR << 6) | off6
    end

    def st(sr, off9)
      (Op::ST << 12) | (sr << 9) | off9
    end 

    def sti(dr, off9)
      (Op::STI << 12) | (dr << 9) | (off9 & 0b1_1111_1111)  
    end 

    def str(sr, baseR, off6)
      (Op::STR << 12) | (sr << 9) | (baseR << 6) | off6
    end

    def res
      Op::RES << 12
    end

    def br(nzp, label)
      (Op::BR << 12) | (nzp.to_i << 9) | (label & 0x1_FF)
    end

    def jmp(baseR)
      (Op::JMP << 12) | (baseR << 6)
    end

    def ret
      jmp(7)
    end

    def jsr(off11)
      (Op::JSR << 12) | 1 << 11 | off11
    end

    def jsrr(baseR)
      (Op::JSR << 12) | (baseR << 6)
    end

    def lea(dr, off9)
      (Op::LEA << 12) | (dr << 9) | (off9 & 0b1_1111_1111)
    end

    def not(dr, sr)
      (Op::NOT << 12) | (dr << 9) | (sr << 6) | 0b1_11111
    end

    # traps
    def trap(trapv8)
      (Op::TRAP << 12) | trapv8
    end

    def getc
      trap(0x20_u16)
    end

    def out
      trap(0x21_u16)
    end

    def puts!
      trap(0x22)
    end
    def in
      trap(0x23)
    end
    def putsp
      trap(0x24)
    end
    def halt
      trap(0x25)
    end
  end
end