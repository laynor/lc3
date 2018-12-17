# antani
require "./utils"
require "./types"

module Vm
  MEM_SIZE = UInt16::MAX.to_i + 1
  TRAPV_START = 0x0000_u16
  INTV_START  = 0x0100_u16
  ESR_START   = 0x0100_u16
  ISR_START   = 0x0180_u16
  OS_START    = 0x0200_u16
  PC_START    = 0x3000_u16
  MMIO_START  = 0xFE00_u16
  MMIO_KBSR   = MMIO_START
  MMIO_KBDR   = MMIO_KBSR + 1

  class LC3Vm
    include Utils
    property debug    : Bool
    property loglevel : LogLevel
    property r_pc     : UInt16
    property r_psr    : UInt16
    property r_cond   : UInt16

    def initialize
      @mem      = Array(UInt16).new(MEM_SIZE, 0_u16)
      @loglevel = LogLevel::Info
      @debug    = false
      @running  = false

      # registers 
      @reg      = Array(UInt16).new(8, 0_u16)
      @r_pc     = 0_u16
      @r_psr    = 0_u16
      @r_cond   = 0_u16
    end

    # utilities
    def update_flags(reg)
      val  = @reg[reg]
      flag = case 
        when val == 0
          Flg::ZRO
        when bit?(val, 15)
          Flg::NEG
        else
          Flg::POS
        end
      @r_cond = flag.to_u16
    end 

    def log(lev, msg)
      puts "[#{lev}] #{msg}" if lev >= @loglevel
    end


    ## instructions

    def res(args)
      print_registers
    end

    # Riduzione del codice per equivalenze
    # da codice astratto a codice diretto

    # Math/Logic

    def add(args)
      # instruction format:
      # opc dr sr1 0 00 sr2
      # opc dr sr1 1 imm5

      dr, sr1, mode, imm5 = bitsplit(args, [3, 3, 1, 5])

      @reg[dr] = if mode == 1
        # sign extend the last 5 bits
        @reg[sr1] + sign_extend(imm5, 5)
      else
        _, sr2 = bitsplit(imm5, [2, 3])
        @reg[sr1] + @reg[sr2]
      end

      update_flags(dr.to_i)
    end

    def and(args : UInt16)
      # instruction format:
      # opc dr sr1 0 00 sr2
      # opc dr sr1 1 imm5

      dr, sr1, mode, imm5 = bitsplit(args, [3, 3, 1, 5])

      @reg[dr] = if mode == 1
        # sign extend the last 5 bits
        @reg[sr1] & sign_extend(imm5, 5)
      else
        _, sr2 = bitsplit(imm5, [2, 3])
        @reg[sr1] & @reg[sr2]
      end

      update_flags(dr)
    end

    def not(args)
      dr, sr, _ = bitsplit(args, [3, 3, 6])
      @reg[dr] = ~@reg[sr]
      update_flags(dr)
    end

    # Load/Store
    def ld(args)
      # 0010 DR LABEL
      dr, off9 = bitsplit(args, [3, 9])
      @reg[dr] = mem_read(r_pc + sign_extend(off9, 9))
      update_flags(dr)
    end

    def ldi(args)
      # iwuction format 
      # opc(4) dr(3) pcoffset(9)
      dr, off9 = bitsplit(args, [3, 9])
      # load the address from MEM[PC + OFFSET]
      addr = mem_read(r_pc + sign_extend(off9, 9))
      # load the calue from the calculated address
      @reg[dr] = mem_read(addr)
      update_flags(dr)
    end

    def ldr(args)
      dr, baseR, off6 = bitsplit(args, [3, 3, 6])     
      @reg[dr] = mem_read(@reg[baseR] + sign_extend(off6, 6))
      update_flags(dr)
    end

    def lea(args)
      dr, off9 = bitsplit(args, [3, 9])
      @reg[dr] = r_pc + sign_extend(off9, 9)
      update_flags(dr)
    end

    def st(args)
      # iwuction format
      # opc(4) sr(3) pcoffset(9)
      sr, off9 = bitsplit(args, [4, 3, 9])
      mem_store(r_pc + sign_extend(off9, 9), @reg[sr])
    end 

    def sti(args)
      # iwuction format 
      # opc(4) dr(3) pcoffset(9)
      sr, off9 = bitsplit(args, [3, 9])
      # load the address from MEM[PC + OFFSET]
      addr = mem_read(r_pc + sign_extend(off9, 9))
      # load the calue from the calculated address
      mem_store(addr, @reg[sr])
    end

    def str(args)
      sr, baseR, off6 = bitsplit(args, [3, 3, 6])
      mem_store(@reg[baseR] + sign_extend(off6, 6), 
      @reg[sr])
    end

    # Branching/Jumping

    def br(args)
      nzp, off9 = bitsplit(args, [3, 9])
      if (r_cond & nzp) != 0
        @r_pc += sign_extend(off9, 9)
      end
    end

    def jmp(args)
      _, baseR, _ = bitsplit(args, [3, 3, 6])
      @r_pc = @reg[baseR]
    end

    def ret(args)
      jmp(args)
    end

    def rti(args)
      if !bit?(r_psr, 15)
        @r_pc    = mem_read(@reg[6])
        @reg[6] += 1
        tmp = mem_read(@reg[6])
        @reg[6] += 1
        @r_psr   = tmp
        # todo: the interrupted process are restored
      else
        # todo: privilege mode exception
      end 
    end

    def jsr(args)
      mode, off11 = bitsplit(args, [1, 11])
      _, baseR, _ = bitsplit(off11, [2, 3, 6])
      if mode == 1
        @r_pc += sign_extend(off11, 11)
      else
        @r_pc = @reg[baseR]
      end
    end

    def trap(args)
      _, trapv = bitsplit(args, [4, 8])
      # real assembler implementation
      # @reg[7] = r_pc
      # @r_pc = @mem[trapv]

      # VM implementation
      case Trap.new(trapv.to_i)
      when Trap::GETC
        trap_getc
      when Trap::PUTS
        trap_puts
      when Trap::IN
        trap_in
      when Trap::PUTSP
        trap_putsp
      when Trap::HALT
        trap_halt
      end
    end

    # traps

    def trap_getc
      res = STDIN.noecho { |si| si.raw &.read_char }
      unless res.nil?
        @reg[0] = res.ord.to_u16
      end
    end

    def trap_puts
      # fixme
      # endianess?
      i = @reg[0]
      buff = Bytes.new(1)
      while @mem[i] != 0
        buff[0] = @mem[i].to_u8
        STDOUT.write(buff)
        i += 1
      end
      STDOUT.flush
    end

    def trap_in
      puts "Enter a character"
      trap_getc
    end

    def trap_putsp
      i = @reg[0]
      buff = Bytes.new(1)
      while @mem[i] != 0
        buff[0] = (@mem[i] & 0xFF_u16).to_u8
        STDOUT.write(buff)
        buff[0] = (@mem[i] >> 8).to_u8
        STDOUT.write(buff) if buff[0] != 0
        i += 1
      end
      STDOUT.flush
    end 

    def trap_halt
      @running = false
    end
    # engine

    def mem_store(addr, val)
      @mem[addr] = val
    end

    def mem_read(addr : UInt16)
      
      if addr == MMIO_KBSR
        if stdin_empty?
          @mem[MMIO_KBSR] = 0
        else
          @mem[MMIO_KBSR] = 1_u16 << 15
          char = STDIN.raw {|si| si.noecho &.read_char}
          if !char.nil?
            @mem[MMIO_KBDR] = char.ord.to_u16
          else
            raise "Nil char returned!"
          end
        end
      end 
      @mem[addr.to_i]
    end 

    def interpret(iw : UInt16)

      opcode, args = bitsplit(iw, [4, 12])
      op = Op.new(opcode.to_i)
      case op
        # arithmetic and logic ops
      when Op::ADD
        add(args)

      when Op::AND
        and(args)

      when Op::NOT
        not(args)

        # load

      when Op::LD
        ld(args)

      when Op::LDI
        ldi(args)

      when Op::LDR
        ldr(args)

      when Op::LEA
        lea(args)

        # store
      when Op::ST
        st(args)

      when Op::STI
        sti(args)

      when Op::STR
        str(args)

        # Jump/Branching
      when Op::BR
        br(args)

      when Op::JMP
        jmp(args)

      when Op::JSR
        jsr(args)

      when Op::RTI
        rti(args)

      when Op::TRAP
        trap(args)

      when Op::RES
        res(args)
      else
        puts "Unknown opcode #{opcode} #{Op::ADD.value}"
      end
    end

    def load(program)
      prg_start = PC_START
      prg_len = program.size
      prg_end = prg_start + prg_len
      @mem[prg_start..prg_end] = program
    end

    def read_image_file(path)
      File.open(path) do |f|
        i = PC_START

        while true
          begin
            @mem[i] = f.read_bytes(UInt16, IO::ByteFormat::BigEndian)
            i += 1
          rescue
            break
          end
        end

      end
    end

    def step
      iw = mem_read r_pc
      log LogLevel::Info, "Instr: %016b - %s" % [iw, ISA.decode(iw)]
      gets if @debug
      @r_pc += 1

      interpret iw
    end

    def print_mem(from=0, to=65535, fmt="%016b")
      (from..to).each do |i| 
        word = @mem[i]
        opc, _ = bitsplit(word, [4, 12])
        c1, c2 = bitsplit(word, [8, 8])
        c1 = (c1 != 0) ? c1.chr : ' '
        c2 = (c2 != 0) ? c2.chr : ' '
        
        log LogLevel::Info, "#{fmt % word} Op: #{Op.new(opc.to_i)} #{c1}#{c2}"
      end
    end

    def run
      log LogLevel::Info, "Crystal LC-3 VM"
      log LogLevel::Info, "Mem: #{MEM_SIZE} words"
      log LogLevel::Info, " "

      print_mem(PC_START, PC_START+32)
      puts

      @r_pc = PC_START
      puts "PC: #{r_pc} #{"0x%04x" % PC_START}"
      @running = true

      while @running
        step
      end
    end

    def print_registers
      log LogLevel::Info, "-" * 80
      (0..3).each do |i|
        rl = Reg.new(i).to_s
        rr = Reg.new(4 + i).to_s
        log LogLevel::Info, "#{rl}: %04x #{rr}: %04x" % [ @reg[i], @reg[4+i] ]
      end
      log LogLevel::Info, "" 
      log LogLevel::Info, "PC: %04x %016b COND: %03b %s" % [r_pc, r_pc, r_cond, "#{Flg.new(r_cond.to_i)}"]
      log LogLevel::Info, "-" * 80
      log LogLevel::Info, ""
    end
  end

end