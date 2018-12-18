module Vm
  enum LogLevel
    Trace
    Debug
    Info
    Warning
    Error
    Fatal
  end

  enum Reg
    R0 = 0
    R1
    R2
    R3
    R4
    R5
    R6
    R7
    PC
    COND
    Count
    def <<(n)
      self.value.to_u16 << n
    end
  end 

  enum Op
    BR = 0
    ADD    # add  
    LD     # load 
    ST     # store
    JSR    # jump register 
    AND    # bitwise and 
    LDR    # load register 
    STR    # store register 
    RTI    # unused
    NOT    # bitwise not
    LDI    # load indirect
    STI    # store indirect
    JMP    # jump
    RES    # reserved (unused)
    LEA    # load effective address
    TRAP
    def <<(n)
      self.value.to_u16 << n
    end
  end

  enum Trap
    GETC  = 0x20
    OUT   = 0x21
    PUTS  = 0x22
    IN    = 0x23
    PUTSP = 0x24
    HALT  = 0x25
  end 

  enum Res
    PRNREG
    DEBUG
    LOGLEV
    def <<(n)
      self.value.to_u16 << n
    end
  end

  @[Flags]
  enum Flg
    POS = 1
    ZRO = 1 << 1
    NEG = 1 << 2
  end 
end