require "./spec_helper"
include Vm

describe Vm do
  # TODO: Write tests


  describe "lea" do
    it "should load the correct address" do
      vm = LC3Vm.new
      vm.@mem[0x3000] = ISA.lea(Reg::R0, 0)
      vm.r_pc = 0x3000_u16
      vm.step

      vm.@reg[0].should eq 0x3001_u16
      vm.r_cond.should eq Flg::POS.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "add" do
    it "1 + 2 = 3" do
      vm = LC3Vm.new
      vm.@reg[0] = 0
      vm.@reg[1] = 1
      vm.@reg[2] = 2
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.add(Reg::R0, Reg::R1, Reg::R2)
      vm.step

      vm.@reg[0].should eq 3
      vm.r_cond.should eq Flg::POS.to_u16
      vm.r_pc.should eq 0x3001
    end

    it "immediate: 1 + 2 = 3" do
      vm = LC3Vm.new
      vm.@reg[0] = 0
      vm.@reg[1] = 1
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.addi(Reg::R0, Reg::R1, 2)
      vm.step

      vm.@reg[0].should eq 3
      vm.r_cond.should eq Flg::POS.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "and" do
    it "0b0101_1111 & 0b0000_0101 " do
      vm = LC3Vm.new
      vm.@reg[0] = 0
      vm.@reg[1] = 0b0101_1111_u16
      vm.@reg[2] = 0b0000_0101_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.and(Reg::R0, Reg::R1, Reg::R2)
      vm.step

      vm.@reg[0].should eq 0b0101_u16
      vm.r_cond.should eq Flg::POS.to_u16
      vm.r_pc.should eq 0x3001
    end

    it "immediate 0b0101_1111 & 0b0000_0101" do
      vm = LC3Vm.new
      vm.@reg[0] = 0
      vm.@reg[1] = 0b0101_1111_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.andi(Reg::R0, Reg::R1, 0b0101_u16)
      vm.step

      vm.@reg[0].should eq 0b0101_u16
      vm.r_cond.should eq Flg::POS.to_u16
      vm.r_pc.should eq 0x3001
    end
  end
  describe "not" do
    it "NOT 0b0101_1010_1111_0000 = 0b1010_0101_0000_1111" do
      vm = LC3Vm.new
      vm.@reg[0] = 0
      vm.@reg[1] = 0b0101_1010_1111_0000_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.not(Reg::R0, Reg::R1)
      vm.step

      vm.@reg[0].should eq 0b1010_0101_0000_1111_u16
      vm.r_cond.should eq Flg::NEG.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "ld" do
    it "should load the value correctly" do
      vm = LC3Vm.new
      vm.@mem[0x3002] = 0xABCD_u16
      vm.@reg[0] = 0
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.ld(Reg::R0, 1)
      vm.step

      vm.@reg[0].should eq 0xABCD_u16
      vm.r_cond.should eq Flg::NEG.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "ldi" do
    it "should load the value correctly" do
      vm = LC3Vm.new
      vm.@mem[0x3002] = 0x3003_u16
      vm.@mem[0x3003] = 0xABCD_u16
      vm.@reg[0] = 0
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.ldi(Reg::R0, 1)
      vm.step

      vm.@reg[0].should eq 0xABCD_u16
      vm.r_cond.should eq Flg::NEG.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "ldr" do
    it "should load the value correctly" do
      vm = LC3Vm.new
      vm.@mem[0x3003] = 0xABCD_u16
      vm.@reg[0] = 0
      vm.@reg[1] = 0x3003_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.ldr(Reg::R0, Reg::R1, 0)
      vm.step

      vm.@reg[0].should eq 0xABCD_u16
      vm.r_cond.should eq Flg::NEG.to_u16
      vm.r_pc.should eq 0x3001
    end
  end

  describe "st" do
    it "should store the value correctly" do
      vm = LC3Vm.new
      vm.@mem[0x3003] = 0xABCD_u16
      vm.@reg[0] = 2_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.st(Reg::R0, 2)

      vm.@mem[0x3003].should eq 0xABCD_u16

      vm.step

      vm.@mem[0x3003].should eq 2_u16
      vm.r_pc.should eq 0x3001
    end
  end
  describe "sti" do
    it "should store the value correctly" do
      vm = LC3Vm.new
      val = 123_u16
      vm.@mem[0x3002] = 0x3003_u16
      vm.@mem[0x3003] = 0xABCD_u16
      vm.@reg[0] = val
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.sti(Reg::R0, 1)

      vm.@mem[0x3003].should eq 0xABCD_u16

      vm.step

      vm.@mem[0x3003].should eq val
      vm.r_pc.should eq 0x3001
    end
  end

  describe "str" do
    it "should store the value correctly" do
      vm = LC3Vm.new
      val = 123_u16
      vm.@mem[0x3003] = 0xABCD_u16
      vm.@reg[0] = val
      vm.@reg[1] = 0x3003_u16
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.str(Reg::R0, Reg::R1, 0)

      vm.@mem[0x3003].should eq 0xABCD_u16

      vm.step

      vm.@mem[0x3003].should eq val
      vm.r_pc.should eq 0x3001
    end
  end

  describe "br" do
    it "BRz should branch the right address" do
      vm = LC3Vm.new
      vm.@mem[0x4000] = 0xABCD_u16
      vm.@reg[0] = 0x4000
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.ld(Reg::R0, 1)
      vm.@mem[0x3000] = ISA.br(Reg::R0, 1)
      vm.step

      vm.@reg[0].should eq 0xABCD_u16
      vm.r_cond.should eq Flg::NEG.to_u16
      vm.r_pc.should eq 0x3001

      # TODO
    end
  end

  describe "jmp" do
    it "should jump to the correct location" do
      vm = LC3Vm.new
      vm.r_pc = 0x3000_u16
      vm.@reg[0] = 0x4000_u16
      vm.@mem[0x3000] = ISA.jmp(Reg::R0)

      vm.r_pc.should eq 0x3000_u16
      vm.step

      vm.r_pc.should eq 0x4000_u16
    end
  end

  describe "jsr" do
    it "should jump to the correct location" do
      vm = LC3Vm.new
      vm.r_pc = 0x3000_u16
      vm.@mem[0x3000] = ISA.jsr(0x100_u16)
      vm.@mem[0x3101] = ISA.jsr(0b0000_0111_1111_1110) # jump -2

      vm.r_pc.should eq 0x3000_u16
      vm.step

      vm.@reg[7].should eq 0x3001_u16
      vm.r_pc.should eq 0x3101_u16

      vm.step

      vm.@reg[7].should eq 0x3102_u16
      vm.r_pc.should eq 0x3100_u16
    end
  end

  describe "jsrr" do
    it "should jump to the correct location" do
      vm = LC3Vm.new
      vm.r_pc = 0x3000_u16
      vm.@reg[0] = 0x4000_u16
      vm.@mem[0x3000] = ISA.jsrr(Reg::R0)

      vm.r_pc.should eq 0x3000_u16
      vm.step

      vm.r_pc.should eq 0x4000_u16
      vm.@reg[7].should eq 0x3001_u16
    end
  end

end
