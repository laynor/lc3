# TODO: Write documentation for `Sandbox`
require "./vm"
require "./isa"
include Vm
include Vm::ISA
module Foo
  # instructions to test:
  # BR
  # ADD
  # AND
  # NOT
  # LD
  # LDI
  # LDR
  # LEA
  # JMP
  # JSR
  # ST
  # STI
  # STR
  # RTI (unused)
  # RES (unused)
  # TRAP

  R0 = 0
  R1 = 1
  R2 = 2
  R3 = 3
  R4 = 4
  R5 = 5
  R6 = 6
  R7 = 7

  Program = [
    (lea R5, 2),  # load data base address
    (lea R1, 22), # load jump address
    (jmp R1),     # jump data
    0_u16,
    27_u16,
    '['.ord.to_u16,
    '3'.ord.to_u16,
    '1'.ord.to_u16,
    'm'.ord.to_u16,
    'A'.ord.to_u16,
    'n'.ord.to_u16,
    't'.ord.to_u16,
    'a'.ord.to_u16,
    'n'.ord.to_u16,
    'i'.ord.to_u16,
    27_u16,
    '['.ord.to_u16,
    '3'.ord.to_u16,
    '9'.ord.to_u16,
    ';'.ord.to_u16,
    '4'.ord.to_u16,
    '9'.ord.to_u16,
    'm'.ord.to_u16,
    0_u16,
    (add R0, R5, 1), # load base string address into R0
    puts!,
    getc,
    (ld R0, 1), # load next next word into R0
    (jsr 1),
    0xFE00_u16, # keyboard status register address
    res,
    (lea R2, 0), 
    (ldr R3, R0, 0), # kbd loop start
    (br 0b010, -2), # previous instruction is -2! (PC already points to next instruction)
    (ldr R4, R0, 1), # kbd data register
    res,
    halt,
  ]

end 


module LC3Vm
  include Vm
  VERSION = "0.1.0"
  vm = LC3Vm.new
  # vm.load(Foo::Program)
  puts "loading #{ARGV[0]}"
  gets
  vm.read_image_file(ARGV[0])
  # vm.loglevel = LogLevel::Error
  # vm.debug = true
  vm.run
end
