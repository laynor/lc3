// Parser for debugger commands

// st              print PC and COND
// p R[0-7]        direct print
// p x[addr]
// p @R[0-7]       indirect print
// p @x[addr]

// b x[addr]       set breakpoint at addr
// lb              list breakpoints

// l               list around pc
// l x[addr]       list around addr
// l x[from] x[to] list from addr to addr

// n               next

use std::fmt;
use std::str::FromStr;

named!(newline<&str, &str>,
       tag!("\n"));


named!(space1<&str, char>,
       one_of!(" \t"));

named!(space<&str, &str>,
       recognize!(many1!(space1)));

named!(toeol<&str, &str>,
       recognize!(tuple!(many0!(space1), newline)));

#[macro_export]
macro_rules! sslist(
    ($i:expr, $submac:ident!( $($args:tt)* )) => (
        many0!($i, preceded!(space, $submac!($($args)*)));
    );
    ($i:expr, $f:expr) => (
        sslist!!($i, call!($f));
    );
);


#[derive(Debug)]
pub enum Item {
    Reg(u8),
    Addr(u16),
}

impl fmt::Display for Item {
    fn fmt(&self, fmt:&mut fmt::Formatter) -> fmt::Result {
        match self {
            Item::Reg(n) => write!(fmt, "R{}", n),
            Item::Addr(a) => write!(fmt, "x{}", a)
        }
    }
}

#[derive(Debug)]
pub enum Command {
    Print(bool, Item),
    PrintRegisters,
    Break(u16),
    ListBreak,
    List(Option<Item>, Option<Item>),
    Next,
    Status,
    Continue,
    Quit,
}


fn from_hex(input: &str) -> Result<u16, std::num::ParseIntError> {
    u16::from_str_radix(input, 16)
}

fn is_hex_digit(c: char) -> bool {
    c.is_digit(16)
}

named!(hex_primary<&str, u16>,
    map_res!(take_while_m_n!(1, 4, is_hex_digit), from_hex)
);

// Register parser (Rx)
named!(regx<&str, Item>,
       do_parse!(tag!("R") >>
                 reg: one_of!("01234567") >>
                 (Item::Reg(u8::from_str(&reg.to_string()).unwrap()))
       )
);

// Address parser (hex - xXXXX)
named!(addr<&str, Item>,
       do_parse!(tag!("x") >>
                 n: hex_primary >>
                 (Item::Addr(n))
       )
);


// p R[0-7]        direct print
// p x[addr]
// p @R[0-7]       indirect print
// p @x[addr]

named!(print_cmd<&str, Command>,
       do_parse!(
           one_of!("pP") >>
               is_a!(" \t") >>
               a: opt!(char!('@')) >>
               it: alt!(regx | addr) >>
               (Command::Print(a != None, it))
       )
);

named!(print_regs_cmd<&str, Command>,
       do_parse!(
           tag!("pr") >>
               toeol >>
       (Command::PrintRegisters)));

// l               list around pc
// l x[addr]       list around addr
// l x[from] x[to] list from addr to addr
named!(pub list_cmd<&str, Command>,
       do_parse!(
           one_of!("lL") >>
           res: sslist!(alt!(addr | regx)) >>
               (match &res[..] {
                   [] => Command::List(None, None),
                   [Item::Addr(a)] => Command::List(Some(Item::Addr(*a)), None),
                   [Item::Addr(a1), Item::Addr(a2)] => Command::List(Some(Item::Addr(*a1)), Some(Item::Addr(*a2))),
                   _ => Command::List(None, None)
               }))
);


// st              print PC and COND
named!(st_cmd<&str, Command>,
       do_parse!(tag!("st") >>
                 (Command::Status)
       ));

// lb              list breakpoints
named!(lb_cmd<&str, Command>,
       do_parse!(tag!("lb") >>
                 (Command::ListBreak)
       ));

// b x[addr]       set breakpoint at addr
named!(break_cmd<&str, Command>,
       do_parse!(
           one_of!("bB") >>
               is_a!(" \t") >>
               a: addr >>
               (match a {
                   Item::Addr(a) => Command::Break(a),
                   _ => panic!("Wrong break command")

               })

       ));


// n               next
named!(next_cmd<&str, Command>,
       do_parse!(
           tag!("n") >>
               (Command::Next)
       ));

// q               quit
named!(quit_cmd<&str, Command>,
       do_parse!(
           one_of!("qQ") >>
               toeol >>
               (Command::Quit)
       ));

// c               continue
named!(continue_cmd<&str, Command>,
       do_parse!(
           tag!("c") >>
               toeol >>
               (Command::Continue)
       ));


named!(pub cmd<&str, Command>,
       do_parse!(
           cmd: alt!(print_cmd | lb_cmd | list_cmd | st_cmd | break_cmd | next_cmd | quit_cmd | continue_cmd | print_regs_cmd) >>
               (cmd)
       ));
