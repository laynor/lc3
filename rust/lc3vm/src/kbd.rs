
extern crate nix;
use nix::sys::termios;
extern crate rustyline;


// buffer input using a channel
fn with_terminal<T>(f:impl FnOnce()->T) -> T {
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

    let res = f();

    termios::tcsetattr(0, termios::SetArg::TCSADRAIN, &orig_term).unwrap();

    res
}

pub fn getch() -> u8 {
    use std::io::Read;
    with_terminal(|| {
        std::io::stdin().bytes().next().unwrap().ok().unwrap()
    })
}

pub fn check_input() -> bool {
    with_terminal(|| {
        use nix::sys::select;
        use nix::sys::time;
        use nix::sys::time::TimeValLike;
        let mut timeout = time::TimeVal::milliseconds(0);
        let mut readfds = select::FdSet::new();
        readfds.insert(0);
        let res = select::select(None, Some(&mut readfds), None, None, Some(&mut timeout));
        match res.ok() {
            None    => false,
            Some(0) => false,
            Some(_) => true,
        }
    })
}
