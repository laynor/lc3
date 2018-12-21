mod lc3;

fn main() {
    ncurses::initscr();

    let vm = lc3::LC3Vm::new();


    let foo = Ok(12);
    let bar = Some(12);
    match std::io::stdin()
        .bytes()
        .next()
        .and_then(|foo| foo.ok())
        .unwrap();

    ncurses::endwin();
}
