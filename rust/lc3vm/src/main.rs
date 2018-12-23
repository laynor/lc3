extern crate ctrlc;

mod lc3;

fn main() {
    let mut vm = lc3::LC3Vm::new();

    let mut args = std::env::args();
    let filename = args.skip(1).next();
    let mut foo = false;
    let c = std::cell::Cell::new(false);


    ctrlc::set_handler( move || {
        c.set(true);
    });

    match filename {
        None => {
            println!("USAGE: lc3vm imagefile.obj");
        },
        Some(fname) => {
            vm.load_image_file(&fname);
            vm.run();
        }
    }
    // ncurses::endwin();
}
