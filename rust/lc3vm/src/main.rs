#[macro_use]
extern crate nom;
extern crate ctrlc;

mod kbd;
mod lc3;
mod debugger;

use std::env;

fn main() {
    let mut vm = lc3::LC3Vm::new();

    let args = std::env::args();
    let filename = args.skip(1).next();

    let debug = vm.getdebug();

    match env::var("DBG") {
        Ok(_) => debug.store(true, std::sync::atomic::Ordering::SeqCst),
        Err(_) => (),
    }

    ctrlc::set_handler(move || {
        println!("debug");
        debug.store(true, std::sync::atomic::Ordering::SeqCst);
    }).expect("Error setting ctrl+c handler");


    match filename {
        None => {
            println!("USAGE: lc3vm imagefile.obj");
        },
        Some(fname) => {
            vm.load_image_file(&fname);
            vm.run();
        }
    }
}
