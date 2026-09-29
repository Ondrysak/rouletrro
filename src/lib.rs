//! Digitakt mk1 (OS 1.53) emulator: a ColdFire MCF5441x machine in Rust.

pub mod firmware;
pub mod bus;
pub mod cpu;
pub mod io;
pub mod edma;
pub mod symbols;
pub mod machine;
pub mod panel;
pub mod esdhc;
