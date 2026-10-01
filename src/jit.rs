//! A block compiler: hot predecoded blocks become native code (Cranelift).
//!
//! A compiled block leaves the machine exactly as `Cpu::run_ops` would --
//! registers, condition codes, memory, the clock, `pc` -- bit for bit, and
//! `jitcheck` holds it to that. The common operations (`fast::Kind`) are
//! translated:
//!
//! * guest registers live in Cranelift variables, loaded on first use and
//!   stored back only before a call out and at the block's end;
//! * condition codes are computed only where something can read them (a
//!   backward pass over the block), and a branch after a compare or a
//!   logical result tests the operands directly;
//! * DDR loads and stores are inline; anything else (SRAM, peripherals,
//!   stores into cached code) goes through `Cpu::rd` / `Cpu::wr`.
//!
//! Every other operation is a call to its `fast` handler, with the
//! interpreter's bookkeeping around it (`op_pc`, the clock, `pc` preset)
//! and the cached registers written back before and reloaded after.
//!
//! The code refers to its block's `Op`s and to DDR by address, so it lives
//! exactly as long as the block cache does: a flush (`Bus::icache_settle`)
//! drops every block, and the next block compiled starts a fresh module.

use crate::cpu::Cpu;
use crate::fast::{Ea, Kind, Op};
use cranelift_codegen::ir::condcodes::IntCC;
use cranelift_codegen::ir::{types, AbiParam, InstBuilder, MemFlagsData, SigRef, Signature, Type, UserFuncName, Value};
use cranelift_codegen::settings::{self, Configurable};
use cranelift_codegen::Context;
use cranelift_frontend::{FunctionBuilder, FunctionBuilderContext, Variable};
use cranelift_jit::{JITBuilder, JITModule};
use cranelift_module::Module;
use std::mem::offset_of;

/// A compiled block.
pub type BlockFn = unsafe extern "C" fn(*mut Cpu);

/// Runs of a block before it is compiled.
pub const HOT: u32 = 256;

pub struct Jit {
    module: Option<JITModule>,
    ctx: Context,
    fctx: FunctionBuilderContext,
    /// `Bus::icache_flushes` when the module was started.
    pub epoch: u64,
    pub compiled: u64,
    pub failed: u64,
    /// Time spent compiling, in seconds.
    pub compile_secs: f64,
    /// Translate operations (false: every op is a handler call).
    pub translate: bool,
}

fn new_module() -> JITModule {
    let mut flags = settings::builder();
    // DTEMU_JIT_OPT / DTEMU_JIT_RA: Cranelift's opt_level and
    // regalloc_algorithm, for weighing compile time against code speed.
    // "none" makes code as fast as "speed" here, and compiles faster.
    let opt = std::env::var("DTEMU_JIT_OPT").unwrap_or_else(|_| "none".into());
    flags.set("opt_level", &opt).unwrap();
    if let Ok(ra) = std::env::var("DTEMU_JIT_RA") {
        flags.set("regalloc_algorithm", &ra).unwrap();
    }
    flags.set("enable_verifier", "false").unwrap();
    flags.set("use_colocated_libcalls", "false").unwrap();
    flags.set("is_pic", "false").unwrap();
    let isa = cranelift_native::builder()
        .expect("host machine not supported by Cranelift")
        .finish(settings::Flags::new(flags))
        .unwrap();
    JITModule::new(JITBuilder::with_isa(isa, cranelift_module::default_libcall_names()))
}

/// Memory reads and writes compiled code cannot do inline.
extern "C" fn jit_rd(c: &mut Cpu, a: u32, sz: u32) -> u32 {
    c.rd(a, sz)
}

extern "C" fn jit_wr(c: &mut Cpu, a: u32, sz: u32, v: u32) {
    c.wr(a, sz, v)
}

/// Run a block on the interpreter (a compiled block whose EMAC mode is not
/// the one it was compiled for).
extern "C" fn jit_interp(c: &mut Cpu, ops: *const Op, n: u32, pc: u32) {
    // SAFETY: the block's own ops, cached while its code exists.
    let ops = unsafe { std::slice::from_raw_parts(ops, n as usize) };
    c.run_ops_slice(pc, ops);
}

/// An accumulator as MOVE.L ACC,Rx reads it, in any mode.
extern "C" fn jit_mac_read(c: &mut Cpu, i: u32, _: u32) -> u32 {
    c.mac_read(i as usize)
}

/// What compiled code needs from the machine: the page tables for inline
/// memory accesses (`Bus::jit_pages`).
#[derive(Clone, Copy)]
pub struct Mem {
    pub rd_pages: *const usize,
    pub wr_pages: *const usize,
    /// MACSR when the block is compiled: EMAC blocks are specialized on
    /// its OMC bit (and check it on entry).
    pub macsr: u32,
}

impl Mem {
    pub fn of(bus: &mut crate::bus::Bus) -> Mem {
        let (rd_pages, wr_pages) = bus.jit_pages();
        Mem { rd_pages, wr_pages, macsr: 0 }
    }
}

impl Jit {
    pub fn new(epoch: u64) -> Jit {
        let module = new_module();
        let ctx = module.make_context();
        Jit {
            module: Some(module),
            ctx,
            fctx: FunctionBuilderContext::new(),
            epoch,
            compiled: 0,
            failed: 0,
            compile_secs: 0.0,
            translate: std::env::var_os("DTEMU_JIT_CALLS").is_none(),
        }
    }

    /// Drop all compiled code (the blocks it was built from are gone).
    pub fn reset(&mut self, epoch: u64) {
        if let Some(m) = self.module.take() {
            // SAFETY: called only after the block cache dropped every
            // block, and with them every pointer to this code.
            unsafe { m.free_memory() };
        }
        self.module = Some(new_module());
        self.epoch = epoch;
    }

    /// Compile the block `ops`, which starts at `pc`.
    pub fn compile(&mut self, pc: u32, ops: &[Op], mem: Mem) -> Option<BlockFn> {
        let t = std::time::Instant::now();
        let f = self.build(pc, ops, mem);
        self.compile_secs += t.elapsed().as_secs_f64();
        match f {
            Some(f) => {
                self.compiled += 1;
                Some(f)
            }
            None => {
                self.failed += 1;
                None
            }
        }
    }

    fn build(&mut self, pc: u32, ops: &[Op], mem: Mem) -> Option<BlockFn> {
        let module = self.module.as_mut()?;
        let ptr = module.target_config().pointer_type();
        let call_conv = module.target_config().default_call_conv;
        let tc = module.target_config();
        let mut sig = Signature::new(call_conv);
        sig.params.push(AbiParam::new(ptr));
        self.ctx.func.signature = sig.clone();
        let id = module.declare_anonymous_function(&sig).ok()?;
        self.ctx.func.name = UserFuncName::user(0, id.as_u32());

        let mut hsig = Signature::new(call_conv);
        hsig.params.push(AbiParam::new(ptr));
        hsig.params.push(AbiParam::new(ptr));
        let mut rsig = Signature::new(call_conv);
        rsig.params.extend([AbiParam::new(ptr), AbiParam::new(types::I32), AbiParam::new(types::I32)]);
        rsig.returns.push(AbiParam::new(types::I32));
        let mut wsig = Signature::new(call_conv);
        wsig.params.extend([
            AbiParam::new(ptr),
            AbiParam::new(types::I32),
            AbiParam::new(types::I32),
            AbiParam::new(types::I32),
        ]);
        let mut isig = Signature::new(call_conv);
        isig.params.extend([AbiParam::new(ptr), AbiParam::new(ptr), AbiParam::new(types::I32), AbiParam::new(types::I32)]);

        {
            let mut b = FunctionBuilder::new(&mut self.ctx.func, &mut self.fctx);
            let hsig = b.import_signature(hsig);
            let rsig = b.import_signature(rsig);
            let wsig = b.import_signature(wsig);
            let isig = b.import_signature(isig);
            let entry = b.create_block();
            b.append_block_params_for_function_params(entry);
            b.switch_to_block(entry);
            let cpu = b.block_params(entry)[0];
            let mut t = Tx::new(b, cpu, ptr, [hsig, rsig, wsig, isig], mem);
            t.block(pc, ops, self.translate);
            t.b.seal_all_blocks();
            t.b.finalize(tc);
        }
        // DTEMU_JIT_DUMP=<hex pc>: print that block's IR and machine code.
        let dump = std::env::var("DTEMU_JIT_DUMP").ok().and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok()) == Some(pc);
        if dump {
            eprintln!("{}", self.ctx.func.display());
            self.ctx.set_disasm(true);
        }
        let ok = module.define_function(id, &mut self.ctx).is_ok();
        if dump {
            if let Some(v) = self.ctx.compiled_code().and_then(|c| c.vcode.clone()) {
                eprintln!("{v}");
            }
            self.ctx.set_disasm(false);
        }
        module.clear_context(&mut self.ctx);
        if !ok {
            return None;
        }
        module.finalize_definitions().ok()?;
        let code = module.get_finalized_function(id);
        // SAFETY: the function was built with BlockFn's signature.
        Some(unsafe { std::mem::transmute::<*const u8, BlockFn>(code) })
    }
}

/// Compiles blocks on a background thread, so a burst of newly hot code
/// never stalls the emulator: a block runs interpreted until its code
/// arrives. The thread owns its compiler and a copy of each block's ops
/// (the code points at them), both kept until a request from a newer
/// cache epoch shows the main thread has dropped the old blocks.
pub struct Worker {
    tx: std::sync::mpsc::Sender<Req>,
    rx: std::sync::mpsc::Receiver<Done>,
    pub stats: std::sync::Arc<WorkerStats>,
}

#[derive(Default)]
pub struct WorkerStats {
    pub compiled: std::sync::atomic::AtomicU64,
    pub failed: std::sync::atomic::AtomicU64,
    pub nanos: std::sync::atomic::AtomicU64,
}

struct Req {
    idx: usize,
    epoch: u64,
    pc: u32,
    ops: Box<[Op]>,
    mem: Mem,
}

/// A compiled block, for block `idx` of cache epoch `epoch`.
pub struct Done {
    pub idx: usize,
    pub epoch: u64,
    pub f: Option<BlockFn>,
}

// SAFETY: Mem's pointers are only dereferenced by compiled code the main
// thread runs; the worker passes them through as constants.
unsafe impl Send for Req {}

impl Default for Worker {
    fn default() -> Self {
        Self::new()
    }
}

impl Worker {
    pub fn new() -> Worker {
        use std::sync::atomic::Ordering::Relaxed;
        let (tx, rx_req) = std::sync::mpsc::channel::<Req>();
        let (tx_done, rx) = std::sync::mpsc::channel::<Done>();
        let stats = std::sync::Arc::new(WorkerStats::default());
        let st = stats.clone();
        std::thread::Builder::new()
            .name("jit".into())
            .spawn(move || {
                let mut jit = Jit::new(0);
                let mut keep: Vec<Box<[Op]>> = Vec::new();
                for r in rx_req {
                    if r.epoch < jit.epoch {
                        continue;
                    }
                    if r.epoch > jit.epoch {
                        jit.reset(r.epoch);
                        keep.clear();
                    }
                    let t = std::time::Instant::now();
                    let f = jit.compile(r.pc, &r.ops, r.mem);
                    st.nanos.fetch_add(t.elapsed().as_nanos() as u64, Relaxed);
                    if f.is_some() {
                        st.compiled.fetch_add(1, Relaxed);
                    } else {
                        st.failed.fetch_add(1, Relaxed);
                    }
                    keep.push(r.ops);
                    if tx_done.send(Done { idx: r.idx, epoch: r.epoch, f }).is_err() {
                        break;
                    }
                }
            })
            .expect("spawn the jit thread");
        Worker { tx, rx, stats }
    }

    /// Ask for block `idx` (cache epoch `epoch`) to be compiled.
    pub fn queue(&self, idx: usize, epoch: u64, pc: u32, ops: &[Op], mem: Mem) {
        let _ = self.tx.send(Req { idx, epoch, pc, ops: ops.into(), mem });
    }

    /// Blocks compiled since the last call.
    pub fn poll(&self) -> std::sync::mpsc::TryIter<'_, Done> {
        self.rx.try_iter()
    }
}

/// A cached value's state against the Cpu struct.
#[derive(Clone, Copy, PartialEq)]
enum St {
    Unloaded,
    Clean,
    Dirty,
}

/// The flag-setting op whose condition codes are not in SR yet (lazy
/// flags): its operands are in variables `fs` `fd` `fr`, and SR's N Z V C
/// (and X where it sets X) are worked out only when something reads them.
#[derive(Clone, Copy, PartialEq)]
enum Pend {
    None,
    /// N Z of fr (masked to size), V = C = 0; X untouched.
    Logic(u8),
    /// fd + fs at size (add_flags): X = C.
    Add(u8),
    /// fd - fs at size (sub_flags): X = C, or kept (CMP: `true`).
    Sub(u8, bool),
}

impl Pend {
    fn writes_x(self) -> bool {
        matches!(self, Pend::Add(_) | Pend::Sub(_, false))
    }
}

const CF_C: i64 = 0x01;
const CF_V: i64 = 0x02;
const CF_Z: i64 = 0x04;
const CF_N: i64 = 0x08;
const CF_X: i64 = 0x10;

fn mask(sz: u8) -> i64 {
    match sz {
        1 => 0xFF,
        2 => 0xFFFF,
        _ => 0xFFFF_FFFF,
    }
}

fn bits(sz: u8) -> i64 {
    sz as i64 * 8
}

/// Does `Tx::op` translate this op (rather than call its handler)?
fn inline_ok(o: &Op) -> bool {
    match o.k {
        Kind::Call | Kind::Bcc | Kind::Bra => false,
        Kind::Move { .. } | Kind::Clr { .. } => !matches!(o.b, Ea::A(_) | Ea::Imm(_)),
        Kind::AluImm { .. } => !matches!(o.b, Ea::A(_) | Ea::Imm(_)),
        _ => true,
    }
}

/// Which condition codes each op must leave correct: (N Z V C, X).
fn flag_liveness(ops: &[Op]) -> Vec<(bool, bool)> {
    let mut out = vec![(true, true); ops.len()];
    let (mut nzvc, mut x) = (true, true); // live out of the block
    for (i, op) in ops.iter().enumerate().rev() {
        out[i] = (nzvc, x);
        match op.k {
            Kind::Call => {
                nzvc = true;
                x = true;
            }
            Kind::Move { .. }
            | Kind::MoveQ
            | Kind::Mvs { .. }
            | Kind::Mvz { .. }
            | Kind::Clr { .. }
            | Kind::Tst { .. }
            | Kind::CmpiDn { .. } => nzvc = false,
            Kind::AluEaDn { op, .. } | Kind::AluImm { op, .. } => {
                nzvc = false;
                if op <= 1 {
                    x = false;
                }
            }
            Kind::AluAn { op, .. } => {
                if op == 5 {
                    nzvc = false;
                }
            }
            Kind::Bcc => nzvc = true,
            Kind::Shift { reg, .. } => {
                nzvc = false;
                // A zero count (only from a register) keeps X.
                if !reg {
                    x = false;
                }
            }
            Kind::Addx { .. } => {
                // Z is kept when the result is zero; X is an input.
                nzvc = true;
                x = true;
            }
            // Reads V, writes N Z V C.
            Kind::Sats => nzvc = true,
            Kind::Swap | Kind::EorDn { .. } => nzvc = false,
            // Sets Z, keeps N V C: reads them.
            Kind::BitDn { .. } => nzvc = true,
            Kind::Movem => {}
            Kind::MoveA { .. } | Kind::Lea | Kind::QuickAn | Kind::Bra | Kind::Mac { .. } | Kind::MovAcc => {}
        }
    }
    out
}

struct Tx<'a> {
    b: FunctionBuilder<'a>,
    cpu: Value,
    ptr: Type,
    hsig: SigRef,
    rsig: SigRef,
    wsig: SigRef,
    isig: SigRef,
    mem: Mem,
    regs: [Variable; 16],
    reg_st: [St; 16],
    sr: Variable,
    sr_st: St,
    /// The clock at the start of the current pass through the ops.
    now: Variable,
    pend: Pend,
    fs: Variable,
    fd: Variable,
    fr: Variable,
    /// EMAC state cached in variables (blocks without calls, entered
    /// in fractional mode): MACSR and the accumulators.
    mac_cached: bool,
    /// The OMC bit the EMAC code is specialized on (with `mac_cached`).
    omc: bool,
    macsr: Variable,
    macsr_st: St,
    accs: [Variable; 4],
    acc_st: [St; 4],
    /// The last accumulator result that set MACSR's N Z V EV, and that
    /// accumulator's PAV bit (0: none since the block began), so the bits
    /// are worked out once, when MACSR is written back.
    lastv: Variable,
    lastpav: Variable,
    /// The op being translated, for the clock a slow access must see.
    i: usize,
    /// Its address.
    op_pc: u32,
}

fn mf() -> MemFlagsData {
    MemFlagsData::trusted()
}

impl<'a> Tx<'a> {
    fn new(mut b: FunctionBuilder<'a>, cpu: Value, ptr: Type, sigs: [SigRef; 4], mem: Mem) -> Tx<'a> {
        let [hsig, rsig, wsig, isig] = sigs;
        let regs = std::array::from_fn(|_| b.declare_var(types::I32));
        let sr = b.declare_var(types::I32);
        let now = b.declare_var(types::I64);
        let now0 = b.ins().load(types::I64, mf(), cpu, offset_of!(Cpu, bus.io.now) as i32);
        b.def_var(now, now0);
        let macsr = b.declare_var(types::I32);
        let accs = std::array::from_fn(|_| b.declare_var(types::I64));
        let fs = b.declare_var(types::I32);
        let fd = b.declare_var(types::I32);
        let fr = b.declare_var(types::I32);
        let lastv = b.declare_var(types::I64);
        let lastpav = b.declare_var(types::I32);
        let z64 = b.ins().iconst(types::I64, 0);
        let z32 = b.ins().iconst(types::I32, 0);
        b.def_var(lastv, z64);
        b.def_var(lastpav, z32);
        Tx {
            b,
            cpu,
            ptr,
            hsig,
            rsig,
            wsig,
            isig,
            mem,
            regs,
            reg_st: [St::Unloaded; 16],
            sr,
            sr_st: St::Unloaded,
            now,
            pend: Pend::None,
            fs,
            fd,
            fr,
            mac_cached: false,
            omc: false,
            macsr,
            macsr_st: St::Unloaded,
            accs,
            acc_st: [St::Unloaded; 4],
            lastv,
            lastpav,
            i: 0,
            op_pc: 0,
        }
    }

    // -- constants and plumbing ---------------------------------------------

    fn k(&mut self, v: i64) -> Value {
        self.b.ins().iconst(types::I32, v as u32 as i32 as i64)
    }

    fn reg_off(n: usize) -> i32 {
        // repr(C): a follows d, both [u32; 8].
        (offset_of!(Cpu, d) + 4 * n) as i32
    }

    fn reg(&mut self, n: usize) -> Value {
        if self.reg_st[n] == St::Unloaded {
            let v = self.b.ins().load(types::I32, mf(), self.cpu, Self::reg_off(n));
            self.b.def_var(self.regs[n], v);
            self.reg_st[n] = St::Clean;
        }
        self.b.use_var(self.regs[n])
    }

    fn set_reg(&mut self, n: usize, v: Value) {
        self.b.def_var(self.regs[n], v);
        self.reg_st[n] = St::Dirty;
    }

    fn sr(&mut self) -> Value {
        if self.sr_st == St::Unloaded {
            let v = self.b.ins().uload16(types::I32, mf(), self.cpu, offset_of!(Cpu, sr) as i32);
            self.b.def_var(self.sr, v);
            self.sr_st = St::Clean;
        }
        self.b.use_var(self.sr)
    }

    fn set_sr(&mut self, v: Value) {
        self.b.def_var(self.sr, v);
        self.sr_st = St::Dirty;
    }

    /// Write cached registers and SR back to the Cpu.
    fn flush(&mut self) {
        self.materialize();
        for n in 0..16 {
            if self.reg_st[n] == St::Dirty {
                let v = self.b.use_var(self.regs[n]);
                self.b.ins().store(mf(), v, self.cpu, Self::reg_off(n));
                self.reg_st[n] = St::Clean;
            }
        }
        if self.sr_st == St::Dirty {
            let v = self.b.use_var(self.sr);
            self.b.ins().istore16(mf(), v, self.cpu, offset_of!(Cpu, sr) as i32);
            self.sr_st = St::Clean;
        }
        if self.mac_cached {
            for i in 0..4 {
                if self.acc_st[i] == St::Dirty {
                    let v = self.b.use_var(self.accs[i]);
                    self.b.ins().store(mf(), v, self.cpu, Self::acc_off(i));
                    self.acc_st[i] = St::Clean;
                }
            }
            if self.macsr_st == St::Dirty {
                self.settle_macsr();
                let v = self.b.use_var(self.macsr);
                self.b.ins().store(mf(), v, self.cpu, offset_of!(Cpu, macsr) as i32);
                self.macsr_st = St::Clean;
            }
        }
    }

    /// After a call that may have changed anything: reload on next use.
    fn forget(&mut self) {
        self.reg_st = [St::Unloaded; 16];
        self.sr_st = St::Unloaded;
        self.pend = Pend::None;
    }

    fn store_i32(&mut self, v: i64, off: usize) {
        let v = self.k(v);
        self.b.ins().store(mf(), v, self.cpu, off as i32);
    }

    /// Set the clock to the interpreter's after `ops_done` ops have begun.
    fn store_now(&mut self, ops_done: usize) {
        let n = self.b.use_var(self.now);
        let v = self.b.ins().iadd_imm_s(n, ops_done as i64);
        self.b.ins().store(mf(), v, self.cpu, offset_of!(Cpu, bus.io.now) as i32);
    }

    // -- memory ---------------------------------------------------------------

    /// The host address for an inline access of `sz` bytes at `addr`
    /// through page table `table`. -> (condition, host address)
    fn page(&mut self, table: *const usize, addr: Value, sz: u8) -> (Value, Value) {
        let pg = self.b.ins().ushr_imm_u(addr, 16);
        let pg = self.b.ins().uextend(self.ptr, pg);
        let pg = self.b.ins().ishl_imm_u(pg, 3);
        let t = self.b.ins().iconst(self.ptr, table as usize as i64);
        let e = self.b.ins().iadd(t, pg);
        let host = self.b.ins().load(self.ptr, mf(), e, 0);
        let off = self.b.ins().band_imm_u(addr, 0xFFFF);
        let mapped = self.b.ins().icmp_imm_u(IntCC::NotEqual, host, 0);
        let inside = self.b.ins().icmp_imm_u(IntCC::UnsignedLessThanOrEqual, off, 0x1_0000 - sz as i64);
        let ok = self.b.ins().band(mapped, inside);
        let off = self.b.ins().uextend(self.ptr, off);
        (ok, self.b.ins().iadd(host, off))
    }

    fn load_be(&mut self, p: Value, sz: u8) -> Value {
        let m = MemFlagsData::new().with_notrap();
        match sz {
            1 => self.b.ins().uload8(types::I32, m, p, 0),
            2 => {
                let h = self.b.ins().load(types::I16, m, p, 0);
                let h = self.b.ins().bswap(h);
                self.b.ins().uextend(types::I32, h)
            }
            _ => {
                let w = self.b.ins().load(types::I32, m, p, 0);
                self.b.ins().bswap(w)
            }
        }
    }

    fn store_be(&mut self, p: Value, sz: u8, v: Value) {
        let m = MemFlagsData::new().with_notrap();
        match sz {
            1 => {
                self.b.ins().istore8(m, v, p, 0);
            }
            2 => {
                let h = self.b.ins().ireduce(types::I16, v);
                let h = self.b.ins().bswap(h);
                self.b.ins().store(m, h, p, 0);
            }
            _ => {
                let w = self.b.ins().bswap(v);
                self.b.ins().store(m, w, p, 0);
            }
        }
    }

    /// A read as `Bus::read*` makes it: plain memory inline, the rest
    /// through the bus.
    fn read(&mut self, addr: Value, sz: u8) -> Value {
        let (ok, p) = self.page(self.mem.rd_pages, addr, sz);
        let fast = self.b.create_block();
        let slow = self.b.create_block();
        let join = self.b.create_block();
        self.b.append_block_param(join, types::I32);
        self.b.ins().brif(ok, fast, &[], slow, &[]);

        self.b.switch_to_block(fast);
        let v = self.load_be(p, sz);
        self.b.ins().jump(join, &[v.into()]);

        self.b.switch_to_block(slow);
        self.b.set_cold_block(slow);
        // A peripheral may read the clock.
        self.store_now(self.i + 1);
        let f = self.b.ins().iconst(self.ptr, jit_rd as *const () as usize as i64);
        let szv = self.k(sz as i64);
        let call = self.b.ins().call_indirect(self.rsig, f, &[self.cpu, addr, szv]);
        let v = self.b.inst_results(call)[0];
        self.b.ins().jump(join, &[v.into()]);

        self.b.switch_to_block(join);
        self.b.block_params(join)[0]
    }

    /// A write as `Bus::write*` makes it (stores near cached code go
    /// through the bus, which tells the block cache).
    fn write(&mut self, addr: Value, sz: u8, v: Value) {
        let (ok, p) = self.page(self.mem.wr_pages, addr, sz);
        let fast = self.b.create_block();
        let slow = self.b.create_block();
        let join = self.b.create_block();
        self.b.ins().brif(ok, fast, &[], slow, &[]);

        self.b.switch_to_block(fast);
        self.store_be(p, sz, v);
        self.b.ins().jump(join, &[]);

        self.b.switch_to_block(slow);
        self.b.set_cold_block(slow);
        self.store_now(self.i + 1);
        let f = self.b.ins().iconst(self.ptr, jit_wr as *const () as usize as i64);
        let szv = self.k(sz as i64);
        self.b.ins().call_indirect(self.wsig, f, &[self.cpu, addr, szv, v]);
        self.b.ins().jump(join, &[]);

        self.b.switch_to_block(join);
    }

    // -- effective addresses -----------------------------------------------------

    fn index(&mut self, base: Value, ext: u16) -> Value {
        let r = ((ext >> 12) & 7) as usize;
        let xi = self.reg(if ext & 0x8000 != 0 { 8 + r } else { r });
        let xi = if ext & 0x0800 != 0 {
            xi
        } else {
            let h = self.b.ins().ireduce(types::I16, xi);
            self.b.ins().sextend(types::I32, h)
        };
        let xi = self.b.ins().ishl_imm_u(xi, ((ext >> 9) & 3) as i64);
        let a = self.b.ins().iadd_imm_s(base, ext as u8 as i8 as i64);
        self.b.ins().iadd(a, xi)
    }

    /// The address of a memory operand, applying (An)+ / -(An).
    fn addr(&mut self, e: Ea, sz: u8) -> Value {
        match e {
            Ea::Ind(r) => self.reg(8 + (r & 7) as usize),
            Ea::Post(r) => {
                let n = 8 + (r & 7) as usize;
                let a = self.reg(n);
                let a2 = self.b.ins().iadd_imm_s(a, sz as i64);
                self.set_reg(n, a2);
                a
            }
            Ea::Pre(r) => {
                let n = 8 + (r & 7) as usize;
                let a = self.reg(n);
                let a2 = self.b.ins().iadd_imm_s(a, -(sz as i64));
                self.set_reg(n, a2);
                a2
            }
            Ea::Disp(r, d) => {
                let a = self.reg(8 + (r & 7) as usize);
                self.b.ins().iadd_imm_s(a, d as i64)
            }
            Ea::Idx(r, ext) => {
                let a = self.reg(8 + (r & 7) as usize);
                self.index(a, ext)
            }
            Ea::PcIdx(base, ext) => {
                let a = self.k(base as i64);
                self.index(a, ext)
            }
            Ea::Abs(a) => self.k(a as i64),
            Ea::D(_) | Ea::A(_) | Ea::Imm(_) => unreachable!("not a memory operand"),
        }
    }

    /// `v` masked to `sz` bytes (nothing to do at 4).
    fn masked(&mut self, v: Value, sz: u8) -> Value {
        if sz == 4 {
            v
        } else {
            self.b.ins().band_imm_u(v, mask(sz))
        }
    }

    /// A source operand masked to its size (`fast::read`, masking an
    /// immediate too, which the decoder already sized).
    fn get(&mut self, e: Ea, sz: u8) -> Value {
        match e {
            Ea::D(r) => {
                let v = self.reg((r & 7) as usize);
                self.masked(v, sz)
            }
            Ea::A(r) => {
                let v = self.reg(8 + (r & 7) as usize);
                self.masked(v, sz)
            }
            Ea::Imm(v) => self.k(v as i64 & mask(sz)),
            _ => {
                let a = self.addr(e, sz);
                self.read(a, sz)
            }
        }
    }

    /// Store to a data register's low `sz` bytes.
    fn put_dn(&mut self, r: usize, sz: u8, v: Value) {
        if sz == 4 {
            self.set_reg(r, v);
            return;
        }
        let old = self.reg(r);
        let keep = self.b.ins().band_imm_u(old, !mask(sz) & 0xFFFF_FFFF);
        let low = self.b.ins().band_imm_u(v, mask(sz));
        let n = self.b.ins().bor(keep, low);
        self.set_reg(r, n);
    }

    fn sext(&mut self, v: Value, sz: u8) -> Value {
        match sz {
            1 => {
                let h = self.b.ins().ireduce(types::I8, v);
                self.b.ins().sextend(types::I32, h)
            }
            2 => {
                let h = self.b.ins().ireduce(types::I16, v);
                self.b.ins().sextend(types::I32, h)
            }
            _ => v,
        }
    }

    // -- condition codes --------------------------------------------------------

    /// 0/1 as an i32 from a comparison, shifted to bit `at`.
    fn bit(&mut self, c: Value, at: i64) -> Value {
        let v = self.b.ins().uextend(types::I32, c);
        if at == 0 {
            v
        } else {
            self.b.ins().ishl_imm_u(v, at)
        }
    }

    /// N and Z of `r` (masked to sz) as SR bits.
    fn nz_bits(&mut self, r: Value, sz: u8) -> Value {
        let n = self.b.ins().ushr_imm_u(r, bits(sz) - 1);
        let n = self.b.ins().band_imm_u(n, 1);
        let n = self.b.ins().ishl_imm_u(n, 3);
        let z = self.b.ins().icmp_imm_u(IntCC::Equal, r, 0);
        let z = self.bit(z, 2);
        self.b.ins().bor(n, z)
    }

    /// The pending op's carry (0/1 as i32), for Add and Sub.
    fn pend_carry(&mut self, p: Pend) -> Value {
        let s = self.b.use_var(self.fs);
        let d = self.b.use_var(self.fd);
        let c = match p {
            Pend::Add(4) => {
                let r = self.b.ins().iadd(d, s);
                self.b.ins().icmp(IntCC::UnsignedLessThan, r, s)
            }
            Pend::Add(sz) => {
                let w = self.b.ins().iadd(d, s);
                let c = self.b.ins().ushr_imm_u(w, bits(sz));
                let c = self.b.ins().band_imm_u(c, 1);
                self.b.ins().icmp_imm_u(IntCC::NotEqual, c, 0)
            }
            _ => self.b.ins().icmp(IntCC::UnsignedGreaterThan, s, d),
        };
        self.bit(c, 0)
    }

    /// V as the pending op (or SR) has it, as an i8 truth value.
    fn overflow(&mut self) -> Value {
        match self.pend {
            Pend::Add(sz) | Pend::Sub(sz, _) => {
                let sub = matches!(self.pend, Pend::Sub(..));
                let s = self.b.use_var(self.fs);
                let d = self.b.use_var(self.fd);
                let r = self.b.use_var(self.fr);
                let v = if sub {
                    let a = self.b.ins().bxor(s, d);
                    let b2 = self.b.ins().bxor(r, d);
                    self.b.ins().band(a, b2)
                } else {
                    let a = self.b.ins().bxor(s, r);
                    let b2 = self.b.ins().bxor(d, r);
                    self.b.ins().band(a, b2)
                };
                let v = self.b.ins().ushr_imm_u(v, bits(sz) - 1);
                let v = self.b.ins().band_imm_u(v, 1);
                self.b.ins().icmp_imm_u(IntCC::NotEqual, v, 0)
            }
            Pend::Logic(_) => self.b.ins().iconst(types::I8, 0),
            Pend::None => {
                let sr = self.sr();
                let v = self.b.ins().band_imm_u(sr, CF_V);
                self.b.ins().icmp_imm_u(IntCC::NotEqual, v, 0)
            }
        }
    }

    /// Work the pending op's condition codes into SR.
    fn materialize(&mut self) {
        let p = self.pend;
        self.pend = Pend::None;
        let sr = self.sr();
        let f = match p {
            Pend::None => return,
            Pend::Logic(sz) => {
                let r = self.b.use_var(self.fr);
                let keep = self.b.ins().band_imm_u(sr, !0xF & 0xFFFF);
                let nz = self.nz_bits(r, sz);
                self.b.ins().bor(keep, nz)
            }
            Pend::Add(sz) | Pend::Sub(sz, _) => {
                let sub = matches!(p, Pend::Sub(..));
                let s = self.b.use_var(self.fs);
                let d = self.b.use_var(self.fd);
                let w = if sub { self.b.ins().isub(d, s) } else { self.b.ins().iadd(d, s) };
                let r = if sz == 4 { w } else { self.b.ins().band_imm_u(w, mask(sz)) };
                let cbit = self.pend_carry(p);
                let v = if sub {
                    let a = self.b.ins().bxor(s, d);
                    let b2 = self.b.ins().bxor(r, d);
                    self.b.ins().band(a, b2)
                } else {
                    let a = self.b.ins().bxor(s, r);
                    let b2 = self.b.ins().bxor(d, r);
                    self.b.ins().band(a, b2)
                };
                let v = self.b.ins().ushr_imm_u(v, bits(sz) - 1);
                let v = self.b.ins().band_imm_u(v, 1);
                let v = self.b.ins().ishl_imm_u(v, 1);
                let base = if p.writes_x() {
                    let cx = self.b.ins().ishl_imm_u(cbit, 4);
                    let s0 = self.b.ins().band_imm_u(sr, !0x1F & 0xFFFF);
                    self.b.ins().bor(s0, cx)
                } else {
                    self.b.ins().band_imm_u(sr, !0xF & 0xFFFF)
                };
                let nz = self.nz_bits(r, sz);
                let f = self.b.ins().bor(base, nz);
                let f = self.b.ins().bor(f, v);
                self.b.ins().bor(f, cbit)
            }
        };
        self.set_sr(f);
    }

    /// An op is about to set N Z V C (and X if `writes_x`): the pending
    /// op's codes die, except X if this op keeps it and it is still live.
    fn drop_pending(&mut self, writes_x: bool, x_live: bool) {
        let p = self.pend;
        self.pend = Pend::None;
        if p.writes_x() && !writes_x && x_live {
            let c = self.pend_carry(p);
            let x = self.b.ins().ishl_imm_u(c, 4);
            let sr = self.sr();
            let s0 = self.b.ins().band_imm_u(sr, !CF_X & 0xFFFF);
            let n = self.b.ins().bor(s0, x);
            self.set_sr(n);
        }
    }

    /// Make `p` (operands s, d, result r) the pending op, if its codes can
    /// be read (`live`: N Z V C, X).
    fn set_pending(&mut self, p: Pend, s: Value, d: Value, r: Value, live: (bool, bool)) {
        self.drop_pending(p.writes_x(), live.1);
        if live.0 || (p.writes_x() && live.1) {
            self.b.def_var(self.fs, s);
            self.b.def_var(self.fd, d);
            self.b.def_var(self.fr, r);
            self.pend = p;
        }
    }

    /// set_nz: N Z from r, V = C = 0, X kept. `r` masked to sz.
    fn logic_flags(&mut self, r: Value, sz: u8, live: (bool, bool)) {
        self.set_pending(Pend::Logic(sz), r, r, r, live);
    }

    /// add_flags / sub_flags (x = false), or cmp_flags (keep X). s and d
    /// masked to sz. -> the masked result.
    fn arith(&mut self, sub: bool, s: Value, d: Value, sz: u8, keep_x: bool, live: (bool, bool)) -> Value {
        let wide = if sub { self.b.ins().isub(d, s) } else { self.b.ins().iadd(d, s) };
        let r = if sz == 4 { wide } else { self.b.ins().band_imm_u(wide, mask(sz)) };
        let p = if sub { Pend::Sub(sz, keep_x) } else { Pend::Add(sz) };
        self.set_pending(p, s, d, r, live);
        r
    }

    /// `alu`: op 0 add, 1 sub, 2 and, 3 or, 4 eor, 5 cmp. s, d masked.
    fn alu(&mut self, op: u8, s: Value, d: Value, sz: u8, live: (bool, bool)) -> Value {
        match op {
            0 => self.arith(false, s, d, sz, false, live),
            1 => self.arith(true, s, d, sz, false, live),
            5 => {
                self.arith(true, s, d, sz, true, live);
                d
            }
            _ => {
                let r = match op {
                    2 => self.b.ins().band(s, d),
                    3 => self.b.ins().bor(s, d),
                    _ => self.b.ins().bxor(s, d),
                };
                self.logic_flags(r, sz, live);
                r
            }
        }
    }

    /// Condition `cc` (as `Cpu::cond`) -> an i8 truth value.
    fn cond(&mut self, cc: u8) -> Value {
        match (self.pend, cc) {
            (Pend::Sub(sz, _), 2..=7 | 10..=15) => {
                let s = self.b.use_var(self.fs);
                let d = self.b.use_var(self.fd);
                if matches!(cc, 10 | 11) {
                    // PL / MI: the sign of d - s at size.
                    let r = self.b.ins().isub(d, s);
                    let r = self.sext(r, sz);
                    let c = if cc == 10 { IntCC::SignedGreaterThanOrEqual } else { IntCC::SignedLessThan };
                    return self.b.ins().icmp_imm_s(c, r, 0);
                }
                let (s, d) = if matches!(cc, 12..=15) { (self.sext(s, sz), self.sext(d, sz)) } else { (s, d) };
                let c = match cc {
                    2 => IntCC::UnsignedGreaterThan,
                    3 => IntCC::UnsignedLessThanOrEqual,
                    4 => IntCC::UnsignedGreaterThanOrEqual,
                    5 => IntCC::UnsignedLessThan,
                    6 => IntCC::NotEqual,
                    7 => IntCC::Equal,
                    12 => IntCC::SignedGreaterThanOrEqual,
                    13 => IntCC::SignedLessThan,
                    14 => IntCC::SignedGreaterThan,
                    _ => IntCC::SignedLessThanOrEqual,
                };
                self.b.ins().icmp(c, d, s)
            }
            (Pend::Logic(sz), 2..=15) => {
                let r = self.b.use_var(self.fr);
                let sv = self.sext(r, sz);
                let (c, v) = match cc {
                    2 | 6 => (IntCC::NotEqual, r),
                    3 | 7 => (IntCC::Equal, r),
                    4 | 8 => return self.b.ins().iconst(types::I8, 1),
                    5 | 9 => return self.b.ins().iconst(types::I8, 0),
                    10 | 12 => (IntCC::SignedGreaterThanOrEqual, sv),
                    11 | 13 => (IntCC::SignedLessThan, sv),
                    14 => (IntCC::SignedGreaterThan, sv),
                    _ => (IntCC::SignedLessThanOrEqual, sv),
                };
                self.b.ins().icmp_imm_s(c, v, 0)
            }
            _ => {
                self.materialize();
                self.cond_sr(cc)
            }
        }
    }

    /// Condition from the SR bits.
    fn cond_sr(&mut self, cc: u8) -> Value {
        let sr = self.sr();
        let mut f = |bit: i64| {
            let v = self.b.ins().band_imm_u(sr, bit);
            self.b.ins().icmp_imm_u(IntCC::NotEqual, v, 0)
        };
        let c = f(CF_C);
        let v = f(CF_V);
        let z = f(CF_Z);
        let n = f(CF_N);
        match cc {
            0 => self.b.ins().iconst(types::I8, 1),
            1 => self.b.ins().iconst(types::I8, 0),
            2 => {
                let cz = self.b.ins().bor(c, z);
                self.b.ins().bxor_imm_u(cz, 1)
            }
            3 => self.b.ins().bor(c, z),
            4 => self.b.ins().bxor_imm_u(c, 1),
            5 => c,
            6 => self.b.ins().bxor_imm_u(z, 1),
            7 => z,
            8 => self.b.ins().bxor_imm_u(v, 1),
            9 => v,
            10 => self.b.ins().bxor_imm_u(n, 1),
            11 => n,
            12 => self.b.ins().icmp(IntCC::Equal, n, v),
            13 => self.b.ins().icmp(IntCC::NotEqual, n, v),
            14 => {
                let nv = self.b.ins().icmp(IntCC::Equal, n, v);
                let nz = self.b.ins().bxor_imm_u(z, 1);
                self.b.ins().band(nz, nv)
            }
            _ => {
                let nv = self.b.ins().icmp(IntCC::NotEqual, n, v);
                self.b.ins().bor(z, nv)
            }
        }
    }

    // -- ops -----------------------------------------------------------------

    /// Translate op `o` inline. -> false if it must be a call.
    fn op(&mut self, o: &Op, live: (bool, bool)) -> bool {
        let lf = live;
        match o.k {
            Kind::Move { sz } => {
                let v = self.get(o.a, sz);
                match o.b {
                    Ea::D(r) => self.put_dn((r & 7) as usize, sz, v),
                    Ea::A(_) | Ea::Imm(_) => return false,
                    e => {
                        let a = self.addr(e, sz);
                        self.write(a, sz, v);
                    }
                }
                self.logic_flags(v, sz, lf);
            }
            Kind::MoveA { sz } => {
                let v = self.get(o.a, sz);
                let v = self.sext(v, sz);
                self.set_reg(8 + (o.r & 7) as usize, v);
            }
            Kind::MoveQ => {
                let v = self.k(o.x as i64);
                self.set_reg((o.r & 7) as usize, v);
                self.logic_flags(v, 4, lf);
            }
            Kind::Mvs { sz } | Kind::Mvz { sz } => {
                let v = self.get(o.a, sz);
                let v = if matches!(o.k, Kind::Mvs { .. }) { self.sext(v, sz) } else { v };
                self.set_reg((o.r & 7) as usize, v);
                self.logic_flags(v, 4, lf);
            }
            Kind::Lea => {
                let a = self.addr(o.a, 4);
                self.set_reg(8 + (o.r & 7) as usize, a);
            }
            Kind::AluEaDn { op, sz } => {
                let s = self.get(o.a, sz);
                let dn = (o.r & 7) as usize;
                let d = self.reg(dn);
                let d = self.masked(d, sz);
                let r = self.alu(op, s, d, sz, live);
                if op != 5 {
                    self.put_dn(dn, sz, r);
                }
            }
            Kind::AluImm { op, sz } => {
                let s = self.k(o.x as i64 & mask(sz));
                let need = live;
                match o.b {
                    Ea::D(r) => {
                        let dn = (r & 7) as usize;
                        let d = self.reg(dn);
                        let d = self.masked(d, sz);
                        let r = self.alu(op, s, d, sz, need);
                        if op != 5 {
                            self.put_dn(dn, sz, r);
                        }
                    }
                    Ea::A(_) | Ea::Imm(_) => return false,
                    e => {
                        let a = self.addr(e, sz);
                        let d = self.read(a, sz);
                        let r = self.alu(op, s, d, sz, need);
                        if op != 5 {
                            self.write(a, sz, r);
                        }
                    }
                }
            }
            Kind::AluAn { op, sz } => {
                let s = self.get(o.a, sz);
                let s = self.sext(s, sz);
                let an = 8 + (o.r & 7) as usize;
                let a = self.reg(an);
                match op {
                    0 => {
                        let n = self.b.ins().iadd(a, s);
                        self.set_reg(an, n);
                    }
                    1 => {
                        let n = self.b.ins().isub(a, s);
                        self.set_reg(an, n);
                    }
                    _ => {
                        self.arith(true, s, a, 4, true, lf);
                    }
                }
            }
            Kind::QuickAn => {
                let an = 8 + (o.r & 7) as usize;
                let a = self.reg(an);
                let n = self.b.ins().iadd_imm_s(a, o.x as i32 as i64);
                self.set_reg(an, n);
            }
            Kind::CmpiDn { sz } => {
                let s = self.k(o.x as i64 & mask(sz));
                let d = self.reg((o.r & 7) as usize);
                let d = self.masked(d, sz);
                self.arith(true, s, d, sz, true, lf);
            }
            Kind::Clr { sz } => {
                let z = self.k(0);
                match o.b {
                    Ea::D(r) => self.put_dn((r & 7) as usize, sz, z),
                    Ea::A(_) | Ea::Imm(_) => return false,
                    e => {
                        let a = self.addr(e, sz);
                        self.write(a, sz, z);
                    }
                }
                self.logic_flags(z, sz, lf);
            }
            Kind::Tst { sz } => {
                let v = self.get(o.a, sz);
                self.logic_flags(v, sz, lf);
            }
            Kind::Shift { kind, reg } => self.shift(o, kind, reg, live),
            Kind::Addx { sub } => self.addx(o, sub),
            Kind::Mac { lm, long } => {
                if self.mac_cached {
                    self.mac_fast(o, lm, long)
                } else {
                    self.mac(o, lm, long)
                }
            }
            Kind::MovAcc => {
                if self.mac_cached {
                    self.movacc_fast(o)
                } else {
                    self.movacc(o)
                }
            }
            Kind::Sats => {
                let dn = (o.r & 7) as usize;
                let d = self.reg(dn);
                // Only V is read; the rest of the pending codes die here
                // (logic_flags keeps X if it is still live).
                let ovf = self.overflow();
                let neg = self.b.ins().icmp_imm_s(IntCC::SignedLessThan, d, 0);
                let mx = self.k(0x7FFF_FFFF);
                let mn = self.k(0x8000_0000);
                let sat = self.b.ins().select(neg, mx, mn);
                let r = self.b.ins().select(ovf, sat, d);
                self.set_reg(dn, r);
                self.logic_flags(r, 4, lf);
            }
            Kind::Swap => {
                let dn = (o.r & 7) as usize;
                let d = self.reg(dn);
                let r = self.b.ins().rotl_imm_u(d, 16);
                self.set_reg(dn, r);
                self.logic_flags(r, 4, lf);
            }
            Kind::Movem => {
                let base = self.addr(o.a, 4);
                let mut k = 0;
                for i in 0..16usize {
                    if o.x & (1 << i) == 0 {
                        continue;
                    }
                    let a = self.b.ins().iadd_imm_s(base, 4 * k);
                    if o.r != 0 {
                        let v = self.reg(i);
                        self.write(a, 4, v);
                    } else {
                        let v = self.read(a, 4);
                        self.set_reg(i, v);
                    }
                    k += 1;
                }
            }
            Kind::BitDn { kind } => {
                let dn = (o.r & 7) as usize;
                let n = self.reg((o.x & 7) as usize);
                let n = self.b.ins().band_imm_u(n, 31);
                let one = self.k(1);
                let m = self.b.ins().ishl(one, n);
                let v = self.reg(dn);
                let t = self.b.ins().band(v, m);
                let z = self.b.ins().icmp_imm_u(IntCC::Equal, t, 0);
                let z = self.bit(z, 2);
                self.materialize();
                let sr = self.sr();
                let s0 = self.b.ins().band_imm_u(sr, !CF_Z & 0xFFFF);
                let f = self.b.ins().bor(s0, z);
                self.set_sr(f);
                let nv = match kind {
                    1 => Some(self.b.ins().bxor(v, m)),
                    2 => {
                        let nm = self.b.ins().bnot(m);
                        Some(self.b.ins().band(v, nm))
                    }
                    3 => Some(self.b.ins().bor(v, m)),
                    _ => None,
                };
                if let Some(nv) = nv {
                    self.set_reg(dn, nv);
                }
            }
            Kind::EorDn { sz } => {
                let s = self.reg((o.x & 7) as usize);
                let s = self.masked(s, sz);
                let dn = (o.r & 7) as usize;
                let d = self.reg(dn);
                let d = self.masked(d, sz);
                let r = self.b.ins().bxor(s, d);
                self.put_dn(dn, sz, r);
                self.logic_flags(r, sz, lf);
            }
            Kind::Bcc | Kind::Bra | Kind::Call => return false,
        }
        true
    }

    fn shift(&mut self, o: &Op, kind: u8, reg: bool, live: (bool, bool)) {
        let dn = (o.r & 7) as usize;
        let v = self.reg(dn);
        let need = live.0 || live.1;
        if !reg {
            let c = o.x as i64; // 1..8
            let res = match kind {
                0 => self.b.ins().ishl_imm_u(v, c),
                1 => self.b.ins().ushr_imm_u(v, c),
                _ => self.b.ins().sshr_imm_u(v, c),
            };
            self.set_reg(dn, res);
            self.drop_pending(true, live.1);
            if need {
                let cy = if kind == 0 { self.b.ins().ushr_imm_u(v, 32 - c) } else { self.b.ins().ushr_imm_u(v, c - 1) };
                let cy = self.b.ins().band_imm_u(cy, 1);
                let cx = self.b.ins().imul_imm_s(cy, CF_C | CF_X);
                let sr = self.sr();
                let s0 = self.b.ins().band_imm_u(sr, !0x1F & 0xFFFF);
                let nz = self.nz_bits(res, 4);
                let f = self.b.ins().bor(s0, cx);
                let f = self.b.ins().bor(f, nz);
                self.set_sr(f);
            }
            return;
        }
        // A zero count keeps X: it is an input.
        self.materialize();
        let cnt = self.reg((o.x & 7) as usize);
        let c = self.b.ins().band_imm_u(cnt, 63);
        let zero = self.k(0);
        let ge32 = self.b.ins().icmp_imm_u(IntCC::UnsignedGreaterThanOrEqual, c, 32);
        let is0 = self.b.ins().icmp_imm_u(IntCC::Equal, c, 0);
        let gt32 = self.b.ins().icmp_imm_u(IntCC::UnsignedGreaterThan, c, 32);
        let cm1 = self.b.ins().iadd_imm_s(c, -1);
        let (res, cy) = match kind {
            0 => {
                let sh = self.b.ins().ishl(v, c);
                let res = self.b.ins().select(ge32, zero, sh);
                let k32 = self.k(32);
                let back = self.b.ins().isub(k32, c);
                let t = self.b.ins().ushr(v, back);
                let t = self.b.ins().band_imm_u(t, 1);
                let none = self.b.ins().bor(is0, gt32);
                let cy = self.b.ins().select(none, zero, t);
                (res, cy)
            }
            1 => {
                let sh = self.b.ins().ushr(v, c);
                let res = self.b.ins().select(ge32, zero, sh);
                let t = self.b.ins().ushr(v, cm1);
                let t = self.b.ins().band_imm_u(t, 1);
                let none = self.b.ins().bor(is0, gt32);
                let cy = self.b.ins().select(none, zero, t);
                (res, cy)
            }
            _ => {
                let sign = self.b.ins().sshr_imm_u(v, 31);
                let sh = self.b.ins().sshr(v, c);
                let res = self.b.ins().select(ge32, sign, sh);
                let t = self.b.ins().ushr(v, cm1);
                let t = self.b.ins().band_imm_u(t, 1);
                let s1 = self.b.ins().band_imm_u(sign, 1);
                let t = self.b.ins().select(ge32, s1, t);
                let cy = self.b.ins().select(is0, zero, t);
                (res, cy)
            }
        };
        self.set_reg(dn, res);
        if need {
            let sr = self.sr();
            let keepx = self.b.ins().band_imm_u(sr, CF_X);
            let cx = self.b.ins().imul_imm_s(cy, CF_C | CF_X);
            let low = self.b.ins().select(is0, keepx, cx);
            let s0 = self.b.ins().band_imm_u(sr, !0x1F & 0xFFFF);
            let nz = self.nz_bits(res, 4);
            let f = self.b.ins().bor(s0, low);
            let f = self.b.ins().bor(f, nz);
            self.set_sr(f);
        }
    }

    fn addx(&mut self, o: &Op, sub: bool) {
        // X and Z are inputs.
        self.materialize();
        let sr = self.sr();
        let x = self.b.ins().ushr_imm_u(sr, 4);
        let x = self.b.ins().band_imm_u(x, 1);
        let s = self.reg((o.x & 7) as usize);
        let dn = (o.r & 7) as usize;
        let d = self.reg(dn);
        let s64 = self.b.ins().uextend(types::I64, s);
        let d64 = self.b.ins().uextend(types::I64, d);
        let x64 = self.b.ins().uextend(types::I64, x);
        let (r, c) = if sub {
            let t = self.b.ins().isub(d, s);
            let r = self.b.ins().isub(t, x);
            let sx = self.b.ins().iadd(s64, x64);
            let c = self.b.ins().icmp(IntCC::UnsignedGreaterThan, sx, d64);
            (r, c)
        } else {
            let w = self.b.ins().iadd(s64, d64);
            let w = self.b.ins().iadd(w, x64);
            let r = self.b.ins().ireduce(types::I32, w);
            let hi = self.b.ins().ushr_imm_u(w, 32);
            let c = self.b.ins().icmp_imm_u(IntCC::NotEqual, hi, 0);
            (r, c)
        };
        let v = if sub {
            let a = self.b.ins().bxor(s, d);
            let b2 = self.b.ins().bxor(r, d);
            self.b.ins().band(a, b2)
        } else {
            let a = self.b.ins().bxor(s, r);
            let b2 = self.b.ins().bxor(d, r);
            self.b.ins().band(a, b2)
        };
        let v = self.b.ins().ushr_imm_u(v, 31);
        let v = self.b.ins().ishl_imm_u(v, 1);
        let cbit = self.bit(c, 0);
        let cx = self.b.ins().imul_imm_s(cbit, CF_C | CF_X);
        let n = self.b.ins().ushr_imm_u(r, 31);
        let n = self.b.ins().ishl_imm_u(n, 3);
        // Z: cleared by a non-zero result, else kept.
        let z_old = self.b.ins().band_imm_u(sr, CF_Z);
        let is0 = self.b.ins().icmp_imm_u(IntCC::Equal, r, 0);
        let zero = self.k(0);
        let z = self.b.ins().select(is0, z_old, zero);
        let s0 = self.b.ins().band_imm_u(sr, !0x1F & 0xFFFF);
        let f = self.b.ins().bor(s0, cx);
        let f = self.b.ins().bor(f, v);
        let f = self.b.ins().bor(f, n);
        let f = self.b.ins().bor(f, z);
        self.set_sr(f);
        self.set_reg(dn, r);
    }

    // -- EMAC ---------------------------------------------------------------------

    fn acc_off(i: usize) -> i32 {
        (offset_of!(Cpu, accv) + 8 * i) as i32
    }

    /// MAC/MSAC as `fast::h_mac_f`: fractional mode without rounding
    /// inline; any other mode runs `h_mac`.
    fn mac(&mut self, o: &Op, lm: u8, long: bool) {
        use crate::cpu::{MACSR_FI, MACSR_OMC, MACSR_PAV0, MACSR_RT, MACSR_SU, MACSR_V};
        let ext = o.x;
        let acc = ((ext >> 24) & 3) as usize;
        let (rxn, ryn) = (((ext >> 16) & 15) as usize, ((ext >> 20) & 15) as usize);
        let areg = 8 + (o.op & 7) as usize;
        let rw = ((o.op >> 9) & 7) as usize + if o.op & 0x40 != 0 { 8 } else { 0 };
        // Registers either path may load or write: the handler path reloads
        // them all, so both define them where the paths meet.
        let mut touched = vec![rxn, ryn];
        if lm != 0 {
            touched.extend([rw, areg]);
        }
        let off_m = offset_of!(Cpu, macsr) as i32;

        let m = self.b.ins().load(types::I32, mf(), self.cpu, off_m);
        let g = self.b.ins().band_imm_u(m, (MACSR_RT | MACSR_SU | MACSR_FI) as i64);
        let fast_mode = self.b.ins().icmp_imm_u(IntCC::Equal, g, MACSR_FI as i64);
        let inline = self.b.create_block();
        let other = self.b.create_block();
        let join = self.b.create_block();
        self.b.ins().brif(fast_mode, inline, &[], other, &[]);

        // Another mode: the handler, with registers written back first and
        // reloaded after (including any this op writes).
        self.b.switch_to_block(other);
        self.b.set_cold_block(other);
        let before = self.reg_st;
        for (n, st) in before.iter().enumerate() {
            if *st == St::Dirty {
                let v = self.b.use_var(self.regs[n]);
                self.b.ins().store(mf(), v, self.cpu, Self::reg_off(n));
            }
        }
        // As the interpreter has them while the op runs (a peripheral read
        // may look at the clock).
        self.store_i32(self.op_pc as i64, offset_of!(Cpu, op_pc));
        self.store_now(self.i + 1);
        self.call_handler(o);
        for (n, st) in before.iter().enumerate() {
            if *st != St::Unloaded || touched.contains(&n) {
                let v = self.b.ins().load(types::I32, mf(), self.cpu, Self::reg_off(n));
                self.b.def_var(self.regs[n], v);
            }
        }
        self.b.ins().jump(join, &[]);

        // Fractional mode.
        self.b.switch_to_block(inline);
        self.reg_st = before;
        let rx = self.reg(rxn);
        let ry = self.reg(ryn);
        let mut addr = None;
        let mut lv = None;
        if lm != 0 {
            let a = self.reg(areg);
            let base = match lm {
                2 | 3 => a,
                4 => self.b.ins().iadd_imm_s(a, -4),
                _ => {
                    let d = match o.a {
                        Ea::Imm(d) => d,
                        _ => 0,
                    };
                    self.b.ins().iadd_imm_s(a, d as i32 as i64)
                }
            };
            let ad = if ext & 0x20 != 0 {
                let mk = self.b.ins().load(types::I32, mf(), self.cpu, offset_of!(Cpu, mask) as i32);
                self.b.ins().band(base, mk)
            } else {
                base
            };
            lv = Some(self.read(ad, 4));
            addr = Some(ad);
        }
        let pav = (MACSR_PAV0 << acc) as i64;
        let omc = self.b.ins().band_imm_u(m, MACSR_OMC as i64);
        let has_pav = self.b.ins().band_imm_u(m, pav);
        let omc_on = self.b.ins().icmp_imm_u(IntCC::NotEqual, omc, 0);
        let pav_on = self.b.ins().icmp_imm_u(IntCC::NotEqual, has_pav, 0);
        let frozen = self.b.ins().band(omc_on, pav_on);
        let compute = self.b.create_block();
        let after = self.b.create_block();
        self.b.ins().brif(frozen, after, &[], compute, &[]);

        self.b.switch_to_block(compute);
        let (x, y) = if long {
            (rx, ry)
        } else {
            let hi = |t: &mut Self, v: Value, upper: bool| {
                if upper {
                    t.b.ins().band_imm_u(v, 0xFFFF_0000u32 as i64)
                } else {
                    t.b.ins().ishl_imm_u(v, 16)
                }
            };
            (hi(self, rx, ext & 0x80 != 0), hi(self, ry, ext & 0x40 != 0))
        };
        let x64 = self.b.ins().sextend(types::I64, x);
        let y64 = self.b.ins().sextend(types::I64, y);
        let prod = self.b.ins().imul(x64, y64);
        let prod = self.b.ins().ishl_imm_u(prod, 1);
        let prod = self.b.ins().sshr_imm_u(prod, 24);
        let xm = self.b.ins().icmp_imm_u(IntCC::Equal, x, 0x8000_0000u32 as i32 as i64);
        let ym = self.b.ins().icmp_imm_u(IntCC::Equal, y, 0x8000_0000u32 as i32 as i64);
        let both = self.b.ins().band(xm, ym);
        let one = self.b.ins().iconst(types::I64, 1i64 << 39);
        let p = self.b.ins().select(both, one, prod);
        let cur = self.b.ins().load(types::I64, mf(), self.cpu, Self::acc_off(acc));
        let sum = if ext & 0x100 != 0 { self.b.ins().isub(cur, p) } else { self.b.ins().iadd(cur, p) };
        let e48 = self.b.ins().ishl_imm_u(sum, 16);
        let e48 = self.b.ins().sshr_imm_u(e48, 16);
        let ovf = self.b.ins().icmp(IntCC::NotEqual, e48, sum);
        let neg = self.b.ins().icmp_imm_s(IntCC::SignedLessThan, sum, 0);
        let lo = self.b.ins().iconst(types::I64, 0xFFFF_FF80_0000_0000u64 as i64);
        let hi = self.b.ins().iconst(types::I64, 0x007F_FFFF_FF00);
        let sat = self.b.ins().select(neg, lo, hi);
        let ovf_v = self.b.ins().select(omc_on, sat, e48);
        let v = self.b.ins().select(ovf, ovf_v, sum);
        self.b.ins().store(mf(), v, self.cpu, Self::acc_off(acc));
        let f0 = self.b.ins().band_imm_u(m, !0xF & 0xFFFF_FFFF);
        let vp = self.k((MACSR_V as i64) | pav);
        let zero = self.k(0);
        let ovf_bits = self.b.ins().select(ovf, vp, zero);
        let f = self.b.ins().bor(f0, ovf_bits);
        let is0 = self.b.ins().icmp_imm_u(IntCC::Equal, v, 0);
        let b47 = self.b.ins().ushr_imm_u(v, 47);
        let b47 = self.b.ins().band_imm_u(b47, 1);
        let b47 = self.b.ins().ireduce(types::I32, b47);
        let nbit = self.b.ins().ishl_imm_u(b47, 3);
        let four = self.k(4);
        let nz = self.b.ins().select(is0, four, nbit);
        let f = self.b.ins().bor(f, nz);
        let pv = self.b.ins().band_imm_u(f, pav);
        let pv_on = self.b.ins().icmp_imm_u(IntCC::NotEqual, pv, 0);
        let vbit = self.k(MACSR_V as i64);
        let vb = self.b.ins().select(pv_on, vbit, zero);
        let f = self.b.ins().bor(f, vb);
        let t = self.b.ins().sshr_imm_u(v, 39);
        let t0 = self.b.ins().icmp_imm_s(IntCC::Equal, t, 0);
        let tm = self.b.ins().icmp_imm_s(IntCC::Equal, t, -1);
        let fits = self.b.ins().bor(t0, tm);
        let onev = self.k(1);
        let ev = self.b.ins().select(fits, zero, onev);
        let f = self.b.ins().bor(f, ev);
        self.b.ins().store(mf(), f, self.cpu, off_m);
        self.b.ins().jump(after, &[]);

        self.b.switch_to_block(after);
        if let (Some(ad), Some(lv)) = (addr, lv) {
            self.set_reg(rw, lv);
            match lm {
                3 => {
                    let n = self.b.ins().iadd_imm_s(ad, 4);
                    self.set_reg(areg, n);
                }
                4 => self.set_reg(areg, ad),
                _ => {}
            }
        }
        self.b.ins().jump(join, &[]);

        // From here the inline path's register states hold: where the
        // handler ran, every touched register was just reloaded, so a
        // "dirty" one only gets stored again.
        self.b.switch_to_block(join);
    }

    fn acc(&mut self, i: usize) -> Value {
        if self.acc_st[i] == St::Unloaded {
            let v = self.b.ins().load(types::I64, mf(), self.cpu, Self::acc_off(i));
            self.b.def_var(self.accs[i], v);
            self.acc_st[i] = St::Clean;
        }
        self.b.use_var(self.accs[i])
    }

    fn set_acc(&mut self, i: usize, v: Value) {
        self.b.def_var(self.accs[i], v);
        self.acc_st[i] = St::Dirty;
    }

    /// Work MACSR's N Z V EV out from the last MAC that set them, as that
    /// MAC would have (V: its accumulator's PAV, which nothing has cleared
    /// since -- MOVCLR settles first).
    fn settle_macsr(&mut self) {
        let lp = self.b.use_var(self.lastpav);
        let v = self.b.use_var(self.lastv);
        let m = self.b.use_var(self.macsr);
        let zero = self.k(0);
        let is0 = self.b.ins().icmp_imm_u(IntCC::Equal, v, 0);
        let b47 = self.b.ins().ushr_imm_u(v, 47);
        let b47 = self.b.ins().band_imm_u(b47, 1);
        let b47 = self.b.ins().ireduce(types::I32, b47);
        let nbit = self.b.ins().ishl_imm_u(b47, 3);
        let four = self.k(4);
        let nz = self.b.ins().select(is0, four, nbit);
        let pv = self.b.ins().band(m, lp);
        let pv_on = self.b.ins().icmp_imm_u(IntCC::NotEqual, pv, 0);
        let vbit = self.k(crate::cpu::MACSR_V as i64);
        let vb = self.b.ins().select(pv_on, vbit, zero);
        let t = self.b.ins().sshr_imm_u(v, 39);
        let t0 = self.b.ins().icmp_imm_s(IntCC::Equal, t, 0);
        let tm = self.b.ins().icmp_imm_s(IntCC::Equal, t, -1);
        let fits = self.b.ins().bor(t0, tm);
        let onev = self.k(1);
        let ev = self.b.ins().select(fits, zero, onev);
        let low = self.b.ins().bor(nz, vb);
        let low = self.b.ins().bor(low, ev);
        let hi = self.b.ins().band_imm_u(m, !0xF & 0xFFFF_FFFF);
        let new = self.b.ins().bor(hi, low);
        let any = self.b.ins().icmp_imm_u(IntCC::NotEqual, lp, 0);
        let m2 = self.b.ins().select(any, new, m);
        self.b.def_var(self.macsr, m2);
        self.b.def_var(self.lastpav, zero);
    }

    /// MAC/MSAC with the EMAC state in variables (the block checked the
    /// mode on entry). N Z V EV are settled when MACSR is written back.
    fn mac_fast(&mut self, o: &Op, lm: u8, long: bool) {
        use crate::cpu::{MACSR_OMC, MACSR_PAV0};
        let ext = o.x;
        let acc = ((ext >> 24) & 3) as usize;
        let rx = self.reg(((ext >> 16) & 15) as usize);
        let ry = self.reg(((ext >> 20) & 15) as usize);
        let areg = 8 + (o.op & 7) as usize;
        let rw = ((o.op >> 9) & 7) as usize + if o.op & 0x40 != 0 { 8 } else { 0 };
        let mut load = None;
        if lm != 0 {
            let a = self.reg(areg);
            let base = match lm {
                2 | 3 => a,
                4 => self.b.ins().iadd_imm_s(a, -4),
                _ => {
                    let d = match o.a {
                        Ea::Imm(d) => d,
                        _ => 0,
                    };
                    self.b.ins().iadd_imm_s(a, d as i32 as i64)
                }
            };
            let ad = if ext & 0x20 != 0 {
                let mk = self.b.ins().load(types::I32, mf(), self.cpu, offset_of!(Cpu, mask) as i32);
                self.b.ins().band(base, mk)
            } else {
                base
            };
            load = Some((ad, self.read(ad, 4)));
        }
        let m = self.b.use_var(self.macsr);
        let cur = self.acc(acc);
        let pav = (MACSR_PAV0 << acc) as i64;
        let _ = MACSR_OMC;
        let compute = self.b.create_block();
        let after = self.b.create_block();
        if self.omc {
            // OMC: an accumulator that overflowed stays frozen.
            let has_pav = self.b.ins().band_imm_u(m, pav);
            let frozen = self.b.ins().icmp_imm_u(IntCC::NotEqual, has_pav, 0);
            self.b.ins().brif(frozen, after, &[], compute, &[]);
        } else {
            self.b.ins().jump(compute, &[]);
        }

        self.b.switch_to_block(compute);
        let (x, y) = if long {
            (rx, ry)
        } else {
            let half = |t: &mut Self, v: Value, upper: bool| {
                if upper {
                    t.b.ins().band_imm_u(v, 0xFFFF_0000u32 as i64)
                } else {
                    t.b.ins().ishl_imm_u(v, 16)
                }
            };
            (half(self, rx, ext & 0x80 != 0), half(self, ry, ext & 0x40 != 0))
        };
        let x64 = self.b.ins().sextend(types::I64, x);
        let y64 = self.b.ins().sextend(types::I64, y);
        let prod = self.b.ins().imul(x64, y64);
        let prod = self.b.ins().ishl_imm_u(prod, 1);
        let prod = self.b.ins().sshr_imm_u(prod, 24);
        let xm = self.b.ins().icmp_imm_u(IntCC::Equal, x, 0x8000_0000u32 as i32 as i64);
        let ym = self.b.ins().icmp_imm_u(IntCC::Equal, y, 0x8000_0000u32 as i32 as i64);
        let both = self.b.ins().band(xm, ym);
        let one = self.b.ins().iconst(types::I64, 1i64 << 39);
        let p = self.b.ins().select(both, one, prod);
        let sum = if ext & 0x100 != 0 { self.b.ins().isub(cur, p) } else { self.b.ins().iadd(cur, p) };
        let e48 = self.b.ins().ishl_imm_u(sum, 16);
        let e48 = self.b.ins().sshr_imm_u(e48, 16);
        let ovf = self.b.ins().icmp(IntCC::NotEqual, e48, sum);
        let v = if self.omc {
            // Saturate on overflow.
            let neg = self.b.ins().icmp_imm_s(IntCC::SignedLessThan, sum, 0);
            let lo = self.b.ins().iconst(types::I64, 0xFFFF_FF80_0000_0000u64 as i64);
            let hi = self.b.ins().iconst(types::I64, 0x007F_FFFF_FF00);
            let sat = self.b.ins().select(neg, lo, hi);
            self.b.ins().select(ovf, sat, sum)
        } else {
            // Wrap to 48 bits (equal to the sum when it fits).
            e48
        };
        self.set_acc(acc, v);
        let pk = self.k(pav);
        let zero = self.k(0);
        let pb = self.b.ins().select(ovf, pk, zero);
        let m2 = self.b.ins().bor(m, pb);
        self.b.def_var(self.macsr, m2);
        self.macsr_st = St::Dirty;
        self.b.def_var(self.lastv, v);
        self.b.def_var(self.lastpav, pk);
        self.b.ins().jump(after, &[]);

        self.b.switch_to_block(after);
        if let Some((ad, lv)) = load {
            self.set_reg(rw, lv);
            match lm {
                3 => {
                    let n = self.b.ins().iadd_imm_s(ad, 4);
                    self.set_reg(areg, n);
                }
                4 => self.set_reg(areg, ad),
                _ => {}
            }
        }
    }

    /// MOVE.L / MOVCLR.L ACCy,Rx with the EMAC state in variables.
    fn movacc_fast(&mut self, o: &Op) {
        use crate::cpu::{MACSR_OMC, MACSR_PAV0};
        let i = (o.x & 3) as usize;
        let a = self.acc(i);
        let m = self.b.use_var(self.macsr);
        let _ = (m, MACSR_OMC);
        let q = self.b.ins().sshr_imm_u(a, 8);
        let q32 = self.b.ins().ireduce(types::I32, q);
        let v = if self.omc {
            // Saturate to 32 bits.
            let back = self.b.ins().sextend(types::I64, q32);
            let wide = self.b.ins().icmp(IntCC::NotEqual, back, q);
            let neg = self.b.ins().icmp_imm_s(IntCC::SignedLessThan, q, 0);
            let mn = self.k(i32::MIN as i64);
            let mx = self.k(i32::MAX as i64);
            let sat = self.b.ins().select(neg, mn, mx);
            self.b.ins().select(wide, sat, q32)
        } else {
            q32
        };
        self.set_reg((o.r & 15) as usize, v);
        if o.x & 0x100 != 0 {
            // Clearing PAV changes what a pending V would be: settle first.
            self.settle_macsr();
            let z = self.b.ins().iconst(types::I64, 0);
            self.set_acc(i, z);
            let m = self.b.use_var(self.macsr);
            let n = self.b.ins().band_imm_u(m, !((MACSR_PAV0 << i) as i64) & 0xFFFF_FFFF);
            self.b.def_var(self.macsr, n);
            self.macsr_st = St::Dirty;
        }
    }

    /// MOVE.L / MOVCLR.L ACCy,Rx as `fast::h_movacc`.
    fn movacc(&mut self, o: &Op) {
        use crate::cpu::{MACSR_FI, MACSR_OMC, MACSR_PAV0, MACSR_RT, MACSR_SU};
        let i = (o.x & 3) as usize;
        let off_m = offset_of!(Cpu, macsr) as i32;
        let m = self.b.ins().load(types::I32, mf(), self.cpu, off_m);
        let g = self.b.ins().band_imm_u(m, (MACSR_RT | MACSR_SU | MACSR_FI) as i64);
        let fast_mode = self.b.ins().icmp_imm_u(IntCC::Equal, g, MACSR_FI as i64);
        let inline = self.b.create_block();
        let other = self.b.create_block();
        let join = self.b.create_block();
        self.b.append_block_param(join, types::I32);
        self.b.ins().brif(fast_mode, inline, &[], other, &[]);

        self.b.switch_to_block(inline);
        let a = self.b.ins().load(types::I64, mf(), self.cpu, Self::acc_off(i));
        let q = self.b.ins().sshr_imm_u(a, 8);
        let q32 = self.b.ins().ireduce(types::I32, q);
        let back = self.b.ins().sextend(types::I64, q32);
        let wide = self.b.ins().icmp(IntCC::NotEqual, back, q);
        let omc = self.b.ins().band_imm_u(m, MACSR_OMC as i64);
        let omc_on = self.b.ins().icmp_imm_u(IntCC::NotEqual, omc, 0);
        let sat_needed = self.b.ins().band(wide, omc_on);
        let neg = self.b.ins().icmp_imm_s(IntCC::SignedLessThan, q, 0);
        let mn = self.k(i32::MIN as i64);
        let mx = self.k(i32::MAX as i64);
        let sat = self.b.ins().select(neg, mn, mx);
        let v = self.b.ins().select(sat_needed, sat, q32);
        self.b.ins().jump(join, &[v.into()]);

        self.b.switch_to_block(other);
        self.b.set_cold_block(other);
        let f = self.b.ins().iconst(self.ptr, jit_mac_read as *const () as usize as i64);
        let iv = self.k(i as i64);
        let call = self.b.ins().call_indirect(self.rsig, f, &[self.cpu, iv, iv]);
        let v = self.b.inst_results(call)[0];
        self.b.ins().jump(join, &[v.into()]);

        self.b.switch_to_block(join);
        let v = self.b.block_params(join)[0];
        self.set_reg((o.r & 15) as usize, v);
        if o.x & 0x100 != 0 {
            let z = self.b.ins().iconst(types::I64, 0);
            self.b.ins().store(mf(), z, self.cpu, Self::acc_off(i));
            let n = self.b.ins().band_imm_u(m, !((MACSR_PAV0 << i) as i64) & 0xFFFF_FFFF);
            self.b.ins().store(mf(), n, self.cpu, off_m);
        }
    }

    /// Call `o`'s handler with the interpreter's bookkeeping (registers are
    /// the caller's business).
    fn call_handler(&mut self, o: &Op) {
        let h = self.b.ins().iconst(self.ptr, o.h as usize as i64);
        let p = self.b.ins().iconst(self.ptr, o as *const Op as usize as i64);
        self.b.ins().call_indirect(self.hsig, h, &[self.cpu, p]);
    }

    // -- the block ---------------------------------------------------------------

    fn block(&mut self, pc: u32, ops: &[Op], translate: bool) {
        // `bus.pc` labels debug logs with the block's start (see run_ops).
        self.store_i32(pc as i64, offset_of!(Cpu, bus.pc));
        let live = flag_liveness(ops);
        // EMAC state in variables needs the mode to hold for the whole
        // block: so only blocks that call nothing (no handler can change
        // MACSR), and one check on entry, running the block on the
        // interpreter if the mode is another.
        let has_mac = ops.iter().any(|o| matches!(o.k, Kind::Mac { .. } | Kind::MovAcc));
        if translate && has_mac && ops.iter().all(|o| inline_ok(o) || matches!(o.k, Kind::Bcc | Kind::Bra)) {
            use crate::cpu::{MACSR_FI, MACSR_OMC, MACSR_RT, MACSR_SU};
            self.omc = self.mem.macsr & MACSR_OMC != 0;
            let want = MACSR_FI | if self.omc { MACSR_OMC } else { 0 };
            let m = self.b.ins().load(types::I32, mf(), self.cpu, offset_of!(Cpu, macsr) as i32);
            let g = self.b.ins().band_imm_u(m, (MACSR_RT | MACSR_SU | MACSR_FI | MACSR_OMC) as i64);
            let ok = self.b.ins().icmp_imm_u(IntCC::Equal, g, want as i64);
            let go = self.b.create_block();
            let other = self.b.create_block();
            self.b.ins().brif(ok, go, &[], other, &[]);
            self.b.switch_to_block(other);
            self.b.set_cold_block(other);
            let f = self.b.ins().iconst(self.ptr, jit_interp as *const () as usize as i64);
            let p = self.b.ins().iconst(self.ptr, ops.as_ptr() as usize as i64);
            let n = self.k(ops.len() as i64);
            let pcv = self.k(pc as i64);
            self.b.ins().call_indirect(self.isig, f, &[self.cpu, p, n, pcv]);
            self.b.ins().return_(&[]);
            self.b.switch_to_block(go);
            self.mac_cached = true;
            self.b.def_var(self.macsr, m);
            self.macsr_st = St::Clean;
        }
        let last = ops.last().unwrap();
        let self_loop = translate
            && matches!(last.k, Kind::Bcc | Kind::Bra)
            && last.x == pc
            && ops[..ops.len() - 1].iter().all(inline_ok);
        if self_loop {
            return self.loop_block(pc, ops, &live);
        }
        let exit = self.b.create_block();
        let mut at = pc;
        for (i, o) in ops.iter().enumerate() {
            self.i = i;
            self.op_pc = at;
            let next = at.wrapping_add(o.len as u32);
            let last = i + 1 == ops.len();
            if translate && self.op(o, live[i]) {
                if last {
                    self.finish(i + 1, next, None);
                }
            } else if translate && matches!(o.k, Kind::Bcc | Kind::Bra) && last {
                let taken = if o.k == Kind::Bra { None } else { Some(self.cond(o.r)) };
                self.finish(i + 1, next, Some((taken, o.x)));
            } else {
                // A call: the interpreter's bookkeeping, then the handler.
                self.flush();
                self.store_i32(at as i64, offset_of!(Cpu, op_pc));
                self.store_now(i + 1);
                self.store_i32(next as i64, offset_of!(Cpu, pc));
                let h = self.b.ins().iconst(self.ptr, o.h as usize as i64);
                let p = self.b.ins().iconst(self.ptr, o as *const Op as usize as i64);
                self.b.ins().call_indirect(self.hsig, h, &[self.cpu, p]);
                self.forget();
                if last {
                    self.b.ins().jump(exit, &[]);
                } else {
                    let got = self.b.ins().load(types::I32, mf(), self.cpu, offset_of!(Cpu, pc) as i32);
                    let moved = self.b.ins().icmp_imm_u(IntCC::NotEqual, got, next as i32 as i64);
                    let cont = self.b.create_block();
                    self.b.ins().brif(moved, exit, &[], cont, &[]);
                    self.b.switch_to_block(cont);
                }
            }
            at = next;
        }
        self.b.switch_to_block(exit);
        self.b.ins().return_(&[]);
    }

    /// One pass through a self-looping block's ops. -> (taken condition,
    /// None for always; the fall-through pc)
    fn pass(&mut self, pc: u32, ops: &[Op], live: &[(bool, bool)]) -> (Option<Value>, u32) {
        let mut at = pc;
        for (i, o) in ops[..ops.len() - 1].iter().enumerate() {
            self.i = i;
            self.op_pc = at;
            let done = self.op(o, live[i]);
            debug_assert!(done);
            at = at.wrapping_add(o.len as u32);
        }
        let o = ops.last().unwrap();
        self.i = ops.len() - 1;
        self.op_pc = at;
        let taken = if o.k == Kind::Bra { None } else { Some(self.cond(o.r)) };
        (taken, at.wrapping_add(o.len as u32))
    }

    /// A block whose branch goes back to its own start: it goes round in
    /// here, registers in host registers, for as long as the interpreter
    /// would run it back to back -- clock below `run`'s limit and the next
    /// event, no interrupt pending, no cache flush pending -- and returns
    /// with everything written back otherwise. The body is laid down twice:
    /// the first pass loads what it uses, so the second, the loop, finds
    /// it all in variables.
    fn loop_block(&mut self, pc: u32, ops: &[Op], live: &[(bool, bool)]) {
        let n = ops.len() as i64;
        let taken_out = self.b.create_block();
        let fall_out = self.b.create_block();
        let head = self.b.create_block();
        for copy in 0..2 {
            if copy == 1 {
                self.b.switch_to_block(head);
            }
            let (taken, next) = self.pass(pc, ops, live);
            let now = self.b.use_var(self.now);
            let now = self.b.ins().iadd_imm_s(now, n);
            self.b.def_var(self.now, now);
            let again = self.b.create_block();
            match taken {
                Some(c) => {
                    self.b.ins().brif(c, again, &[], fall_out, &[]);
                }
                None => {
                    self.b.ins().jump(again, &[]);
                }
            }
            self.b.switch_to_block(again);
            let ok = self.may_continue(now);
            self.b.ins().brif(ok, head, &[], taken_out, &[]);
            if copy == 1 {
                // Both exits write back the state the passes leave.
                let saved = (self.reg_st, self.sr_st, self.acc_st, self.macsr_st, self.pend);
                self.b.switch_to_block(taken_out);
                self.leave(pc);
                (self.reg_st, self.sr_st, self.acc_st, self.macsr_st, self.pend) = saved;
                self.b.switch_to_block(fall_out);
                self.leave(next);
            }
        }
    }

    /// Would the interpreter run another block straight away? (Clock
    /// below run's limit and the next event; no interrupt; no flush.)
    fn may_continue(&mut self, now: Value) -> Value {
        let lim = self.b.ins().load(types::I64, mf(), self.cpu, offset_of!(Cpu, limit) as i32);
        let dl = self.b.ins().load(types::I64, mf(), self.cpu, offset_of!(Cpu, bus.io.deadline) as i32);
        let a = self.b.ins().icmp(IntCC::UnsignedLessThan, now, lim);
        let b2 = self.b.ins().icmp(IntCC::UnsignedLessThan, now, dl);
        let irq = self.b.ins().uload8(types::I32, mf(), self.cpu, offset_of!(Cpu, bus.io.irq_level) as i32);
        let quiet = self.b.ins().icmp_imm_u(IntCC::Equal, irq, 0);
        let fp = self.b.ins().uload8(types::I32, mf(), self.cpu, offset_of!(Cpu, bus.flush_pending) as i32);
        let no_flush = self.b.ins().icmp_imm_u(IntCC::Equal, fp, 0);
        let x = self.b.ins().band(a, b2);
        let y = self.b.ins().band(quiet, no_flush);
        self.b.ins().band(x, y)
    }

    /// Write everything back with the clock as it stands, set pc, return.
    fn leave(&mut self, pc: u32) {
        self.flush();
        self.store_now(0);
        self.store_i32(pc as i64, offset_of!(Cpu, pc));
        self.b.ins().return_(&[]);
    }

    /// End the block after `done` ops: write everything back and set pc
    /// (a branch: its target if taken; None for taken always).
    fn finish(&mut self, done: usize, next: u32, branch: Option<(Option<Value>, u32)>) {
        let pcv = match branch {
            None => self.k(next as i64),
            Some((None, t)) => self.k(t as i64),
            Some((Some(c), t)) => {
                let tv = self.k(t as i64);
                let nv = self.k(next as i64);
                self.b.ins().select(c, tv, nv)
            }
        };
        self.flush();
        self.store_now(done);
        self.b.ins().store(mf(), pcv, self.cpu, offset_of!(Cpu, pc) as i32);
        self.b.ins().return_(&[]);
    }
}
