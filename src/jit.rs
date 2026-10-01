//! A block compiler: hot predecoded blocks become native code (Cranelift).
//!
//! A compiled block does exactly what `Cpu::run_ops` does for it, in the
//! same order -- set `op_pc`, advance the clock by one, preset `pc` to the
//! next instruction, run the operation, leave if `pc` moved -- so the
//! machine's state after a block is the same bit for bit whichever way it
//! ran, and `jitcheck` can hold the two to that. Operations it does not
//! translate itself are calls to their `fast` handlers.
//!
//! The code refers to its block's `Op`s by address, so it lives exactly as
//! long as the block cache does: a flush (`Bus::icache_settle`) drops every
//! block, and the next block run here starts a fresh code module.

use crate::cpu::Cpu;
use crate::fast::Op;
use cranelift_codegen::ir::condcodes::IntCC;
use cranelift_codegen::ir::{types, AbiParam, InstBuilder, MemFlagsData, Signature, UserFuncName};
use cranelift_codegen::settings::{self, Configurable};
use cranelift_codegen::Context;
use cranelift_frontend::{FunctionBuilder, FunctionBuilderContext};
use cranelift_jit::{JITBuilder, JITModule};
use cranelift_module::Module;
use std::mem::offset_of;

/// A compiled block.
pub type BlockFn = unsafe extern "C" fn(*mut Cpu);

/// Runs of a block before it is compiled.
pub const HOT: u32 = 16;

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
}

fn new_module() -> JITModule {
    let mut flags = settings::builder();
    flags.set("opt_level", "speed").unwrap();
    flags.set("use_colocated_libcalls", "false").unwrap();
    flags.set("is_pic", "false").unwrap();
    let isa = cranelift_native::builder()
        .expect("host machine not supported by Cranelift")
        .finish(settings::Flags::new(flags))
        .unwrap();
    JITModule::new(JITBuilder::with_isa(isa, cranelift_module::default_libcall_names()))
}

impl Jit {
    pub fn new(epoch: u64) -> Jit {
        let module = new_module();
        let ctx = module.make_context();
        Jit { module: Some(module), ctx, fctx: FunctionBuilderContext::new(), epoch, compiled: 0, failed: 0, compile_secs: 0.0 }
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
    pub fn compile(&mut self, pc: u32, ops: &[Op]) -> Option<BlockFn> {
        let t = std::time::Instant::now();
        let f = self.build(pc, ops);
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

    fn build(&mut self, pc: u32, ops: &[Op]) -> Option<BlockFn> {
        let module = self.module.as_mut()?;
        let ptr = module.target_config().pointer_type();
        let call_conv = module.target_config().default_call_conv;
        let mut sig = Signature::new(call_conv);
        sig.params.push(AbiParam::new(ptr));
        self.ctx.func.signature = sig.clone();
        let id = module.declare_anonymous_function(&sig).ok()?;
        self.ctx.func.name = UserFuncName::user(0, id.as_u32());

        let mut hsig = Signature::new(call_conv);
        hsig.params.push(AbiParam::new(ptr));
        hsig.params.push(AbiParam::new(ptr));

        let off_op_pc = offset_of!(Cpu, op_pc) as i32;
        let off_pc = offset_of!(Cpu, pc) as i32;
        let off_now = offset_of!(Cpu, bus.io.now) as i32;
        let off_bus_pc = offset_of!(Cpu, bus.pc) as i32;
        let tc = module.target_config();

        {
            let mut b = FunctionBuilder::new(&mut self.ctx.func, &mut self.fctx);
            let hsig = b.import_signature(hsig);
            // Accesses to the Cpu struct: aligned, never trapping.
            let mf = MemFlagsData::trusted();
            let entry = b.create_block();
            let exit = b.create_block();
            b.append_block_params_for_function_params(entry);
            b.switch_to_block(entry);
            let cpu = b.block_params(entry)[0];
            // `bus.pc` labels debug logs with the block's start (see run_ops).
            let v = b.ins().iconst(types::I32, pc as i64);
            b.ins().store(mf, v, cpu, off_bus_pc);
            let mut at = pc;
            for (i, op) in ops.iter().enumerate() {
                let next = at.wrapping_add(op.len as u32);
                let v = b.ins().iconst(types::I32, at as i64);
                b.ins().store(mf, v, cpu, off_op_pc);
                let now = b.ins().load(types::I64, mf, cpu, off_now);
                let now = b.ins().iadd_imm_s(now, 1);
                b.ins().store(mf, now, cpu, off_now);
                let nv = b.ins().iconst(types::I32, next as i64);
                b.ins().store(mf, nv, cpu, off_pc);
                let h = b.ins().iconst(ptr, op.h as usize as i64);
                let o = b.ins().iconst(ptr, op as *const Op as usize as i64);
                b.ins().call_indirect(hsig, h, &[cpu, o]);
                if i + 1 < ops.len() {
                    let got = b.ins().load(types::I32, mf, cpu, off_pc);
                    let moved = b.ins().icmp_imm_u(IntCC::NotEqual, got, next as i64);
                    let cont = b.create_block();
                    b.ins().brif(moved, exit, &[], cont, &[]);
                    b.switch_to_block(cont);
                }
                at = next;
            }
            b.ins().jump(exit, &[]);
            b.switch_to_block(exit);
            b.ins().return_(&[]);
            b.seal_all_blocks();
            b.finalize(tc);
        }
        let ok = module.define_function(id, &mut self.ctx).is_ok();
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
