//! Peripheral dispatch. Filled in by the peripheral models.

pub struct Io {
    /// Highest pending interrupt level (0 = none); the CPU samples it.
    pub irq_level: u8,
}

impl Io {
    pub fn new() -> Io {
        Io { irq_level: 0 }
    }

    /// Acknowledge the highest pending interrupt above `ipl`. -> (vector, level)
    pub fn ack_irq(&mut self, _ipl: u8) -> Option<(u8, u8)> {
        None
    }

    /// -> Some(value) when a model answers the read. `plain` is what the
    /// address would read as memory.
    pub fn read(&mut self, _a: u32, _size: u32, _plain: u32, _pc: u32, _ddr: &mut [u8]) -> Option<u32> {
        None
    }

    /// -> true when a model consumed the write (it is then not stored).
    pub fn write(&mut self, _a: u32, _size: u32, _v: u32, _pc: u32, _ddr: &mut [u8]) -> bool {
        false
    }
}

impl Default for Io {
    fn default() -> Self {
        Self::new()
    }
}
