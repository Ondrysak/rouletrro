# rouletrro — a Digitakt (MK1) emulator in Rust

`dtemu` runs the Elektron Digitakt's own operating system, unmodified,
on emulated hardware: a ColdFire V4e CPU with its EMAC unit, the
peripherals the OS drives, the eMMC card with its +Drive filesystem, and
the front-panel microcontroller. You get the OLED, the key LEDs, the keys
and encoders, and live audio from the Digitakt's sample engine.

It reimplements, in Rust, the machine described in
[digiemu's DIGITAKT-MK1.md](https://github.com/irpina/digiemu/blob/main/DIGITAKT-MK1.md).

**No firmware is included.** You need an OS update file from Elektron
(`Digitakt_OS1.53.syx`). The MAIN OS inside it is checked by hash; other
versions are refused, because the emulator uses a few addresses specific to
that build. Firmware and everything derived from it (extracted sections,
snapshots, card images) are git-ignored and must not be committed.

## Build

```sh
cargo build --release
```

On Linux, sound (cpal) builds against ALSA: install `pkg-config` and
`libasound2-dev` or your distribution's equivalent. The window uses X11
(minifb). The emulator itself has no platform-specific code.

Put the firmware at `fw/Digitakt_OS1.53.syx` (the default path) or pass
its path as the first argument to any of the tools.

## Play

```sh
# A missing image is created, with the sample area formatted.
./target/release/digitakt --card digitakt.img --snapshot digitakt.snap
```

The first start boots from reset: splash screens, then, on a new card,
the OS copies its factory project to the +Drive (about half a minute). On exit (or with
F5) the machine is saved to `--snapshot` and the card to `--card`; the next
start resumes from the snapshot instantly.

Controls:

| Action | Mouse |
| --- | --- |
| Press a key | click (held while the button is down) |
| Latch a key (to hold FUNC, a trig, ...) | shift-click; again to release |
| Turn an encoder | wheel over it, or drag it up/down |
| Push an encoder (A–H) | click it |
| Release everything | Esc |
| Save snapshot and card | F5 |

Options:

- `--ips N`: instructions per emulated second (default `200M`). The OS's
  audio render needs about 127M executed instructions per second of
  audio. Lower values starve it. With the block compiler a 2.8 GHz Xeon
  runs playback at about 150% of real time (the interpreter alone: 80%).
- `--latency MS`: audio kept buffered ahead of the output (default 60).
  Raise it if a busy machine gives dropouts.
- `--no-audio`: run without sound, paced by the wall clock.

The status line shows the buffered audio, dropouts, and emulation speed
(100% is real time; a machine that cannot keep up shows less).

### Samples

`dtcard` writes WAV files to the card's `/incoming` directory, where the
OS's sample browser (SETTINGS > SAMPLES) finds them:

```sh
./target/release/dtcard digitakt.img add kick.wav snare.wav
./target/release/dtcard digitakt.img list
./target/release/dtcard digitakt.img format      # erases the samples (only)
```

The samples live in their own area of the card, formatted when the
emulator creates the image (on the hardware the factory does it, or FORMAT
+DRIVE); `format` recreates it and touches nothing else. An image from an
older version of the emulator may lack it: run `format` once.

Do this while the emulator is not running, then start once without
`--snapshot` so the OS cold-boots and mounts the image as it is. A
snapshot carries its own copy of the card (the OS's view of the drive must
match it), and resuming one runs that copy. If the image differs from it,
the emulator warns, and before writing the card back it keeps the image as
`IMAGE.bak`. WAV files (8-32 bit integer or
32-bit float, any channel count) are mixed to 16-bit mono; the sample rate
is kept in the sample's header.

### Headless runs

The GUI binary runs without a window too, which is handy for scripted
tests:

```sh
./target/release/digitakt --headless --audio-null --snapshot s.snap --after 6 \
    --press 24@1 --press 24@2 --wav out.wav --screenshot out.png
```

`--press CODE@SECS` presses a key at a wall time (held 150 ms);
`--turn ENC:DELTA@SECS` turns an encoder. Key codes:

| Code | Key | Code | Key | Code | Key |
| --- | --- | --- | --- | --- | --- |
| 1 | FUNC | 9 | RECORD | 17 | RIGHT |
| 2 | TRK | 10 | PLAY | 18 | PAGE |
| 3 | PTN | 11 | STOP | 19 | TRIG |
| 4 | BANK | 12 | YES | 20 | SRC |
| 5 | SONG | 13 | NO | 21 | FLTR |
| 6 | GLOBAL | 14 | UP | 22 | AMP |
| 7 | SAMPLE | 15 | DOWN | 23 | LFO |
| 8 | TEMPO | 16 | LEFT | 24–39 | trigs 1–16 |
| | | | | 40–47 | encoder pushes A–H |

Encoders: 0–7 are A–H, 8 is LEVEL/DATA.

## Tools

| Binary | Purpose |
| --- | --- |
| `digitakt` | the emulator with its panel window |
| `fwinfo` | list a firmware's sections and hashes; `-o DIR` extracts them |
| `dtboot` | headless runner for debugging: instruction budgets, breakpoints, watchpoints, guest profiling, snapshots, scripted input (`--help`) |
| `dtpeek` | inspect a snapshot: DMA descriptors, memory, LEDs, UART traffic, the frame buffer |
| `dtcard` | format a card image, add samples, list and check the +Drive |
| `cfdiff` | the CPU side of the differential tester below |
| `jitcheck` | runs a snapshot (or a cold boot) on the interpreter and the block compiler side by side and compares the whole machine at every checkpoint |

## How it works

- **`firmware`**: SysEx 8-in-7 decoding, the ELE3 container, and the
  section depacker. MAIN OS (section 3) loads at 0x40000400.
- **`cpu`**: the ColdFire V4e interpreter: ISA A/B/C and the EMAC with its
  accumulator modes, exceptions, and supervisor state. It is the reference
  for everything else.
- **`fast`**: a predecoded basic-block cache over the OS text. Each
  instruction is decoded once into a handler with resolved operands; stores
  into cached code invalidate it. Anything not predecoded runs on the
  interpreter.
- **`jit`**: a block compiler (Cranelift). A block that runs hot becomes
  native code that leaves the machine exactly as the interpreter would,
  bit for bit: guest registers, SR, MACSR and the accumulators live in
  host registers within a block, condition codes are computed lazily, DDR
  and SRAM accesses are inline, and a block that branches to itself loops
  natively for as long as the interpreter would run it back to back.
  Operations it does not translate are calls to their handlers. Blocks
  compile on a background thread (a block runs interpreted until its code
  arrives), so new code never stalls the audio, and compiled blocks run
  one after another without returning to the run loop while nothing else
  is due. About twice the interpreter's speed; `DTEMU_NO_JIT=1` turns it
  off, `DTEMU_JIT_SYNC=1` compiles in line.
- **`bus`**: 128 MB DDR (with aliases), 64 KB SRAM, sparse memory for
  everything else, and the peripheral models.
- **`io`**, **`edma`**, **`esdhc`**, **`panel`**: the interrupt controllers,
  PIT and DMA timers, UARTs, the 64-channel eDMA, the SSI audio port, the
  eSDHC with an eMMC the OS accepts, and the panel MCU's UART protocol
  (OLED tiles, LED palettes and selectors, key and encoder input).
- **`machine`**: boot setup, a high-level stand-in for the SPI flash read,
  idle-loop fast-forwarding, and the input helpers.
- **`ekfs`**: the +Drive's filesystem (lookup3 checksums, hashed and
  sorted directory indexes, the sample format), byte-identical to the
  reference implementation.
- **`snapshot`**: the whole machine, DDR compressed with LZ4, tied to the
  MAIN OS hash.

Everything runs on one thread against one instruction clock: the CPU runs
a block, then timers, the 48 kHz audio frame clock and DMA advance, and
interrupts are sampled between instructions. The window and the sound
card run on their own threads, fed through a ring buffer.

## Testing

```sh
cargo test --release              # unit tests, EMAC reference cases
pip install unicorn capstone
python3 tools/cfdiff.py 4000      # random instructions against Unicorn's M68K
CFDIFF_JIT=1 python3 tools/cfdiff.py 4000   # the same through the block compiler
./target/release/jitcheck snapshots/x.snap --instr 1G   # compiler vs interpreter
./target/release/fwinfo fw/Digitakt_OS1.53.syx -o sections   # extract, then:
python3 tools/cfdis.py 40075e00 +100   # disassemble MAIN OS
```

`cfdiff` runs random instruction sequences on both cores and compares
registers and memory; it fails on any difference but the known one (MVZ,
which clears N per the ColdFire manual where Unicorn sets it). The block
compiler is held to the interpreter by `jitcheck`: the machine is
deterministic, so registers, clock, DDR, SRAM and every peripheral must
match at each checkpoint. CI runs the build, the tests (including the
randomized EMAC cases through the compiler), clippy and `cfdiff` both ways
on every push; none of it needs the firmware.

## Status

Working: cold boot to the main UI, the +Drive (format, mount, sample
browser, loading samples), sequencer playback with audio out, the panel
LEDs and OLED, save/restore.

Not modelled: MIDI, USB, the audio inputs, the Overbridge link, and the
panel MCU's own firmware (its protocol is modelled instead).
