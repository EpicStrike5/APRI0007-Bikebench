# APRI0007.X — PIC18F47Q84 Servo + Hall Sensor Controller

E-bike gear shift controller. A servo motor is driven by a frequency-variable signal
(200–900 Hz, 50% duty cycle) whose frequency changes via two push-buttons. A hall
sensor counts wheel pulses and reports a rolling average every ~3 seconds.

---

## Hardware Summary

| Pin | Function | Direction |
|-----|----------|-----------|
| RC2 | Servo PWM output | Output |
| RB0 | Button 1 — increase frequency (RIGHT) | Input |
| RB1 | Button 2 — decrease frequency (LEFT) | Input |
| RB2 | Hall sensor input (INT2, rising edge) | Input |
| RD0 | Button feedback LED | Output |
| RD1 | Debug: Servo_Init completed (SERVO_DEBUG=1) | Output |
| RD2 | Debug: Servo ISR heartbeat (SERVO_DEBUG=1) | Output |

**Processor:** PIC18F47Q84 @ 64 MHz (HFINTOSC_64MHZ, internal oscillator)
**Toolchain:** MPLAB X IDE + pic-as assembler
**Required assembler flag:** `-xassembler-with-cpp` (enables `#define` / `#if`)
**Required linker option:** `-pivt=08h` (places the IVT at address 0x08)
**Build:** always use *Production → Clean and Build Main Project* (never just Build —
MPLAB X may silently reuse a stale `.hex` otherwise)

---

## Assembly Concepts Explained

These patterns appear throughout every file. Understanding them once makes the
rest of the code straightforward.

### Memory banks and BANKSEL

The PIC18F47Q84 has 4 KB of RAM and its Special Function Registers (SFRs — the
registers that control timers, ports, etc.) spread across several 256-byte *banks*.
A hardware register called BSR (Bank Select Register) decides which bank is
currently active. Before touching any SFR you must make sure BSR points at the
right bank, otherwise you read or write the wrong register entirely.

```asm
BANKSEL LATC          ; compiler sets BSR to the bank that contains LATC
bsf     LATC, 2       ; now safe to access LATC
```

`BANKSEL` expands to one or two `movlb` instructions that load BSR. It costs 1–2
instruction cycles. **Never assume BSR is still set from a previous BANKSEL** —
an interrupt can fire between any two instructions and change BSR.

### BANKMASK

When an SFR address is used directly as an operand (not just named), the assembler
needs only the *offset within the bank* (low 8 bits), not the full address. The
`BANKMASK()` macro strips the upper bits so the instruction encoding is correct:

```asm
bcf     BANKMASK(PIR3), 4, 1    ; clears bit 4 of PIR3, banked access
```

The `, 1` at the end is the *access mode*: `1` = banked (use BSR), `c` or `0` = access
bank (direct, no BSR needed). See the next section.

### Access bank (udata_acs)

The lowest 96 bytes of RAM (0x000–0x05F) and the highest 160 bytes of SFR space
(0xF60–0xFFF) are always reachable without touching BSR — this region is called
the *Access Bank*. Variables declared in `PSECT udata_acs` land here.

```asm
PSECT udata_acs
my_var: DS 1           ; 1 byte, placed in access bank automatically

    movf    my_var, w, c    ; c = access bank, no BANKSEL needed
```

Using the access bank for frequently-touched variables (ISR state, counters) avoids
expensive `BANKSEL` instructions inside time-critical interrupt handlers.

### PSECT (Program Section)

A `PSECT` is a named chunk of code or data that the *linker* places at the right
address. You declare what *class* and *alignment* it needs; the linker does the
rest.

```asm
PSECT myCode, class=CODE, reloc=4
```

| Attribute | Meaning |
|-----------|---------|
| `class=CODE` | Lives in program flash memory |
| `class=UDATA_ACS` | Lives in the access-bank RAM region |
| `reloc=2` | Must be placed at an address divisible by 2 (word-aligned) |
| `reloc=4` | Must be placed at an address divisible by 4 (required for ISR entry points) |
| `ovrld` | Multiple files can define a PSECT with the same name and the linker merges them (used for the IVT so each `.inc` file can add its own entries) |

### IVT — Interrupt Vector Table

With `MVECEN = ON` (multi-vector interrupts), each interrupt source has a fixed
slot in a table starting at address `0x08`. When an interrupt fires, the CPU
looks up the ISR address in that table directly — there is no shared ISR that
polls flags.

Each slot is 2 bytes wide and holds **the ISR address divided by 4** (because ISRs
must be 4-byte aligned, the bottom 2 bits are always zero and can be omitted):

```asm
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     28*2          ; slot for IRQ 28 = Timer1 overflow
    DW      Servo_ISR >> 2   ; address / 4
```

The CPU left-shifts this value by 2 to recover the full ISR address before jumping.

Every `.inc` file that owns an interrupt adds its own `PSECT ivt … ovrld` block.
`ovrld` tells the linker to merge all these blocks into one table instead of
treating duplicate names as an error.

### retfie 1

`retfie 1` returns from an interrupt and simultaneously restores W, STATUS and BSR
from *shadow registers* that the CPU saved automatically on entry. This is the
standard way to end a vectored ISR — no manual save/restore needed.

### GIE and GIEL

Two global enable bits in `INTCON0` must both be set before any interrupt can fire:

| Bit | Name | Purpose |
|-----|------|---------|
| Bit 7 | GIE / GIEH | Master interrupt enable |
| Bit 6 | GIEL | Required with MVECEN=ON (even if not using priority levels) |

Both are set in `main.asm` after all `_Init` calls return.

---

## File Structure

```
main.asm          — entry point, init sequence, main loop
pinconfig.inc     — all pin directions, pull-ups, output latches, PPS routing
wait.inc          — blocking delay routines (ms / s)
debounce.inc      — Timer2 hardware debounce for push-buttons
servo_hw.inc      — Timer1 frequency-variable servo drive
hall.inc          — INT2 + Timer0 hall sensor pulse counter
can_torque.inc    — CAN FD module setup (not yet active)
```

---

## Module Reference

---

### `pinconfig.inc` — Centralised Pin Configuration

**Purpose:** Configure every I/O pin in one place so peripheral `.inc` files
never fight over register access and the full pin map is easy to audit.

**Hardware used:** No timers or interrupts. Writes to ANSELx, WPUx, LATx, TRISx
(all in bank 4) and PPS registers (bank 2).

**Public function:**

| Function | Description |
|----------|-------------|
| `PinConfig_Init` | Call once at startup, before any other `_Init` routine |

**What it does — step by step:**

1. **ANSELx = 0** — Every pin defaults to *analog* after reset. This must be
   cleared to 0 to make pins digital before reading or writing them.
2. **WPUx** — Weak pull-up resistors are enabled on RB0 and RB1 (buttons) only.
   All others are off. The pull-ups ensure button inputs read HIGH (released)
   when nothing is pressed.
3. **LATx = 0** — Output latches are zeroed *before* the direction registers are
   changed. This prevents a glitch where a pin briefly drives a random value
   while TRIS is being written.
4. **TRISx** — Direction registers: `1` = input, `0` = output.
   - `TRISC = 0xFB` (1111 1011) → RC2 is output, all others input
   - `TRISD = 0xF8` (1111 1000) → RD0, RD1, RD2 are outputs
5. **PPS** — Peripheral Pin Select routes peripheral signals to physical pins.
   Currently no PPS is active (servo uses direct LATC toggle). Commented blocks
   show how to enable CAN, UART, and SPI when needed.

**To reuse:** No parameters. Just call `PinConfig_Init` first and edit the pin
table at the top of the file when adding hardware.

---

### `wait.inc` — Blocking Delay Routines

**Purpose:** Simple busy-wait delays used for startup sequences and the main-loop
20 ms poll interval.

**Hardware used:** No timers. Pure instruction counting. Interrupts still fire
during a wait (the ISRs preempt the loops transparently).

**Public functions:**

| Function | Input | Delay |
|----------|-------|-------|
| `waitMilliSeconds` | W = 1–255 | W × 1 ms (±1 cycle) |
| `waitSeconds` | W = 1–255 | W × 1 s (calls waitMilliSeconds × 4) |

**How to call:**

```asm
movlw   250
call    waitMilliSeconds    ; wait 250 ms
```

**How it works:** Nested counter loops calibrated for 64 MHz (16 MIPS). The
inner-most loop takes exactly 5 cycles per iteration. The outer loops are padded
with `nop` instructions to hit 16 000 cycles = 1 ms. The cycle budget is
documented inline in the source file.

**Variables used (access bank):** `waitMSeconds`, `waitInner`, `waitInnerMost`,
`waitPad`, `waitSecondsValue` — 5 bytes total.

---

### `debounce.inc` — Hardware Button Debounce

**Purpose:** Eliminate contact bounce from push-buttons so that one physical
press produces exactly one logical edge. Result is available in `db_stable`.

**Hardware used:** Timer2 (IRQ 27, PIR3 bit 3 / PIE3 bit 3)

**Timer2 configuration:**

| Parameter | Value | Effect |
|-----------|-------|--------|
| Clock source | Fosc/4 = 16 MHz | — |
| Prescaler | 1:128 | 125 kHz → 8 µs/tick |
| Period register (T2PR) | 124 | (124+1) × 8 µs = 1 ms |
| Postscaler | 1:10 | Interrupt every 10 ms |

**Public interface:**

| Symbol | Type | Description |
|--------|------|-------------|
| `Debounce_Init` | Function | Configure Timer2, seed initial state, enable IRQ |
| `db_stable` | Variable (access bank) | Current debounced button state (read this) |

`db_stable` bit layout: bit 0 = RB0, bit 1 = RB1. Value follows active-LOW
convention: `0` = pressed, `1` = released.

**How to use:**

```asm
call    Debounce_Init
; ... other inits ...
; enable GIE/GIEL
; then in your loop:
movf    db_stable, w, c    ; read debounced state
```

**Algorithm:** The ISR samples PORTB every 10 ms. If the sample matches the
previous sample, a counter increments. If the counter reaches 3 (30 ms of
stable consecutive reads) `db_stable` is updated. Any mismatch resets the counter
to zero and restarts timing. This means a bouncing contact must settle for 30 ms
before the state change is accepted.

**IVT entry:** IRQ 27 → `PSECT ivt`, `ORG 27*2`, `DW Debounce_ISR >> 2`

**Variables used (access bank):** `db_stable`, `db_prev`, `db_count` — 3 bytes.

---

### `servo_hw.inc` — Timer1 Frequency-Variable Servo Drive

**Purpose:** Generate a 50% duty-cycle square wave on RC2. The servo motor
responds to the *frequency* of this signal (tested with a function generator):
higher frequency → higher position.

**Hardware used:** Timer1 (IRQ 28, PIR3 bit 4 / PIE3 bit 4), pin RC2

**Timer1 configuration:**

| Parameter | Value | Effect |
|-----------|-------|--------|
| Clock source | Fosc/4 | 16 MHz |
| Prescaler | 1:8 | 2 MHz → 0.5 µs/tick |
| Mode | 16-bit, overflow interrupt | — |
| T1CON | 0x33 | RD16=1 (16-bit), prescaler 1:8, enabled |

**Frequency range:**

| `servo_pos` | Preload | Half-period | Output frequency |
|-------------|---------|-------------|-----------------|
| 0 | 60 536 | 5 000 ticks = 2.5 ms | 200 Hz |
| 128 | 62 456 | 3 080 ticks = 1.54 ms | 325 Hz |
| 255 | 64 361 | 1 175 ticks = 0.59 ms | ~851 Hz |

RC2 is toggled on *every* Timer1 overflow, so the duty cycle is always exactly
50% regardless of position.

**Preload formula:** `preload = 60536 + pos × 15`

`pos × 15` is computed as `pos × 16 − pos`:
- Four left-shifts give `pos × 16` (each shift = ×2)
- Then `subwf` subtracts `pos`, with `btfss STATUS,0` + `decf` propagating the
  borrow into the high byte

**Public functions:**

| Function | Description |
|----------|-------------|
| `Servo_Init` | Configure Timer1, load default position (128), start ISR |
| `Servo_SetPos` | W = new position (0–255), reloads preload with interrupts off |
| `Servo_IncPos` | Add `SERVO_STEP` to position, clamp at 255 |
| `Servo_DecPos` | Subtract `SERVO_STEP` from position, clamp at 0 |

**Constants you can change:**

| `#define` | Default | Meaning |
|-----------|---------|---------|
| `SERVO_DEBUG` | 1 | 1 = enable startup pulses + debug LEDs |
| `SERVO_STEP` | 20 | Position units per button press |
| `SERVO_DEFAULT` | 128 | Starting position (~325 Hz) |
| `SERVO_PRELOAD_BASE` | 60536 | Preload for pos=0 (200 Hz) |

**SERVO_DEBUG=1 diagnostic behaviour:**
- Two 250 ms HIGH pulses on RC2 at startup (verifies pin and wiring before Timer1)
- RD1 ON steady after `Servo_Init` returns (Timer1 is running)
- RD2 dim glow = ISR is firing (toggled every overflow, appears as ~50% PWM on LED)

Set `SERVO_DEBUG` to 0 only after the signal is confirmed working on a scope.

**IVT entry:** IRQ 28 → `PSECT ivt`, `ORG 28*2`, `DW Servo_ISR >> 2`

**ISR bank sequence (every Timer1 overflow):**
1. `BANKSEL PIR3` → clear TMR1IF (bit 4)
2. `BANKSEL LATC` → `btg LATC, 2` (toggle RC2)
3. `BANKSEL TMR1L` → reload preload (TMR1L first because RD16=1 buffers the write)

Each `BANKSEL` is explicit because BSR cannot be assumed after the flag clear.

**Variables used (access bank):** `servo_pos`, `servo_ticks_l`, `servo_ticks_h`
— 3 bytes.

---

### `hall.inc` — Hall Sensor Pulse Counter

**Purpose:** Count rising-edge pulses from a hall sensor on RB2. Every ~3 seconds
capture the accumulated count (16-bit) into `hall_count_h:hall_count_l` and reset
for the next interval. Intended for computing wheel speed or cadence.

**Hardware used:**
- INT2 external interrupt (IRQ 80, PIR10 bit 0 / PIE10 bit 0), pin RB2
- Timer0 (IRQ 31, PIR3 bit 7 / PIE3 bit 7)

**Timer0 configuration:**

| Parameter | Value | Effect |
|-----------|-------|--------|
| Mode | 8-bit (T0CON0 bit 3 = 0) | Overflow at 256 |
| Clock source (T0CON1 bits 7–5) | 010 = Fosc/4 | 16 MHz input |
| Prescaler (T0CON1 bits 3–0) | 1111 = 1:32768 | 488 Hz tick rate |
| Overflow period | 256 / 488 Hz | ~0.524 s |
| Capture interval | 6 overflows | ~3.1 s |

**INT2 configuration:**

- `INTCON0 bit 5` (INT2EDG) = 1 → trigger on **rising** edge
- `PIR10 bit 0` = INT2IF flag, cleared by ISR
- `PIE10 bit 0` = INT2IE enable

**Public interface:**

| Symbol | Type | Description |
|--------|------|-------------|
| `Hall_Init` | Function | Configure RB2, INT2, Timer0; enable both IRQs |
| `hall_count_l` | Variable (access bank) | Most recent captured count, low byte |
| `hall_count_h` | Variable (access bank) | Most recent captured count, high byte |

**How to read the count:**

```asm
; Disable interrupts while reading to avoid a partial update
BANKSEL INTCON0
bcf     BANKMASK(INTCON0), 7, 1     ; GIE off
movf    hall_count_l, w, c          ; read low byte into W
movf    hall_count_h, w, c          ; read high byte into W (or store separately)
BANKSEL INTCON0
bsf     BANKMASK(INTCON0), 7, 1     ; GIE on
```

**How the counting works:**

- Every rising edge on RB2 fires `Hall_INT2_ISR` → increments `hall_counter`
  (8-bit). When `hall_counter` wraps 255→0, `hall_overflow` increments.
  Full count = `hall_overflow × 256 + hall_counter`.
- Every Timer0 overflow fires `Hall_TMR0_ISR` → increments `hall_tmr0_ticks`.
  When `hall_tmr0_ticks` reaches `HALL_TMR0_INTERVAL` (6), the full count is
  copied to `hall_count_h:hall_count_l`, all counters reset to zero.

**Constant you can change:**

| `#define` | Default | Meaning |
|-----------|---------|---------|
| `HALL_TMR0_INTERVAL` | 6 | Timer0 overflows per capture (~3.1 s) |

**IVT entries:**
- IRQ 80 → `ORG 80*2`, `DW Hall_INT2_ISR >> 2`
- IRQ 31 → `ORG 31*2`, `DW Hall_TMR0_ISR >> 2`

**Variables used (access bank):** `hall_counter`, `hall_overflow`,
`hall_tmr0_ticks`, `hall_count_l`, `hall_count_h` — 5 bytes.

---

### `can_torque.inc` — CAN FD Module Setup *(not yet active)*

**Purpose:** Configure the CAN FD peripheral to receive torque messages.

**Status:** `CAN_Init` is never called in `main.asm`. The file is included for
linking purposes only (its variables occupy access bank space but the code is
dormant).

**Hardware used:** CAN FD module. PPS would route CANTX → RB0, CANRX ← RB1
(conflicts with current button pins — must be resolved before enabling).

**Public functions:** `CAN_Init`, `CAN_IsRxReady`, `CAN_Receive`,
`CAN_GetMsgID`, `CAN_GetDLC`, `CAN_Transmit`

**Variables used (access bank):** `can_rx_buffer[8]`, `can_msg_id_l`,
`can_msg_id_h`, `can_dlc`, `can_temp`, `can_fifo_ptr_l`, `can_fifo_ptr_h`
— 14 bytes.

---

### `main.asm` — Entry Point and Main Loop

**Purpose:** Tie all libraries together: init sequence, interrupt enable,
then a 20 ms polling loop that detects button edges and calls servo handlers.

**Init sequence:**

```
PinConfig_Init          (pin directions, pull-ups, latches)
  ↓
startup LED blink       (confirms chip is alive)
  ↓
Debounce_Init           (Timer2, 10ms ISR)
seed btn_prev           (avoid spurious edges on first loop)
  ↓
Servo_Init              (Timer1, SERVO_DEBUG pulses)
  ↓
Hall_Init               (INT2 + Timer0)
  ↓
GIE = 1, GIEL = 1       (all interrupts now live)
```

**Main loop — edge detection:**

The loop runs every 20 ms. Each iteration:

1. `W = db_stable XOR btn_prev` — find bits that changed
2. `W = W AND btn_prev` — keep only bits that were HIGH (released) and are now LOW
   (pressed) — i.e., just-pressed edges
3. `btfss btn_edge, 0` — if bit is set (just pressed), call `handleButton1`
4. `btfss btn_edge, 1` — same for button 2

`btfss` *skips the next instruction when the bit is set*, so the `call` executes
when the bit is **not** set — this means the call fires on the falling-through path.
The logic is intentionally inverted here (pre-existing behaviour, do not change).

**Button handlers:**

| Button | Handler | Action |
|--------|---------|--------|
| RB0 (pressed) | `handleButton1` | `Servo_IncPos` + LED ON |
| RB1 (pressed) | `handleButton2` | `Servo_DecPos` + LED OFF |

---

## Access Bank Memory Map

All variables are in the access bank (0x000–0x05F, 96 bytes max).

| Module | Variables | Bytes |
|--------|-----------|-------|
| `wait.inc` | 5 loop counters | 5 |
| `debounce.inc` | `db_stable`, `db_prev`, `db_count` | 3 |
| `servo_hw.inc` | `servo_pos`, `servo_ticks_l/h` | 3 |
| `hall.inc` | `hall_counter`, `hall_overflow`, `hall_tmr0_ticks`, `hall_count_l/h` | 5 |
| `can_torque.inc` | rx buffer + metadata | 14 |
| `main.asm` | `btn_prev`, `btn_edge` | 2 |
| **Total** | | **32 / 96 bytes** |

---

## IRQ Assignment Summary

| IRQ | Source | PIR | PIE | Handler |
|-----|--------|-----|-----|---------|
| 27 | Timer2 overflow | PIR3 bit 3 | PIE3 bit 3 | `Debounce_ISR` |
| 28 | Timer1 overflow | PIR3 bit 4 | PIE3 bit 4 | `Servo_ISR` |
| 31 | Timer0 overflow | PIR3 bit 7 | PIE3 bit 7 | `Hall_TMR0_ISR` |
| 80 | INT2 (RB2) | PIR10 bit 0 | PIE10 bit 0 | `Hall_INT2_ISR` |

IVT base address: `0x08` (set by linker option `-pivt=08h`).
IVT slot address = `0x08 + IRQ × 2`.

---

## Common Pitfalls

**Stale build:** MPLAB X sometimes reports `up to date` after editing a `.inc` file
because the dependency tracking misses header changes. Always use
*Production → Clean and Build Main Project* (Shift+F11).

**Missing BANKSEL:** BSR is not preserved across an interrupt. Always put an
explicit `BANKSEL` before the first access to any SFR group, even if you think
BSR is already correct.

**TMR1 write order:** With `RD16=1` (16-bit mode), write `TMR1L` first (it goes to
a shadow buffer), then write `TMR1H` (this transfers both bytes to the timer
simultaneously). Writing `TMR1H` first gives the wrong preload.

**Access mode on variables:** Variables in `udata_acs` must use `, c` (access
bank). Using `, 1` (banked) makes the CPU use BSR to look up the address, which
will point at the wrong location if BSR is not 0.
