# APRI0007.X : PIC18F47Q84 E-Bike Gear Shift Controller

Automatic transmission controller for an e-bike. A servo motor is driven by a
frequency-variable signal (200–323 Hz, 50% duty cycle) selecting one of eight
gears. Torque and cadence are read from a Bafang motor via CAN FD (frame 0x67D).
Rear-wheel speed is measured by a Hall sensor on RB2. A 16×2 LCD displays live
telemetry. Two modes: manual (push-buttons) and automatic (cadence-based with
power-zone hysteresis).

**Processor:** PIC18F47Q84 @ 64 MHz (HFINTOSC, internal oscillator)  
**Toolchain:** MPLAB X + pic-as  
**Assembler flag:** `-xassembler-with-cpp`  
**Linker option:** `-pivt=08h`  
**Build:** always *Production → Clean and Build Main Project* : never just Build.
**Erase Memory device:** : Burn everything to start over on a healthy basis

---

## Pinout

| Pin | Function | Dir |
|-----|----------|-----|
| RA0 | Button : auto/manual toggle | In |
| RA1 | LED : auto mode active | Out |
| RA2 | Button : LCD page toggle | In |
| RA3 | Button : cadence set entry | In |
| RA4 | Button : cadence +10 rpm | In |
| RA5 | LED | Out |
| RA6 | LCD RS | Out |
| RA7 | External contrast test | In |
| RB0 | Button : harder gear (manual) | In |
| RB1 | Button : easier gear (manual) | In |
| RB2 | Hall sensor (INT2, rising edge) | In |
| RB3 | CAN RX (PPS) | In |
| RB4 | CAN TX (PPS, 0x46) | Out |
| RB5 | Servo signal (Timer1 toggle) | Out |
| RC0 | LCD RW | Out |
| RC1 | LCD E | Out |
| RC2–RC5 | LCD DB0–DB3 | Out |
| RD0–RD1 | LCD DB2–DB3 | Out |
| RD2 | LCD BL− | Out |
| RD3 | LCD BL+ | Out |
| RD6–RD7 | LCD DB5–DB4 | Out |
| RE0 | LED : startup | Out |
| RE1 | Button : cadence −10 rpm | In |
| RE2 | LED | Out |

---

## File Structure

```
main.asm          : entry point, init sequence, Timer4 scheduler, main loop
pinconfig.inc     : pin directions, pull-ups, latches, PPS routing
wait.inc          : blocking delays (µs / ms / s)
debounce.inc      : Timer2 debounce, button handlers, LCD UI state machine
servo_hw.inc      : Timer1 servo drive, 8-gear lookup table
hall.inc          : INT2 + Timer0 Hall sensor, speed calculation
can_torque.inc    : CAN FD, Bafang frame parser (0x67D), 128-sample filter
Control.inc       : cadence auto-shift logic, power-zone hysteresis
lcd_direct.inc    : HD44780 8-bit parallel driver
lcd.inc           : HD44780 via PCF8574 I2C backpack (alternative driver)
```
