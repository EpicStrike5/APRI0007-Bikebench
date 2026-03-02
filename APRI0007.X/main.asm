;======================================================================
; main.asm - Servo Control with Button Input
; Processor: PIC18F47Q84 @ 64MHz (Internal HFINTOSC)
;
; Hardware connections:
;   RC2 = Servo PWM output (Timer1 overflow interrupt)
;   RB0 = Button 1 - rotate RIGHT  (active LOW, internal pull-up)
;   RB1 = Button 2 - rotate LEFT   (active LOW, internal pull-up)
;   RD0 = LED feedback
;
; Diagnostic LEDs (active when SERVO_DEBUG=1 in servo_hw.inc):
;   RD1 = ON steady   -> Servo_Init completed successfully
;   RD2 = dim glow    -> Servo_ISR is firing (~100Hz toggle)
;   RC2 = two 250ms pulses at startup -> pin and wiring OK
;   If RD1 ON but RD2 dark -> interrupt not firing (IVT/config)
;   If RD2 glows but no scope on RC2 -> pin or scope issue
;
; How it works:
;   Servo PWM runs autonomously via Timer1 overflow interrupt (50 Hz).
;   The main loop polls buttons every ~20ms and updates position.
;   CPU is free for CAN bus and other tasks between ISR calls.
;
; Servo position (0-255) maps to 0-270 degrees:
;   0   -> ~0 deg   (0.5ms pulse)
;   128 -> ~135 deg (center)
;   255 -> ~270 deg (2.5ms pulse)
;======================================================================

PROCESSOR 18F47Q84

#include <xc.inc>

; ---- Configuration Bits ----
; Internal oscillator 64MHz, no watchdog, low-voltage programming on
CONFIG "FEXTOSC = OFF"              ; no external oscillator
CONFIG "RSTOSC = HFINTOSC_64MHZ"    ; internal 64 MHz at startup
CONFIG "CLKOUTEN = OFF"             ; clock out disabled
CONFIG "WDTE = OFF"                 ; watchdog timer disabled
CONFIG "LVP = ON"                   ; low-voltage programming enabled
CONFIG "MCLRE = EXTMCLR"           ; external master clear
CONFIG "MVECEN = ON"               ; multi-vector interrupts (REQUIRED)
CONFIG "XINST = OFF"                ; extended instruction set off
CONFIG "DEBUG = OFF"                ; background debugger disabled

; ---- Reset Vector (linked to address 0 via linker option) ----
PSECT resetVec, class=CODE, reloc=2
resetVec:
    goto    start

; ---- Include Libraries ----
; These files add their own PSECT udata_acs (variables) and
; PSECT code (functions). The linker auto-places everything.
#include "wait.inc"
#include "pinconfig.inc"
#include "servo_hw.inc"
#include "can_torque.inc"
#include "debounce.inc"

; ---- Main Code ----
PSECT code

start:
    ; ==========================================================
    ; I/O INITIALISATION
    ; ==========================================================

    ; --- All pins: ANSEL, TRIS, LAT, WPU, PPS (one place) ---
    call    PinConfig_Init

    ; --- Startup LED blink: confirms chip is alive ---
    BANKSEL LATD
    bsf     BANKMASK(LATD), 0, 1        ; LED ON
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATD), 0, 1        ; LED OFF
    movlw   250
    call    waitMilliSeconds
    bsf     BANKMASK(LATD), 0, 1        ; LED ON
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATD), 0, 1        ; LED OFF

    ; --- Initialise peripherals and enable interrupts ---
    call    Debounce_Init
    call    Servo_Init
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 6, 1     ; GIEL = 1  (low-priority interrupts)
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE/GIEH = 1 (high-priority)
    movlw   100
    call    waitMilliSeconds

    ; ==========================================================
    ; MAIN LOOP
    ; Servo PWM runs in background via Timer1 overflow interrupt.
    ; Timer2 ISR handles button debounce every 10ms.
    ; Main loop reads debounced state from db_stable.
    ; ==========================================================
mainLoop:
    ; --- Check Button 1 (RB0) : rotate RIGHT ---
    btfss   db_stable, 0, c             ; skip if NOT pressed (active LOW)
    call    handleButton1

    ; --- Check Button 2 (RB1) : rotate LEFT ---
    btfss   db_stable, 1, c             ; skip if NOT pressed (active LOW)
    call    handleButton2

    ; --- Delay ~20ms (polling rate for servo step updates) ---
    movlw   20
    call    waitMilliSeconds

    goto    mainLoop

; ------------------------------------------------------------------
; Button handlers
; ------------------------------------------------------------------
handleButton1:
    call    Servo_IncPos                ; increase angle
    BANKSEL LATD
    bsf     BANKMASK(LATD), 0, 1       ; LED ON  = moving right
    return

handleButton2:
    call    Servo_DecPos                ; decrease angle
    BANKSEL LATD
    bcf     BANKMASK(LATD), 0, 1       ; LED OFF = moving left
    return

    END     resetVec