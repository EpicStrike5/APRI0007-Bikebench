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
;   The main loop detects released->pressed button edges every ~20ms.
;   Edge detection prevents a stuck/shorted pin from continuously
;   driving the servo.  CPU is free for CAN and other tasks.
;
; Servo position (0-255) maps to ~0-270 degrees:
;   0   -> ~0 deg   (400us HIGH pulse)
;   128 -> ~135 deg (~1.49ms HIGH pulse)
;   255 -> ~270 deg (~2.57ms HIGH pulse)
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
#include "hall.inc"

; ---- Main variables (edge detection) ----
; btn_prev tracks the last-seen db_stable value so the main loop can
; detect released->pressed transitions instead of polling raw level.
; A button that is stuck LOW from power-on will fire its handler at most
; once (on the very first edge when prev=released), then never again.
PSECT udata_acs
btn_prev:       DS 1            ; db_stable value from previous loop iteration
btn_edge:       DS 1            ; scratch: bits that just transitioned released->pressed

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
    ; Seed btn_prev with the current debounced state so the first loop
    ; iteration sees no edges and does not fire spurious handlers.
    movf    db_stable, w, c
    movwf   btn_prev, c
    call    Servo_Init
    call    Hall_Init
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 6, 1     ; GIEL = 1  (low-priority interrupts)
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE/GIEH = 1 (high-priority)
    movlw   100
    call    waitMilliSeconds

    ; ==========================================================
    ; MAIN LOOP
    ; Servo PWM runs in background via Timer1 overflow interrupt.
    ; Timer2 ISR (debounce.inc) updates db_stable every 10ms.
    ;
    ; Edge detection: handlers fire only on released->pressed
    ; transition, not while held.  This prevents a shorted/stuck
    ; button pin from continuously driving the servo.
    ;
    ; Algorithm (active-LOW: 0=pressed, 1=released):
    ;   changed = db_stable XOR btn_prev        (bits that flipped)
    ;   just_pressed = changed AND btn_prev      (were released, now pressed)
    ;   btn_prev = db_stable                     (update for next loop)
    ; ==========================================================
mainLoop:
    ; --- Compute just-pressed bits ---
    movf    db_stable, w, c             ; W = current debounced state
    xorwf   btn_prev, w, c             ; W = bits that changed (btn_prev unchanged)
    andwf   btn_prev, w, c             ; W = bits: were 1 (released) AND now 0 (pressed)
    movwf   btn_edge, c                 ; save just-pressed flags
    movf    db_stable, w, c             ; update prev for next iteration
    movwf   btn_prev, c

    ; --- Button 1 (RB0) : rotate RIGHT ---
    btfss   btn_edge, 0, c             ; skip if button 1 not just pressed
    call    handleButton1

    ; --- Button 2 (RB1) : rotate LEFT ---
    btfss   btn_edge, 1, c             ; skip if button 2 not just pressed
    call    handleButton2

    ; --- Delay ~20ms (debounce ISR fires at 10ms; poll at 20ms is fine) ---
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