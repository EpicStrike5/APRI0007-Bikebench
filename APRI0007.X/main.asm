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
#include "lcd.inc"

; ---- Main variables (edge detection) ----
; btn_prev tracks the last-seen db_stable value so the main loop can
; detect released->pressed transitions instead of polling raw level.
; A button that is stuck LOW from power-on will fire its handler at most
; once (on the very first edge when prev=released), then never again.
PSECT udata_acs
btn_prev:       DS 1            ; db_stable value from previous loop iteration
btn_edge:       DS 1            ; scratch: bits that just transitioned released->pressed
main_ctr:       DS 1            ; free-running loop counter (0-255, wraps), never touched by ISR
hall_snap_l:    DS 1            ; atomic snapshot of hall_count_l (taken with GIE=0)
hall_snap_h:    DS 1            ; atomic snapshot of hall_count_h (taken with GIE=0)

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
    call    LCD_I2C_Scan            ; find PCF8574 address, sets lcd_i2c_addr
    ; --- Scan result feedback on RD0 ---
    ; lcd_ack=0 (found): 3 fast blinks then continue
    ; lcd_ack=1 (not found): rapid continuous blink = wiring problem
    tstfsz  lcd_ack, c
    bra     _scan_fail
    ; Found: 3 fast blinks
    movlw   3
    movwf   btn_prev, c             ; reuse btn_prev as blink counter (seeded below anyway)
_scan_ok_blink:
    BANKSEL LATD
    bsf     BANKMASK(LATD), 0, 1
    movlw   80
    call    waitMilliSeconds
    BANKSEL LATD
    bcf     BANKMASK(LATD), 0, 1
    movlw   80
    call    waitMilliSeconds
    decfsz  btn_prev, f, c
    bra     _scan_ok_blink
    bra     _scan_done
_scan_fail:
    BANKSEL LATD
    btg     BANKMASK(LATD), 0, 1    ; toggle LED rapidly = no device found
    movlw   120
    call    waitMilliSeconds
    bra     _scan_fail              ; loops forever: fix wiring before proceeding
_scan_done:
    movf    db_stable, w, c         ; re-seed btn_prev (was used as blink counter)
    movwf   btn_prev, c
    call    LCD_Init
    ; Hello World test — confirms LCD wiring and init are correct
    call    LCD_GotoLine1
    movlw   'H'
    call    LCD_SendChar
    movlw   'e'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'o'
    call    LCD_SendChar
    call    LCD_GotoLine2
    movlw   'W'
    call    LCD_SendChar
    movlw   'o'
    call    LCD_SendChar
    movlw   'r'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'd'
    call    LCD_SendChar
    movlw   '!'
    call    LCD_SendChar
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 6, 1     ; GIEL = 1  (low-priority interrupts)
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE/GIEH = 1 (high-priority)
    movlw   100
    call    waitMilliSeconds
    clrf    main_ctr, c                 ; initialise loop counter after interrupts live

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

    ; --- Increment loop counter then refresh LCD ---
    incf    main_ctr, f, c              ; wraps 255→0, no ISR conflict
    call    LCD_RefreshDisplay
    movlw   200
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

; ------------------------------------------------------------------
; LCD_RefreshDisplay
; Full redraw of both lines every call — no stale chars possible.
;
; Line 1: "HallH: XXX" — hall_count_h, high byte of captured hall count.
; Line 2: "HallL: XXX" — hall_count_l, low byte of captured hall count.
;
; Atomicity: hall_count_h:hall_count_l is a 2-byte value updated by the
;   TMR0 ISR in two separate instructions.  A torn read is possible if an
;   interrupt fires between the two movf instructions.
;   Fix: disable GIE for exactly 4 instructions (~250 ns @ 64 MHz) while
;   copying both bytes into local snapshots hall_snap_h:hall_snap_l, then
;   re-enable and display from the snapshots.  Servo and debounce ISRs
;   are delayed at most one timer tick — harmless.
; ------------------------------------------------------------------
LCD_RefreshDisplay:
    ; Atomic 2-byte snapshot of hall_count_h:hall_count_l.
    ; GIE=0 window = 4 instructions ~250ns @ 64MHz.
    ; Servo and debounce ISRs delayed at most one tick — harmless.
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1     ; GIE=0 — mask all interrupts
    movf    hall_count_l, w, c
    movwf   hall_snap_l, c
    movf    hall_count_h, w, c
    movwf   hall_snap_h, c
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE=1 — re-enable

    ; --- Line 1: "HallH: XXX" ---
    movlw   0x80                    ; DDRAM addr 0x00 -> line 1, col 1
    call    _LCD_SendCmd
    movlw   'H'
    call    LCD_SendChar
    movlw   'a'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'H'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movf    hall_snap_h, w, c
    call    LCD_SendDec

    ; --- Line 2: "HallL: XXX" ---
    movlw   0xC0                    ; DDRAM addr 0x40 -> line 2, col 1
    call    _LCD_SendCmd
    movlw   'H'
    call    LCD_SendChar
    movlw   'a'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'l'
    call    LCD_SendChar
    movlw   'L'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movf    hall_snap_l, w, c
    call    LCD_SendDec
    return

    END     resetVec