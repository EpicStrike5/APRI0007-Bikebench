;======================================================================
; debounce.inc - Hardware Timer2 Button Debounce Library (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Hardware: Timer2 generates a 10ms periodic interrupt.
;           The ISR samples button pins and requires 3 consecutive
;           matching reads (30ms window) before updating the stable
;           debounced state.  This eliminates contact bounce entirely.
;
; Button pins (active LOW, internal pull-ups enabled):
;   RB0 = Button 1 (rotate RIGHT)
;   RB1 = Button 2 (rotate LEFT)
;
; Timer2 configuration:
;   Clock:     Fosc/4 = 16 MHz
;   Prescaler: 1:128  -> 125 kHz  (8 us/tick)
;   T2PR:      124    -> (124+1) * 8 us = 1 ms
;   Postscaler:1:10   -> interrupt every 10 ms
;
; Provides:
;   Debounce_Init    - configure Timer2, init debounce state, enable IRQ
;                      (pin config done externally by pinconfig.inc)
;   db_stable        - debounced button state (read from access bank)
;                      bit 0 = RB0 state, bit 1 = RB1 state
;                      (0 = pressed, 1 = released, matching active-LOW)
;
; Usage: #include "debounce.inc" from main.asm
;        (main must already have #include <xc.inc>)
;        Linker option: -pivt=08h  (shared IVT PSECT)
;        Caller must enable GIE/GIEL after Debounce_Init returns
;======================================================================

; ---- Button pin mask ----
DB_BTN_MASK     equ 0x03            ; bits 0-1 = RB0, RB1

; ---- Timer2 register values ----
; T2CON:  ON=1, CKPS=111 (1:128), OUTPS=1001 (1:10)
DB_T2CON_VAL    equ 0xF9
; T2CLK:  Fosc/4
DB_T2CLK_VAL    equ 0x01
; T2PR:   124 -> 1 ms base period
DB_T2PR_VAL     equ 124
; T2HLT:  free-running mode (no hardware limit)
DB_T2HLT_VAL    equ 0x00
; T2RST:  no external reset source
DB_T2RST_VAL    equ 0x00

; Consecutive matching samples needed to accept new state
DB_THRESHOLD    equ 3

; ---- Variables (Access Bank) ----
PSECT udata_acs
db_stable:      DS 1                ; debounced button state (read this)
db_prev:        DS 1                ; previous raw sample
db_count:       DS 1                ; consecutive-match counter

; ---- ISR Code (4-byte aligned for IVT address lookup) ----
PSECT isrDebounce, class=CODE, reloc=4

;----------------------------------------------------------------------
; Debounce_ISR
; Timer2 postscaled interrupt handler (every 10 ms).
; Samples RB0 and RB1, integrates over DB_THRESHOLD consecutive
; matching reads before updating db_stable.
;
; Uses FAST context save/restore (shadow registers via retfie 1).
;----------------------------------------------------------------------
Debounce_ISR:
    ; Clear TMR2 interrupt flag (PIR3 bit 3, bank 4)
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 3, 1

    ; Sample button pins (RB0, RB1) from PORTB
    BANKSEL PORTB
    movf    BANKMASK(PORTB), 0, 1       ; W = PORTB
    andlw   DB_BTN_MASK                  ; mask to button bits only

    ; Compare with previous sample
    xorwf   db_prev, w, c               ; W = current XOR previous
    bz      _db_isr_match               ; if zero, samples match

    ; --- Samples differ: restart counting ---
    BANKSEL PORTB
    movf    BANKMASK(PORTB), 0, 1       ; re-read PORTB
    andlw   DB_BTN_MASK
    movwf   db_prev, c                  ; save new sample
    clrf    db_count, c                 ; reset counter
    retfie  1

_db_isr_match:
    ; --- Samples match: increment counter ---
    incf    db_count, f, c
    movlw   DB_THRESHOLD
    cpfslt  db_count, c                 ; skip if count < threshold
    goto    _db_isr_accept
    retfie  1                           ; not yet stable

_db_isr_accept:
    ; Stable for DB_THRESHOLD consecutive samples -> update output
    movf    db_prev, w, c
    movwf   db_stable, c                ; commit debounced state
    movlw   DB_THRESHOLD
    movwf   db_count, c                 ; clamp counter at threshold
    retfie  1

; ---- TMR2 Interrupt Vector Table Entry ----
; TMR2 is IRQ 27.  IVT entry = 2-byte address word at IVTBASE + 2*27.
; CPU reads the DW value and left-shifts by 2 to form the ISR address.
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     27*2                        ; byte offset for IRQ 27
    DW      Debounce_ISR >> 2           ; ISR address / 4

; ---- Initialisation (regular code PSECT) ----
PSECT code

;----------------------------------------------------------------------
; Debounce_Init
; Initialises debounce state variables and configures Timer2 for
; 10 ms periodic interrupt.
;
; Pin I/O (ANSEL, TRIS, WPU) is handled by pinconfig.inc.
; Call PinConfig_Init BEFORE this routine.
; Caller enables GIE afterwards.
;----------------------------------------------------------------------
Debounce_Init:
    ; === Initialise debounce state ===
    ; Read current button state as initial stable value
    BANKSEL PORTB
    movf    BANKMASK(PORTB), 0, 1
    andlw   DB_BTN_MASK
    movwf   db_stable, c
    movwf   db_prev, c
    movlw   DB_THRESHOLD
    movwf   db_count, c                 ; already stable

    ; === Configure Timer2 ===
    ; All Timer2 registers are in bank 3 (0x322-0x327)
    BANKSEL T2CON
    clrf    BANKMASK(T2CON), 1          ; stop Timer2 first
    movlw   DB_T2CLK_VAL
    movwf   BANKMASK(T2CLKCON), 1       ; clock = Fosc/4
    movlw   DB_T2HLT_VAL
    movwf   BANKMASK(T2HLT), 1          ; free-running mode
    movlw   DB_T2RST_VAL
    movwf   BANKMASK(T2RST), 1          ; no external reset
    movlw   DB_T2PR_VAL
    movwf   BANKMASK(T2PR), 1           ; period = 124 -> 1 ms
    clrf    BANKMASK(T2TMR), 1          ; clear counter
    movlw   DB_T2CON_VAL
    movwf   BANKMASK(T2CON), 1          ; start Timer2

    ; === Enable TMR2 interrupt ===
    ; TMR2IF is PIR3 bit 3, TMR2IE is PIE3 bit 3 (bank 4)
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 3, 1        ; clear any pending TMR2IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 3, 1        ; TMR2IE = 1

    ; Caller must enable GIE after this returns
    return
