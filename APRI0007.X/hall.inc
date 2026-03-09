;======================================================================
; hall.inc - Hall Sensor Counter (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz
;
; Monitors RB2 (INT2, rising edge) for hall sensor pulses.
; Counts events per interval. Every HALL_TMR0_INTERVAL Timer0
; overflows (~3.1s), captures the running count into
; hall_count_h:hall_count_l and resets for the next interval.
;
; Timer0 timing:
;   Clock = Fosc/4 = 16 MHz, prescaler = 1:32768
;   Tick rate = 16 MHz / 32768 = 488.28 Hz
;   8-bit overflow period = 256 / 488.28 = ~0.524 s
;   Capture interval = 6 overflows = ~3.1 s
;
; IRQ assignments (PIC18F47Q84 MVECEN=ON):
;   IRQ 31 = TMR0 overflow  (PIR3 bit 7, PIE3 bit 7)
;   IRQ 80 = INT2 rising edge (PIR10 bit 0, PIE10 bit 0)
;
; Public interface:
;   Hall_Init          - configure RB2, INT2, Timer0; enable interrupts
;   hall_count_l       - most recent captured count, low byte  (read directly)
;   hall_count_h       - most recent captured count, high byte (read directly)
;
; Caller enables GIE (INTCON0 bit 7) and GIEL (INTCON0 bit 6)
; after all _Init calls return.
;======================================================================

HALL_TMR0_INTERVAL  equ 6           ; Timer0 overflows per capture (~3.1s)

; ---- Variables (Access Bank) ----
PSECT udata_acs
hall_counter:       DS 1            ; event count in current interval
hall_overflow:      DS 1            ; increments when hall_counter wraps 255->0
hall_tmr0_ticks:    DS 1            ; Timer0 overflow counter
hall_count_l:       DS 1            ; last captured count, low byte  (output)
hall_count_h:       DS 1            ; last captured count, high byte (output)

; ---- INT2 ISR (IRQ 80) ----
PSECT isrHallINT2, class=CODE, reloc=4
Hall_INT2_ISR:
    BANKSEL PIR10
    bcf     BANKMASK(PIR10), 0, 1       ; clear INT2IF
    incfsz  hall_counter, f, c          ; increment; skip if result = 0 (wrapped)
    bra     _hall_no_overflow
    incf    hall_overflow, f, c         ; propagate wrap to high counter
_hall_no_overflow:
    retfie  1

; ---- TMR0 ISR (IRQ 31) ----
PSECT isrHallTMR0, class=CODE, reloc=4
Hall_TMR0_ISR:
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 7, 1        ; clear TMR0IF

    ; Count overflows; capture when interval reached
    incf    hall_tmr0_ticks, f, c
    movlw   HALL_TMR0_INTERVAL
    subwf   hall_tmr0_ticks, w, c       ; W = ticks - INTERVAL
    btfss   STATUS, 2, c                ; skip if Z (ticks == INTERVAL)
    bra     _hall_tmr0_exit

    ; Capture hall_overflow:hall_counter into hall_count_h:hall_count_l
    clrf    hall_tmr0_ticks, c
    movf    hall_counter, w, c
    movwf   hall_count_l, c
    movf    hall_overflow, w, c
    movwf   hall_count_h, c
    ; Reset event counters for next interval
    clrf    hall_counter, c
    clrf    hall_overflow, c

_hall_tmr0_exit:
    retfie  1

; ---- IVT entries ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     80*2
    DW      Hall_INT2_ISR >> 2
    ORG     31*2
    DW      Hall_TMR0_ISR >> 2

; ---- Initialisation ----
PSECT code

;----------------------------------------------------------------------
; Hall_Init
; Configures RB2 as digital input, enables INT2 (rising edge) and
; Timer0 interrupts. Call before enabling GIE/GIEL.
;----------------------------------------------------------------------
Hall_Init:
    ; RB2: digital input (pinconfig.inc clears all ANSELx at startup,
    ; but set explicitly here for self-contained initialisation)
    BANKSEL ANSELB
    bcf     BANKMASK(ANSELB), 2, 1      ; RB2 = digital
    BANKSEL TRISB
    bsf     BANKMASK(TRISB), 2, 1       ; RB2 = input

    ; INT2: rising edge, clear flag, enable
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 5, 1     ; INT2EDG = 1 (rising edge)
    BANKSEL PIR10
    bcf     BANKMASK(PIR10), 0, 1       ; clear INT2IF
    BANKSEL PIE10
    bsf     BANKMASK(PIE10), 0, 1       ; INT2IE = 1

    ; Timer0: Fosc/4, 1:32768 prescaler, 8-bit mode, timer off initially
    BANKSEL T0CON0
    clrf    BANKMASK(T0CON0), 1         ; T0EN=0, 8-bit mode (bit3=0)
    BANKSEL T0CON1
    movlw   b'01001111'                 ; T0CS=Fosc/4, sync, CKPS=1:32768
    movwf   BANKMASK(T0CON1), 1
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 7, 1        ; clear TMR0IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 7, 1        ; TMR0IE = 1

    ; Initialise all variables
    clrf    hall_counter, c
    clrf    hall_overflow, c
    clrf    hall_tmr0_ticks, c
    clrf    hall_count_l, c
    clrf    hall_count_h, c

    ; Start Timer0
    BANKSEL T0CON0
    bsf     BANKMASK(T0CON0), 7, 1      ; T0EN = 1

    return
