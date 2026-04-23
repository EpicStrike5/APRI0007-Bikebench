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
;   Hall_Init     - configure RB2, enable INT2 and Timer0 interrupts
;   hall_count_l  - most recent captured count, low byte  (read directly)
;   hall_count_h  - most recent captured count, high byte (read directly)
;
; RD2 toggles on every INT2 pulse (diagnostic).
; RD0 blinks once per capture interval, then blinks hall_count_l times.
;
; NOTE: DisplayHallCount is called from inside the TMR0 ISR.
;       This blocks all other ISRs for the duration of the blink sequence
;       (~300ms per blink * hall_count_l).  Acceptable for diagnostics.
;       waitMilliSeconds uses only access-bank variables so BSR is safe.
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
blink_counter:      DS 1            ; remaining blinks for DisplayHallCount

; ---- INT2 ISR (IRQ 80) ----
PSECT isrHallINT2, class=CODE, reloc=4
Hall_INT2_ISR:
    BANKSEL PIR10
    bcf     BANKMASK(PIR10), 0, 1       ; clear INT2IF
    BANKSEL LATD
    btg     BANKMASK(LATD), 2, 1        ; toggle RD2 on every hall pulse
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

    ; Count overflows; act when interval reached
    incf    hall_tmr0_ticks, f, c
    movlw   HALL_TMR0_INTERVAL
    subwf   hall_tmr0_ticks, w, c       ; W = ticks - INTERVAL
    btfss   STATUS, 2, c                ; skip if Z (ticks == INTERVAL)
    bra     _hall_tmr0_exit

    ; Interval blink: 100ms ON to mark capture moment
    BANKSEL LATD
    bsf     BANKMASK(LATD), 0, 1        ; LED D0 ON
    movlw   100
    call    waitMilliSeconds
    BANKSEL LATD
    bcf     BANKMASK(LATD), 0, 1        ; LED D0 OFF

    ; Capture hall_overflow:hall_counter into hall_count_h:hall_count_l
    clrf    hall_tmr0_ticks, c
    movf    hall_counter, w, c
    movwf   hall_count_l, c
    movf    hall_overflow, w, c
    movwf   hall_count_h, c
    ; Reset event counters for next interval
    clrf    hall_counter, c
    clrf    hall_overflow, c

    ; Blink RD0 hall_count_l times to display pulse count
    call    DisplayHallCount

_hall_tmr0_exit:
    retfie  1

; ---- IVT entries ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     80*2
    DW      Hall_INT2_ISR >> 2
    ORG     31*2
    DW      Hall_TMR0_ISR >> 2

; ---- Code ----
PSECT code

;----------------------------------------------------------------------
; DisplayHallCount
; Blinks RD0 a number of times equal to hall_count_l.
; Skips entirely if hall_count_l == 0.
; Each blink: 100ms ON, 200ms OFF.
; Called from TMR0 ISR — interrupts are blocked during blink sequence.
;----------------------------------------------------------------------
DisplayHallCount:
    movf    hall_count_l, w, c
    bz      _dhc_done               ; nothing to display
    movwf   blink_counter, c
_dhc_blink_loop:
    BANKSEL LATD
    bsf     BANKMASK(LATD), 0, 1    ; LED ON
    movlw   100
    call    waitMilliSeconds
    BANKSEL LATD
    bcf     BANKMASK(LATD), 0, 1    ; LED OFF
    movlw   200
    call    waitMilliSeconds
    decfsz  blink_counter, f, c
    bra     _dhc_blink_loop
_dhc_done:
    return

;----------------------------------------------------------------------
; Hall_Init
; Configures RB2 as digital input, enables INT2 (rising edge) and
; Timer0 interrupts.
; Call before enabling GIE/GIEL.
;----------------------------------------------------------------------
Hall_Init:
    ; RB2: digital input (pinconfig sets all ANSEL/TRIS, but set explicitly
    ; here so Hall_Init is self-contained and order-independent)
    BANKSEL ANSELB
    bcf     BANKMASK(ANSELB), 2, 1      ; RB2 digital
    BANKSEL TRISB
    bsf     BANKMASK(TRISB), 2, 1       ; RB2 input

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
    movlw   0x4F                        ; T0CS=Fosc/4, sync, CKPS=1:32768
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
    clrf    blink_counter, c

    ; Start Timer0
    BANKSEL T0CON0
    bsf     BANKMASK(T0CON0), 7, 1      ; T0EN = 1

    return
