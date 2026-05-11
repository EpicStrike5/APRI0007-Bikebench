;======================================================================
; hall.inc - Hall Sensor Counter (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz
;
; Monitors RB2 (INT2, rising edge) for hall sensor pulses.
; Counts events per interval. Every HALL_TMR0_INTERVAL Timer0
; overflows (~0.5s), captures the running count into
; hall_count_h:hall_count_l and resets for the next interval.
;
; Timer0 timing:
;   Clock = Fosc/4 = 16 MHz, prescaler = 1:32768
;   Tick rate = 16 MHz / 32768 = 488.28 Hz
;   8-bit overflow period = 256 / 488.28 = ~0.524 s
;   Capture interval = 1 overflow = ~0.5 s
;
; IRQ assignments (PIC18F47Q84 MVECEN=ON):
;   IRQ 31 = TMR0 overflow  (PIR3 bit 7, PIE3 bit 7)
;   IRQ 80 = INT2 rising edge (PIR10 bit 0, PIE10 bit 0)
;
; Public interface:
;   Hall_Init     - configure RB2, enable INT2 and Timer0 interrupts
;   hall_count_l  - most recent captured count, low byte  (read directly)
;   hall_count_h  - most recent captured count, high byte (read directly)
;   hall_speed_l  - estimated rear-wheel speed in km/h, low byte
;   hall_speed_h  - estimated rear-wheel speed in km/h, high byte
;
; Caller enables GIE (INTCON0 bit 7) and GIEL (INTCON0 bit 6)
; after all _Init calls return.
;======================================================================

HALL_TMR0_INTERVAL  equ 1           ; Timer0 overflows per capture (~0.5s)
HALL_WHEEL_CIRC_MM  equ 2170        ; first estimate for a 700x35C tyre

; ---- Variables (Access Bank) ----
PSECT udata_acs
hall_counter:       DS 1            ; event count in current interval
hall_overflow:      DS 1            ; increments when hall_counter wraps 255->0
hall_tmr0_ticks:    DS 1            ; Timer0 overflow counter
hall_count_l:       DS 1            ; last captured count, low byte  (output)
hall_count_h:       DS 1            ; last captured count, high byte (output)
hall_speed_l:       DS 1            ; estimated rear-wheel speed in km/h, low byte
hall_speed_h:       DS 1            ; estimated rear-wheel speed in km/h, high byte
hall_tmp_l:         DS 1            ; speed calculation scratch low byte
hall_tmp_h:         DS 1            ; speed calculation scratch mid byte
hall_tmp_u:         DS 1            ; speed calculation scratch high byte
hall_acc_l:         DS 1            ; speed accumulator low byte
hall_acc_h:         DS 1            ; speed accumulator mid byte
hall_acc_u:         DS 1            ; speed accumulator high byte

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

    ; Count overflows; act when interval reached
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

    ; Publish rear-wheel speed in km/h from the captured pulse count.
    ; For the current setup:
    ;   - 9 magnets on the rear wheel
    ;   - 700x35C tyre, estimated circumference ~2170 mm
    ;   - capture interval ~0.524 s
    ;
    ; This gives:
    ;   speed_kmh ~= pulses * 53 / 32
    ; which is a convenient integer approximation of the full formula.
    call    _Hall_UpdateSpeedKmh

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
; _Hall_AddTmpToAcc
; Adds the current 24-bit hall_tmp value into the 24-bit hall_acc.
;----------------------------------------------------------------------
_Hall_AddTmpToAcc:
    movf    hall_tmp_l, w, c
    addwf   hall_acc_l, f, c
    movf    hall_tmp_h, w, c
    addwfc  hall_acc_h, f, c
    movf    hall_tmp_u, w, c
    addwfc  hall_acc_u, f, c
    return

;----------------------------------------------------------------------
; _Hall_ShiftTmpLeft
; Shifts the 24-bit hall_tmp value left by one bit (x2).
;----------------------------------------------------------------------
_Hall_ShiftTmpLeft:
    bcf     STATUS, 0, c
    rlcf    hall_tmp_l, f, c
    rlcf    hall_tmp_h, f, c
    rlcf    hall_tmp_u, f, c
    return

;----------------------------------------------------------------------
; _Hall_UpdateSpeedKmh
; Converts the captured hall pulse count into an estimated rear-wheel
; speed in km/h and publishes it in hall_speed_h:l.
;
; Approximation used here for 9 magnets, 700x35C (~2170 mm), and the
; current Timer0 capture interval:
;   speed_kmh ~= pulses * 53 / 32
;---------------------------------------------------------------------- 
_Hall_UpdateSpeedKmh:
    ; Start with the latest captured 16-bit pulse count.
    movf    hall_count_l, w, c
    movwf   hall_tmp_l, c
    movf    hall_count_h, w, c
    movwf   hall_tmp_h, c
    clrf    hall_tmp_u, c

    ; Build 53x with a shift/add sequence:
    ;   53 = 1 + 4 + 16 + 32
    clrf    hall_acc_l, c
    clrf    hall_acc_h, c
    clrf    hall_acc_u, c

    ; Add 1x
    call    _Hall_AddTmpToAcc

    ; Shift to 2x, then 4x and add
    call    _Hall_ShiftTmpLeft
    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    ; Shift to 8x, then 16x and add
    call    _Hall_ShiftTmpLeft
    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    ; Shift to 32x and add
    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    ; Divide by 32 with five logical right shifts.
    movf    hall_acc_l, w, c
    movwf   hall_tmp_l, c
    movf    hall_acc_h, w, c
    movwf   hall_tmp_h, c
    movf    hall_acc_u, w, c
    movwf   hall_tmp_u, c

    bcf     STATUS, 0, c
    rrcf    hall_tmp_u, f, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_u, f, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_u, f, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_u, f, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_u, f, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c

    movf    hall_tmp_l, w, c
    movwf   hall_speed_l, c
    movf    hall_tmp_h, w, c
    movwf   hall_speed_h, c
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
    bsf     BANKMASK(INTCON0), 2, 1     ; INT2EDG = 1 (rising edge)
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
    clrf    hall_speed_l, c
    clrf    hall_speed_h, c
    clrf    hall_tmp_l, c
    clrf    hall_tmp_h, c
    clrf    hall_tmp_u, c
    clrf    hall_acc_l, c
    clrf    hall_acc_h, c
    clrf    hall_acc_u, c

    ; Start Timer0
    BANKSEL T0CON0
    bsf     BANKMASK(T0CON0), 7, 1      ; T0EN = 1

    return
