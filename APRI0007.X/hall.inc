; ==========================================================
; hall.inc - Rear-wheel pulse counting and speed estimate
; Processor: PIC18F47Q84 @ 64 MHz
;
; Counts Hall pulses through INT2 and converts the captured
; count into a speed estimate over a fixed Timer0 window.
; ==========================================================

; ---- Variables (Access Bank) ----
PSECT udata_acs
hall_counter:       DS 1            ; event count in current interval
hall_count_l:       DS 1            ; last captured count (output)
hall_speed_l:       DS 1            ; estimated rear-wheel speed in km/h, low byte
hall_speed_h:       DS 1            ; estimated rear-wheel speed in km/h, high byte
hall_tmp_l:         DS 1            ; speed calculation scratch low byte
hall_tmp_h:         DS 1            ; speed calculation scratch high byte
hall_acc_l:         DS 1            ; speed accumulator low byte
hall_acc_h:         DS 1            ; speed accumulator high byte

; ---- INT2 ISR ----
PSECT isrHallINT2, class=CODE, reloc=4
Hall_INT2_ISR:
    BANKSEL PIR10
    bcf     BANKMASK(PIR10), 0, 1       ; clear INT2IF
    incf    hall_counter, f, c
    retfie  1

; ---- TMR0 ISR ----
PSECT isrHallTMR0, class=CODE, reloc=4
Hall_TMR0_ISR:
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 7, 1        ; clear TMR0IF

    ; Capture the current pulse count and reset the live counter.
    movf    hall_counter, w, c
    movwf   hall_count_l, c
    clrf    hall_counter, c

    ; Publish the speed estimate from the captured pulse count.
    call    _Hall_UpdateSpeedKmh
    retfie  1

; ---- Vectors ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     80*2
    DW      Hall_INT2_ISR >> 2
    ORG     31*2
    DW      Hall_TMR0_ISR >> 2

; ---- Regular code ----
PSECT code

; ---- _Hall_AddTmpToAcc ----
_Hall_AddTmpToAcc:
    movf    hall_tmp_l, w, c
    addwf   hall_acc_l, f, c
    movf    hall_tmp_h, w, c
    addwfc  hall_acc_h, f, c
    return

; ---- _Hall_ShiftTmpLeft ----
_Hall_ShiftTmpLeft:
    bcf     STATUS, 0, c
    rlcf    hall_tmp_l, f, c
    rlcf    hall_tmp_h, f, c
    return

; ---- _Hall_UpdateSpeedKmh ----
; speed_kmh ~= pulses * 53 / 32
_Hall_UpdateSpeedKmh:
    ; Start from the captured pulse count.
    movf    hall_count_l, w, c
    movwf   hall_tmp_l, c
    clrf    hall_tmp_h, c

    ; Build 53x as 1 + 4 + 16 + 32.
    clrf    hall_acc_l, c
    clrf    hall_acc_h, c

    call    _Hall_AddTmpToAcc

    call    _Hall_ShiftTmpLeft
    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    call    _Hall_ShiftTmpLeft
    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    call    _Hall_ShiftTmpLeft
    call    _Hall_AddTmpToAcc

    ; Divide by 32 with five right shifts.
    movf    hall_acc_l, w, c
    movwf   hall_tmp_l, c
    movf    hall_acc_h, w, c
    movwf   hall_tmp_h, c

    bcf     STATUS, 0, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c
    bcf     STATUS, 0, c
    rrcf    hall_tmp_h, f, c
    rrcf    hall_tmp_l, f, c

    movf    hall_tmp_l, w, c
    movwf   hall_speed_l, c
    movf    hall_tmp_h, w, c
    movwf   hall_speed_h, c
    return

; ---- Hall_Init ----
Hall_Init:
    ; RB2 input.
    BANKSEL ANSELB
    bcf     BANKMASK(ANSELB), 2, 1      ; RB2 digital
    BANKSEL TRISB
    bsf     BANKMASK(TRISB), 2, 1       ; RB2 input

    ; INT2 rising edge.
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 2, 1     ; INT2EDG = 1 (rising edge)
    BANKSEL PIR10
    bcf     BANKMASK(PIR10), 0, 1       ; clear INT2IF
    BANKSEL PIE10
    bsf     BANKMASK(PIE10), 0, 1       ; INT2IE = 1

    ; Timer0 setup.
    BANKSEL T0CON0
    clrf    BANKMASK(T0CON0), 1         ; T0EN=0, 8-bit mode (bit3=0)
    BANKSEL T0CON1
    movlw   0x4F                        ; T0CS=Fosc/4, sync, CKPS=1:32768
    movwf   BANKMASK(T0CON1), 1
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 7, 1        ; clear TMR0IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 7, 1        ; TMR0IE = 1

    ; Reset state.
    clrf    hall_counter, c
    clrf    hall_count_l, c
    clrf    hall_speed_l, c
    clrf    hall_speed_h, c
    clrf    hall_tmp_l, c
    clrf    hall_tmp_h, c
    clrf    hall_acc_l, c
    clrf    hall_acc_h, c

    ; Start Timer0.
    BANKSEL T0CON0
    bsf     BANKMASK(T0CON0), 7, 1      ; T0EN = 1

    return
