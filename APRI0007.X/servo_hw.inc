; ==========================================================
; servo_hw.inc - Gear actuator signal generation
; Processor: PIC18F47Q84 @ 64 MHz
;
; Generates the Timer1-based output waveform on RB5 and
; manages the current gear through a discrete lookup table.
; ==========================================================

; ---- Configuration ----
SERVO_PIN               equ 5         ; RB5 bit in LATB, actual output pin toggled by the ISR
SERVO_GEAR_MIN          equ 1         ; gear numbering convention used in the project
SERVO_GEAR_MAX          equ 8         ; highest valid gear index
SERVO_DEFAULT_GEAR      equ 4         ; startup gear before any manual/auto shift occurs

; ---- Variables (Access Bank) ----
PSECT udata_acs
servo_gear:     DS 1                ; current selected gear number (1..8)
servo_ticks_l:  DS 1                ; low byte of the Timer1 preload currently in use
servo_ticks_h:  DS 1                ; high byte of the Timer1 preload currently in use
servo_tbl_off:  DS 1                ; temporary byte offset used when indexing the flash table

; ---- Servo ISR ----
PSECT isrServo, class=CODE, reloc=4

; ----------------------------------------------------------
; Servo_ISR
; Toggle RB5 and reload Timer1.
; ----------------------------------------------------------
Servo_ISR:
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1        ; clear TMR1IF

    ; Toggle RB5.
    BANKSEL LATB
    btg     BANKMASK(LATB), SERVO_PIN, 1

    ; Reload TMR1.
    BANKSEL TMR1L
    movf    servo_ticks_l, w, c
    movwf   BANKMASK(TMR1L), 1
    movf    servo_ticks_h, w, c
    movwf   BANKMASK(TMR1H), 1

    retfie  1

; ---- TMR1 vector ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     28*2
    DW      Servo_ISR >> 2              ; IVT stores the ISR address right-shifted by 2

; ---- Regular code ----
PSECT code

; ---- Servo_Init ----
Servo_Init:
    ; Configure Timer1.
    BANKSEL T1CON
    clrf    BANKMASK(T1CON), 1              ; stop timer, clear config
    clrf    BANKMASK(T1GCON), 1             ; gate disabled
    movlw   0x01
    movwf   BANKMASK(T1CLK), 1              ; clock = Fosc/4

    ; Load the default gear.
    movlw   SERVO_DEFAULT_GEAR
    movwf   servo_gear, c
    call    _servo_load_ticks_from_gear

    ; Load the initial preload.
    BANKSEL TMR1L
    movf    servo_ticks_l, w, c
    movwf   BANKMASK(TMR1L), 1
    movf    servo_ticks_h, w, c
    movwf   BANKMASK(TMR1H), 1

    ; Start Timer1.
    BANKSEL T1CON
    movlw   0x33
    movwf   BANKMASK(T1CON), 1

    ; Enable the TMR1 interrupt.
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1            ; clear any pending TMR1IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 4, 1            ; TMR1IE = 1

    return

; ---- Servo_SetGear ----
Servo_SetGear:
    movwf   servo_gear, c
    movlw   SERVO_GEAR_MIN
    subwf   servo_gear, w, c
    bc      _servo_check_max_gear
    movlw   SERVO_GEAR_MIN
    movwf   servo_gear, c
    bra     _servo_apply_gear

_servo_check_max_gear:
    movlw   SERVO_GEAR_MAX + 1
    subwf   servo_gear, w, c
    bc      _servo_clamp_max_gear
    bra     _servo_apply_gear

_servo_clamp_max_gear:
    movlw   SERVO_GEAR_MAX
    movwf   servo_gear, c

_servo_apply_gear:
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1
    call    _servo_load_ticks_from_gear
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1
    return

; ---- Servo_IncPos ----
Servo_IncPos:
    movf    servo_gear, w, c
    addlw   -1
    goto    Servo_SetGear

; ---- Servo_DecPos ----
Servo_DecPos:
    movf    servo_gear, w, c
    addlw   1
    goto    Servo_SetGear

; ---- _servo_load_ticks_from_gear ----
_servo_load_ticks_from_gear:
    movf    servo_gear, w, c
    addlw   -1
    movwf   servo_tbl_off, c

    bcf     STATUS, 0, c
    rlcf    servo_tbl_off, f, c

    movlw   low(_servo_preload_table)
    addwf   servo_tbl_off, w, c
    movwf   TBLPTRL, c
    movlw   high(_servo_preload_table)
    movwf   TBLPTRH, c
    clrf    TBLPTRU, c
    bc      _servo_preload_table_carry
    bra     _servo_preload_table_read

_servo_preload_table_carry:
    incf    TBLPTRH, f, c

_servo_preload_table_read:
    tblrd*+
    movff   TABLAT, servo_ticks_l
    tblrd*
    movff   TABLAT, servo_ticks_h
    return

_servo_preload_table:
    DW      62500                       ; gear 1
    DW      62250                       ; gear 2
    DW      62000                       ; gear 3
    DW      61850                       ; gear 4
    DW      61650                       ; gear 5
    DW      61400                       ; gear 6
    DW      61150                       ; gear 7
    DW      60900                       ; gear 8
    
