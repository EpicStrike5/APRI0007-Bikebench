; ==========================================================
; debounce.inc - Button sampling and user actions
; Processor: PIC18F47Q84 @ 64 MHz
;
; Timer2 generates a 10 ms interrupt used to debounce the
; buttons. The main loop then interprets the validated state
; and applies the corresponding actions.
; ==========================================================

; ---- Button pin masks ----
DB_BTN_MASK_B   equ 0x03            ; PORTB bits 0-1 = RB0, RB1
DB_BTN_MASK_A0  equ 0x01            ; PORTA bit 0 = RA0
DB_BTN_MASK_A24 equ 0x1C            ; PORTA bits 2-4 = RA2, RA3, RA4
DB_BTN_MASK_E1  equ 0x02            ; PORTE bit 1 = RE1

; ---- Timer2 setup ----
DB_T2CON_VAL    equ 0xF9
DB_T2CLK_VAL    equ 0x01
DB_T2PR_VAL     equ 124
DB_T2HLT_VAL    equ 0x00
DB_T2RST_VAL    equ 0x00

; Consecutive matching samples needed to accept new state
DB_THRESHOLD    equ 3

; ---- Variables (Access Bank) ----
PSECT udata_acs
db_stable:      DS 1                ; debounced button state (read this)
db_prev:        DS 1                ; previous raw sample
db_count:       DS 1                ; consecutive-match counter
db_sample:      DS 1                ; packed raw sample from PORTA/PORTB/PORTE
db_temp:        DS 1                ; scratch for bit packing
btn_prev:       DS 1                ; last debounced state seen by the main loop
btn_edge:       DS 1                ; buttons that changed from released to pressed
flag_auto:      DS 1                ; bit 0 = auto mode enabled
flag_cadence_setting: DS 1          ; bit 0 = cadence target setting screen active
cadence_saved_ticks: DS 1           ; non-zero while the "cadence set" confirmation screen is shown

CADENCE_SAVED_TICKS  equ 100        ; 100 x 20 ms = 2 s confirmation message

; ---- Debounce ISR ----
PSECT isrDebounce, class=CODE, reloc=4

; ----------------------------------------------------------
; Debounce_ISR
; Sample buttons every 10 ms and update db_stable only
; after DB_THRESHOLD matching reads.
; ----------------------------------------------------------
Debounce_ISR:
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 3, 1

    ; Sample and pack the current button state.
    call    _db_read_sample

    xorwf   db_prev, w, c               ; W = current XOR previous
    bz      _db_isr_match               ; if zero, samples match

    ; New raw state -> restart the counter.
    movf    db_sample, w, c
    movwf   db_prev, c                  ; save new sample
    clrf    db_count, c                 ; reset counter
    retfie  1

_db_isr_match:
    ; Same raw state -> count one more match.
    incf    db_count, f, c
    movlw   DB_THRESHOLD
    cpfslt  db_count, c                 ; skip if count < threshold
    goto    _db_isr_accept
    retfie  1                           ; not yet stable

_db_isr_accept:
    ; Enough matching samples -> accept the new state.
    movf    db_prev, w, c
    movwf   db_stable, c                ; commit debounced state
    movlw   DB_THRESHOLD
    movwf   db_count, c                 ; clamp counter at threshold
    retfie  1

; ---- TMR2 vector ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     27*2                        ; byte offset for IRQ 27
    DW      Debounce_ISR >> 2           ; ISR address / 4

; ---- Regular code ----
PSECT code

; ---- _db_read_sample ----
_db_read_sample:
    ; Read RB0/RB1 into bits 0-1.
    BANKSEL PORTB
    movf    BANKMASK(PORTB), 0, 1
    andlw   DB_BTN_MASK_B
    movwf   db_sample, c

    ; Move RA0 into bit 2.
    BANKSEL PORTA
    movf    BANKMASK(PORTA), 0, 1
    andlw   DB_BTN_MASK_A0
    movwf   db_temp, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    movf    db_temp, w, c
    iorwf   db_sample, f, c

    ; Move RA2/RA3/RA4 into bits 3-5.
    BANKSEL PORTA
    movf    BANKMASK(PORTA), 0, 1
    andlw   DB_BTN_MASK_A24
    movwf   db_temp, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    movf    db_temp, w, c
    iorwf   db_sample, f, c

    ; Move RE1 into bit 6.
    BANKSEL PORTE
    movf    BANKMASK(PORTE), 0, 1
    andlw   DB_BTN_MASK_E1
    movwf   db_temp, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    bcf     STATUS, 0, c
    rlcf    db_temp, f, c
    movf    db_temp, w, c
    iorwf   db_sample, f, c

    movf    db_sample, w, c
    return

; ---- Debounce_Init ----
Debounce_Init:
    ; Seed the debounce state from the current button levels.
    call    _db_read_sample
    movwf   db_stable, c
    movwf   db_prev, c
    movlw   DB_THRESHOLD
    movwf   db_count, c                 ; already stable

    ; Configure Timer2 for a 10 ms interrupt.
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

    ; Enable the Timer2 interrupt.
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 3, 1        ; clear any pending TMR2IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 3, 1        ; TMR2IE = 1

    return

; ---- Buttons_Init ----
Buttons_Init:
    movf    db_stable, w, c
    movwf   btn_prev, c
    clrf    flag_auto, c
    clrf    flag_cadence_setting, c
    clrf    cadence_saved_ticks, c
    BANKSEL LATA
    bcf     BANKMASK(LATA), 1, 1        ; RA1 LED OFF while manual mode is active
    return

; ---- Buttons_HandleMainLoop ----
Buttons_HandleMainLoop:
    ; Compute the just-pressed bits.
    movf    db_stable, w, c
    xorwf   btn_prev, w, c
    andwf   btn_prev, w, c
    movwf   btn_edge, c
    movf    db_stable, w, c
    movwf   btn_prev, c

    ; RA0 always toggles the mode.
    btfsc   btn_edge, 2, c
    call    _buttons_handle_ra0

    ; RA2 always toggles the display page.
    btfsc   btn_edge, 3, c
    call    _buttons_handle_ra2

    ; RA3 always toggles cadence setting.
    btfsc   btn_edge, 4, c
    call    _buttons_handle_ra3

    ; In setting mode, only target adjustment remains active.
    btfsc   flag_cadence_setting, 0, c
    bra     _buttons_setting_mode

    ; In auto mode, manual shift buttons are ignored here.
    btfsc   flag_auto, 0, c
    bra     _buttons_auto_mode

    btfsc   btn_edge, 5, c
    call    _buttons_handle_ra4

    btfsc   btn_edge, 6, c
    call    _buttons_handle_re1

    ; In manual mode, only shift when the wheel is moving.
    movf    hall_count_l, f, c
    bz      _buttons_manual_done

    btfsc   btn_edge, 0, c
    call    _buttons_handle_rb0

    btfsc   btn_edge, 1, c
    call    _buttons_handle_rb1

_buttons_manual_done:
    return

_buttons_setting_mode:
    btfsc   btn_edge, 5, c
    call    _buttons_handle_ra4

    btfsc   btn_edge, 6, c
    call    _buttons_handle_re1

    return

_buttons_auto_mode:
    return

_buttons_handle_rb0:
    call    Servo_DecPos
    return

_buttons_handle_rb1:
    call    Servo_IncPos
    return

_buttons_handle_ra0:
    ; Toggle manual/auto mode and update the status LED.
    btg     flag_auto, 0, c
    BANKSEL LATA
    btfsc   flag_auto, 0, c
    bsf     BANKMASK(LATA), 1, 1        ; RA1 LED ON while auto mode is active
    btfss   flag_auto, 0, c
    bcf     BANKMASK(LATA), 1, 1        ; RA1 LED OFF while manual mode is active
    return

_buttons_handle_ra2:
    btg     lcd_screen_mode, 0, c
    bsf     lcd_clear_request, 0, c
    return

_buttons_handle_ra3:
    btfsc   flag_cadence_setting, 0, c
    bra     _buttons_exit_cadence_setting

    bsf     flag_cadence_setting, 0, c
    clrf    cadence_saved_ticks, c
    bsf     lcd_clear_request, 0, c
    return

_buttons_exit_cadence_setting:
    bcf     flag_cadence_setting, 0, c
    movlw   CADENCE_SAVED_TICKS
    movwf   cadence_saved_ticks, c
    bsf     lcd_clear_request, 0, c
    return

_buttons_handle_ra4:
    movlw   CADENCE_TARGET_MAX
    subwf   cadence_target_base, w, c
    bc      _buttons_ra4_clamp
    movlw   10
    addwf   cadence_target_base, f, c
    return

_buttons_ra4_clamp:
    movlw   CADENCE_TARGET_MAX
    movwf   cadence_target_base, c

_buttons_ra4_done:
    return

_buttons_handle_re1:
    movlw   10
    subwf   cadence_target_base, f, c
    movlw   CADENCE_TARGET_MIN
    subwf   cadence_target_base, w, c
    bc      _buttons_re1_done

_buttons_re1_clamp:
    movlw   CADENCE_TARGET_MIN
    movwf   cadence_target_base, c

_buttons_re1_done:
    return

; ---- LCD_PrintTargetFrame ----
LCD_PrintTargetFrame:
    movf    cadence_saved_ticks, f, c
    bnz     LCD_PrintCadenceSavedFrame

    btfsc   flag_cadence_setting, 0, c
    bra     LCD_PrintCadenceSettingFrame

    btfsc   lcd_screen_mode, 0, c
    bra     LCD_PrintPowerFrame

    call    LCD_GotoLine1
    movlw   'T'
    call    LCD_SendChar
    movlw   'R'
    call    LCD_SendChar
    movlw   'Q'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movf    torque_nm, w, c
    call    LCD_SendDec
    movlw   ' '
    call    LCD_SendChar
    movlw   '['
    call    LCD_SendChar
    movlw   'N'
    call    LCD_SendChar
    movlw   'm'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar

    call    LCD_GotoLine2
    movlw   'R'
    call    LCD_SendChar
    movlw   'P'
    call    LCD_SendChar
    movlw   'M'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movf    cadence, w, c
    call    LCD_SendDec
    movlw   ' '
    call    LCD_SendChar
    movlw   '['
    call    LCD_SendChar
    movlw   'r'
    call    LCD_SendChar
    movlw   'p'
    call    LCD_SendChar
    movlw   'm'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    return

LCD_PrintCadenceSavedFrame:
    call    LCD_GotoLine1
    movlw   'C'
    call    LCD_SendChar
    movlw   'A'
    call    LCD_SendChar
    movlw   'D'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'N'
    call    LCD_SendChar
    movlw   'C'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   'S'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   'O'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar

    call    LCD_GotoLine2
    movlw   'T'
    call    LCD_SendChar
    movlw   'G'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    clrf    lcd_value_h, c
    movf    cadence_target_base, w, c
    movwf   lcd_value_l, c
    call    LCD_SendDec16
    movlw   '['
    call    LCD_SendChar
    movlw   'r'
    call    LCD_SendChar
    movlw   'p'
    call    LCD_SendChar
    movlw   'm'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    return

LCD_PrintCadenceSettingFrame:
    ; Screen used while the rider adjusts the base cadence target.
    call    LCD_GotoLine1
    movlw   'S'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   'C'
    call    LCD_SendChar
    movlw   'A'
    call    LCD_SendChar
    movlw   'D'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'N'
    call    LCD_SendChar
    movlw   'C'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar

    call    LCD_GotoLine2
    movlw   'T'
    call    LCD_SendChar
    movlw   'G'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movf    cadence_target_base, w, c
    call    LCD_SendDec
    movlw   ' '
    call    LCD_SendChar
    movlw   '['
    call    LCD_SendChar
    movlw   'r'
    call    LCD_SendChar
    movlw   'p'
    call    LCD_SendChar
    movlw   'm'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    return

LCD_PrintPowerFrame:
    ; Secondary page: power and speed.
    call    LCD_GotoLine1
    movlw   'P'
    call    LCD_SendChar
    movlw   'W'
    call    LCD_SendChar
    movlw   'R'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movf    power_value_l, w, c
    movwf   lcd_value_l, c
    movf    power_value_h, w, c
    movwf   lcd_value_h, c
    call    LCD_SendDec16
    movlw   '['
    call    LCD_SendChar
    movlw   'W'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar

    call    LCD_GotoLine2
    movlw   'S'
    call    LCD_SendChar
    movlw   'P'
    call    LCD_SendChar
    movlw   'D'
    call    LCD_SendChar
    movlw   ':'
    call    LCD_SendChar
    movf    hall_speed_l, w, c
    movwf   lcd_value_l, c
    movf    hall_speed_h, w, c
    movwf   lcd_value_h, c
    call    LCD_SendDec16
    movlw   '['
    call    LCD_SendChar
    movlw   'k'
    call    LCD_SendChar
    movlw   'p'
    call    LCD_SendChar
    movlw   'h'
    call    LCD_SendChar
    movlw   ']'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    return
