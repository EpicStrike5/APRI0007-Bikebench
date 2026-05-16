; ==========================================================
; lcd_direct.inc - Low-level LCD driver
;
; Drives the HD44780 display in direct 8-bit mode and
; provides the basic routines used by the UI layer.
; ==========================================================

PSECT udata_acs
lcd_byte:       DS 1
lcd_char:       DS 1
lcd_temp:       DS 1
lcd_hundreds:   DS 1
lcd_tens:       DS 1
lcd_value_l:    DS 1
lcd_value_h:    DS 1
lcd_digit:      DS 1
lcd_div_l:      DS 1
lcd_div_h:      DS 1

PSECT code


; ---- lcd_set_ctrl ----
lcd_set_ctrl:
    movwf   lcd_byte, c

    BANKSEL LATA
    bcf     BANKMASK(LATA), 6, 1
    btfsc   lcd_byte, 0, c
    bsf     BANKMASK(LATA), 6, 1

    BANKSEL LATC
    bcf     BANKMASK(LATC), 0, 1
    btfsc   lcd_byte, 1, c
    bsf     BANKMASK(LATC), 0, 1
    return

; ---- lcd_write_byte ----
lcd_write_byte:
    movwf   lcd_byte, c

    BANKSEL LATC
    bcf     BANKMASK(LATC), 2, 1
    btfsc   lcd_byte, 0, c
    bsf     BANKMASK(LATC), 2, 1

    bcf     BANKMASK(LATC), 3, 1
    btfsc   lcd_byte, 1, c
    bsf     BANKMASK(LATC), 3, 1

    BANKSEL LATD
    bcf     BANKMASK(LATD), 0, 1
    btfsc   lcd_byte, 2, c
    bsf     BANKMASK(LATD), 0, 1

    bcf     BANKMASK(LATD), 1, 1
    btfsc   lcd_byte, 3, c
    bsf     BANKMASK(LATD), 1, 1

    bcf     BANKMASK(LATD), 7, 1
    btfsc   lcd_byte, 4, c
    bsf     BANKMASK(LATD), 7, 1

    bcf     BANKMASK(LATD), 6, 1
    btfsc   lcd_byte, 5, c
    bsf     BANKMASK(LATD), 6, 1

    BANKSEL LATC
    bcf     BANKMASK(LATC), 5, 1
    btfsc   lcd_byte, 6, c
    bsf     BANKMASK(LATC), 5, 1

    bcf     BANKMASK(LATC), 4, 1
    btfsc   lcd_byte, 7, c
    bsf     BANKMASK(LATC), 4, 1

    ; Pulse E.
    bsf     BANKMASK(LATC), 1, 1
    movlw   1
    call    waitMicroSeconds
    bcf     BANKMASK(LATC), 1, 1
    movlw   1
    call    waitMicroSeconds

    ; Short settle delay.
    movlw   40
    call    waitMicroSeconds
    return

; ---- _LCD_SendCmd ----
_LCD_SendCmd:
    movwf   lcd_char, c
    movlw   0x00
    call    lcd_set_ctrl
    movf    lcd_char, w, c
    call    lcd_write_byte
    return

; ---- LCD_Init ----
LCD_Init:
    ; Backlight control.
    BANKSEL LATD
    bsf     BANKMASK(LATD), 3, 1
    bcf     BANKMASK(LATD), 2, 1

    movlw   42
    call    waitMilliSeconds

    movlw   0x38
    call    _LCD_SendCmd
    movlw   0x38
    call    _LCD_SendCmd
    movlw   0x0C
    call    _LCD_SendCmd
    movlw   0x01
    call    _LCD_SendCmd
    movlw   2
    call    waitMilliSeconds
    movlw   0x06
    call    _LCD_SendCmd
    return

; ---- LCD_Clear ----
LCD_Clear:
    movlw   0x01
    call    _LCD_SendCmd
    movlw   2
    call    waitMilliSeconds
    movlw   0x02
    call    _LCD_SendCmd
    movlw   2
    call    waitMilliSeconds
    return

; ---- LCD_GotoLine1 ----
LCD_GotoLine1:
    movlw   0x80
    call    _LCD_SendCmd
    return

; ---- LCD_GotoLine2 ----
LCD_GotoLine2:
    movlw   0xC0
    call    _LCD_SendCmd
    return

; ---- LCD_SendChar ----
LCD_SendChar:
    movwf   lcd_char, c
    movlw   0x01
    call    lcd_set_ctrl
    movf    lcd_char, w, c
    call    lcd_write_byte
    return

; ---- LCD_SendDec ----
LCD_SendDec:
    movwf   lcd_temp, c
    clrf    lcd_hundreds, c
    clrf    lcd_tens, c

lcd_find_hundreds:
    movlw   100
    subwf   lcd_temp, w, c
    bnc     lcd_show_hundreds
    movwf   lcd_temp, c
    incf    lcd_hundreds, f, c
    bra     lcd_find_hundreds

lcd_show_hundreds:
    movlw   0x30
    addwf   lcd_hundreds, w, c
    call    LCD_SendChar

lcd_find_tens:
    movlw   10
    subwf   lcd_temp, w, c
    bnc     lcd_show_tens
    movwf   lcd_temp, c
    incf    lcd_tens, f, c
    bra     lcd_find_tens

lcd_show_tens:
    movlw   0x30
    addwf   lcd_tens, w, c
    call    LCD_SendChar

    movlw   0x30
    addwf   lcd_temp, w, c
    call    LCD_SendChar
    return

; ---- LCD_SendDec16 ----
LCD_SendDec16:
    movlw   0x10
    movwf   lcd_div_l, c
    movlw   0x27
    movwf   lcd_div_h, c
    call    _LCD_SendDec16Digit

    movlw   0xE8
    movwf   lcd_div_l, c
    movlw   0x03
    movwf   lcd_div_h, c
    call    _LCD_SendDec16Digit

    movlw   0x64
    movwf   lcd_div_l, c
    clrf    lcd_div_h, c
    call    _LCD_SendDec16Digit

    movlw   0x0A
    movwf   lcd_div_l, c
    clrf    lcd_div_h, c
    call    _LCD_SendDec16Digit

    movf    lcd_value_l, w, c
    addlw   '0'
    call    LCD_SendChar
    return

_LCD_SendDec16Digit:
    clrf    lcd_digit, c
lcd_dec16_loop:
    movf    lcd_div_h, w, c
    subwf   lcd_value_h, w, c
    bz      lcd_dec16_low
    bc      lcd_dec16_sub
    bra     lcd_dec16_done
lcd_dec16_low:
    movf    lcd_div_l, w, c
    subwf   lcd_value_l, w, c
    bc      lcd_dec16_sub
    bra     lcd_dec16_done
lcd_dec16_sub:
    movf    lcd_div_l, w, c
    subwf   lcd_value_l, f, c
    movf    lcd_div_h, w, c
    subwfb  lcd_value_h, f, c
    incf    lcd_digit, f, c
    bra     lcd_dec16_loop
lcd_dec16_done:
    movf    lcd_digit, w, c
    addlw   '0'
    call    LCD_SendChar
    return

; ---- LCD_WriteString ----
LCD_WriteString:
lcd_ws_loop:
    movf    POSTINC0, w, c
    bz      lcd_ws_done
    call    LCD_SendChar
    bra     lcd_ws_loop
lcd_ws_done:
    return
