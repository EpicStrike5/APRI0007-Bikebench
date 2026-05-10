;======================================================================
; lcd_direct.inc - HD44780 direct 8-bit parallel driver, pic-as, PIC18F47Q84
;
; Public functions kept close to the current lcd.inc API:
;   LCD_I2C_Scan   compatibility stub (lcd_ack=0, returns immediately)
;   LCD_Init
;   LCD_Clear
;   LCD_GotoLine1
;   LCD_GotoLine2
;   _LCD_SendCmd
;   LCD_SendChar
;   LCD_SendDec
;   LCD_WriteString
;
; Requires: wait.inc (waitMilliSeconds + waitMicroSeconds)
;
; Wiring used by this file:
;   RS  -> RA6
;   RW  -> RC0
;   E   -> RC1
;   D0  -> RC2
;   D1  -> RC3
;   D2  -> RD0
;   D3  -> RD1
;   D4  -> RD7
;   D5  -> RD6
;   D6  -> RC5
;   D7  -> RC4
;   BL+ -> RD3 (optional, driven high)
;   BL- -> RD2 (optional, driven low)
;
; Note: For the current hardware test, RA7 is left as an input so Vo can
; be driven externally to find a usable analog contrast level.
;======================================================================

PSECT udata_acs
lcd_byte:       DS 1
lcd_char:       DS 1
lcd_temp:       DS 1
lcd_hundreds:   DS 1
lcd_tens:       DS 1

PSECT code


;----------------------------------------------------------------------
; lcd_set_ctrl
; Input W bits:
;   bit0 -> RS
;   bit1 -> RW
;----------------------------------------------------------------------
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

;----------------------------------------------------------------------
; lcd_write_byte
; Writes the byte in W to the LCD data bus and pulses E.
; Includes a short post-write settle delay for standard commands/data.
;----------------------------------------------------------------------
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

    ; Pulse E high for 1 us, then low.
    bsf     BANKMASK(LATC), 1, 1
    movlw   1
    call    waitMicroSeconds
    bcf     BANKMASK(LATC), 1, 1
    movlw   1
    call    waitMicroSeconds

    ; Most HD44780 instructions need >37 us to settle.
    movlw   40
    call    waitMicroSeconds
    return

;----------------------------------------------------------------------
; _LCD_SendCmd
; Input: W = LCD command byte
;----------------------------------------------------------------------
_LCD_SendCmd:
    movwf   lcd_char, c
    movlw   0x00
    call    lcd_set_ctrl
    movf    lcd_char, w, c
    call    lcd_write_byte
    return

;----------------------------------------------------------------------
; LCD_Init
;----------------------------------------------------------------------
LCD_Init:
    ; Optional backlight control pins
    BANKSEL LATD
    bsf     BANKMASK(LATD), 3, 1
    bcf     BANKMASK(LATD), 2, 1

    movlw   42
    call    waitMilliSeconds

    movlw   0x38                    ; 8-bit, 2-line, 5x8 font
    call    _LCD_SendCmd
    movlw   0x38                    ; repeat function set
    call    _LCD_SendCmd
    movlw   0x0C                    ; display on, cursor off, blink off
    call    _LCD_SendCmd
    movlw   0x01                    ; clear display
    call    _LCD_SendCmd
    movlw   2
    call    waitMilliSeconds
    movlw   0x06                    ; entry mode: increment, no shift
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_Clear
;----------------------------------------------------------------------
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

;----------------------------------------------------------------------
; LCD_GotoLine1
;----------------------------------------------------------------------
LCD_GotoLine1:
    movlw   0x80
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_GotoLine2
;----------------------------------------------------------------------
LCD_GotoLine2:
    movlw   0xC0
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_SendChar
; Input: W = ASCII character
;----------------------------------------------------------------------
LCD_SendChar:
    movwf   lcd_char, c
    movlw   0x01
    call    lcd_set_ctrl
    movf    lcd_char, w, c
    call    lcd_write_byte
    return

;----------------------------------------------------------------------
; LCD_SendDec
; Input: W = 0..255, printed as 3 decimal digits
;----------------------------------------------------------------------
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

;----------------------------------------------------------------------
; LCD_WriteString
; Input: FSR0 -> null-terminated string in RAM
;----------------------------------------------------------------------
LCD_WriteString:
lcd_ws_loop:
    movf    POSTINC0, w, c
    bz      lcd_ws_done
    call    LCD_SendChar
    bra     lcd_ws_loop
lcd_ws_done:
    return
