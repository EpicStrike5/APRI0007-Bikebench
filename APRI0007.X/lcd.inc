; lcd.inc  —  HD44780 via PCF8574 I2C backpack, pic-as, PIC18F47Q84 @ 64 MHz
;
; PCF pin  P7   P6   P5   P4 | P3   P2   P1   P0
; Signal   D7   D6   D5   D4 | BL    E   RW   RS
;
; RC3 = SCL, RC4 = SDA  (open-drain, ODCONC bits 3:4 = 1, idle HIGH)

#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR    0x27
#endif

#define LCD_BL_MASK     0x08        ; P3 backlight
#define LCD_E_MASK      0x04        ; P2 enable
#define LCD_RW_MASK     0x02        ; P1 RW (always 0)
#define LCD_RS_MASK     0x01        ; P0 RS

PSECT udata_acs
lcd_ack:        DS 1    ; 0 = ACK, 1 = NACK
lcd_addr:       DS 1    ; I2C write address found by scan (e.g. 0x4E)
lcd_bl:         DS 1    ; backlight byte: 0x08=on, 0x00=off
lcd_rs:         DS 1    ; RS byte: 0x00=command, 0x01=data
lcd_cmd_byte:   DS 1
lcd_nib_byte:   DS 1
lcd_io_byte:    DS 1
lcd_i2c_sr:     DS 1
lcd_i2c_bits:   DS 1
lcd_i2c_dly:    DS 1
lcd_n_bit:      DS 1
lcd_tampon:     DS 1
lcd_flag:       DS 1

PSECT code

_LCD_I2C_HalfPeriod:
    movlw   27
    movwf   lcd_i2c_dly, c
_lcd_hp_loop:
    decfsz  lcd_i2c_dly, f, c
    bra     _lcd_hp_loop
    return

_LCD_I2C_Start:
    BANKSEL LATC
    bsf     BANKMASK(LATC), 4, 1
    bsf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    bcf     BANKMASK(LATC), 4, 1    ; SDA falls while SCL high = START
    call    _LCD_I2C_HalfPeriod
    bcf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    return

_LCD_I2C_Stop:
    BANKSEL LATC
    bcf     BANKMASK(LATC), 4, 1
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 4, 1    ; SDA rises while SCL high = STOP
    call    _LCD_I2C_HalfPeriod
    return

_LCD_I2C_WriteByte:
    movwf   lcd_i2c_sr, c
    movlw   8
    movwf   lcd_i2c_bits, c
_lcd_wb_loop:
    BANKSEL LATC
    bcf     BANKMASK(LATC), 3, 1
    btfss   lcd_i2c_sr, 7, c
    bcf     BANKMASK(LATC), 4, 1
    btfsc   lcd_i2c_sr, 7, c
    bsf     BANKMASK(LATC), 4, 1
    rlncf   lcd_i2c_sr, f, c
    call    _LCD_I2C_HalfPeriod
    BANKSEL LATC
    bsf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    decfsz  lcd_i2c_bits, f, c
    bra     _lcd_wb_loop
    BANKSEL LATC
    bcf     BANKMASK(LATC), 3, 1
    bsf     BANKMASK(LATC), 4, 1
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    btfss   BANKMASK(PORTC), 4, 1
    clrf    lcd_ack, c
    btfsc   BANKMASK(PORTC), 4, 1
    movlw   1
    btfsc   BANKMASK(PORTC), 4, 1
    movwf   lcd_ack, c
    bcf     BANKMASK(LATC), 3, 1
    call    _LCD_I2C_HalfPeriod
    return

_LCD_IO_Write:
    movwf   lcd_io_byte, c
    call    _LCD_I2C_Start
    movf    lcd_addr, w, c
    call    _LCD_I2C_WriteByte
    movf    lcd_io_byte, w, c
    iorwf   lcd_bl, w, c
    call    _LCD_I2C_WriteByte
    call    _LCD_I2C_Stop
    return

_LCD_Write4Bit:
    movwf   lcd_nib_byte, c
    movf    lcd_rs, w, c
    iorwf   lcd_nib_byte, f, c
    movf    lcd_nib_byte, w, c
    call    _LCD_IO_Write
    movf    lcd_nib_byte, w, c
    iorlw   LCD_E_MASK
    call    _LCD_IO_Write
    movf    lcd_nib_byte, w, c
    andlw   0xFB
    call    _LCD_IO_Write
    movlw   1
    call    waitMilliSeconds
    return

_LCD_SendCmd:
    movwf   lcd_cmd_byte, c
    clrf    lcd_rs, c
    movf    lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    swapf   lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    return

LCD_I2C_Scan:
    movlw   0x40
    movwf   lcd_addr, c
_lcd_scan_try:
    call    _LCD_I2C_Start
    movf    lcd_addr, w, c
    call    _LCD_I2C_WriteByte
    call    _LCD_I2C_Stop
    tstfsz  lcd_ack, c
    bra     _lcd_scan_next
    return
_lcd_scan_next:
    movlw   2
    addwf   lcd_addr, f, c
    movlw   0x50
    cpfseq  lcd_addr, c
    bra     _lcd_scan_cont
    movlw   0x70
    movwf   lcd_addr, c
    bra     _lcd_scan_try
_lcd_scan_cont:
    movlw   0x80
    cpfseq  lcd_addr, c
    bra     _lcd_scan_try
    clrf    lcd_addr, c
    return

LCD_Init:
    movlw   LCD_BL_MASK
    movwf   lcd_bl, c
    movlw   0x00
    call    _LCD_IO_Write
    movlw   30
    call    waitMilliSeconds        ; >30 ms power-on wait
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds
    movlw   0x02                    ; switch to 4-bit mode
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds
    movlw   0x28                    ; 4-bit, 2 lines, 5x8 font
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds
    movlw   0x0C                    ; display on, cursor off, blink off
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds
    movlw   0x01                    ; clear display
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds
    movlw   0x06                    ; entry mode: cursor right, no shift
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds
    return

LCD_Clear:
    movlw   0x01
    call    _LCD_SendCmd
    movlw   3
    call    waitMilliSeconds        ; clear needs >1.52 ms
    movlw   0x80
    call    _LCD_SendCmd
    return

LCD_GotoLine1:
    movlw   0x80
    call    _LCD_SendCmd
    return

LCD_GotoLine2:
    movlw   0xC0
    call    _LCD_SendCmd
    return

LCD_SendChar:
    movwf   lcd_cmd_byte, c
    movlw   LCD_RS_MASK
    movwf   lcd_rs, c
    movf    lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    swapf   lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    return

LCD_SendDec:
    movwf   lcd_tampon, c
    clrf    lcd_flag, c
    clrf    lcd_n_bit, c
_lcd_h_sub:
    movlw   100
    subwf   lcd_tampon, w, c
    bnc     _lcd_h_done
    movwf   lcd_tampon, c
    incf    lcd_n_bit, f, c
    bra     _lcd_h_sub
_lcd_h_done:
    movf    lcd_n_bit, w, c
    movwf   lcd_flag, c
    bz      _lcd_h_sp
    addlw   '0'
    call    LCD_SendChar
    bra     _lcd_t_start
_lcd_h_sp:
    movlw   ' '
    call    LCD_SendChar
_lcd_t_start:
    clrf    lcd_n_bit, c
_lcd_t_sub:
    movlw   10
    subwf   lcd_tampon, w, c
    bnc     _lcd_t_done
    movwf   lcd_tampon, c
    incf    lcd_n_bit, f, c
    bra     _lcd_t_sub
_lcd_t_done:
    movf    lcd_n_bit, w, c
    bnz     _lcd_t_send
    tstfsz  lcd_flag, c
    bra     _lcd_t_zero
    movlw   ' '
    call    LCD_SendChar
    bra     _lcd_u
_lcd_t_zero:
    movlw   '0'
    call    LCD_SendChar
    bra     _lcd_u
_lcd_t_send:
    addlw   '0'
    call    LCD_SendChar
_lcd_u:
    movf    lcd_tampon, w, c
    addlw   '0'
    call    LCD_SendChar
    return

LCD_WriteString:
    movf    POSTINC0, w
    bz      _lcd_ws_done
    call    LCD_SendChar
    bra     LCD_WriteString
_lcd_ws_done:
    return
