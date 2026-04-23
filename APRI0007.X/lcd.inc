;======================================================================
; lcd.inc  —  HD44780 via PCF8574 I2C backpack, pic-as, PIC18F47Q84 @ 64 MHz
;
; PIN MAPPING  — Type A  (YwRobot / QAPASS 1602A / LiquidCrystal_I2C)
;
;   PCF pin  P7   P6   P5   P4 | P3   P2   P1   P0
;   Signal   D7   D6   D5   D4 | BL    E   RW   RS
;
;   BL = 0x08  (bit 3)    backlight enable, 1=on
;   E  = 0x04  (bit 2)    enable pulse
;   RW = 0x02  (bit 1)    always 0 (write only)
;   RS = 0x01  (bit 0)    register select: 0=command, 1=data
;   data nibble in bits 7:4  (D7:D4)
;
; SOURCE
;   Translated directly from I2C_LCD.c (working C reference).
;   Every function maps 1:1 to its C equivalent.
;   I2C is bit-bang on RC3(SCL) / RC4(SDA), open-drain, ~100 kHz.
;   pinconfig.inc must set ODCONC bits 3,4=1  LATC bits 3,4=1 at idle.
;
; PUBLIC FUNCTIONS
;   LCD_I2C_Scan      find PCF8574 I2C address  (call before LCD_Init)
;   LCD_Init          power-on initialisation
;   LCD_Clear         clear display + cursor home
;   LCD_GotoLine1     cursor to line 1 col 1
;   LCD_GotoLine2     cursor to line 2 col 1
;   _LCD_SendCmd      send 8-bit command in W    (also called by main.asm)
;   LCD_SendChar      display ASCII char in W
;   LCD_SendDec       display W (0-255) as 3-digit decimal
;   LCD_WriteString   FSR0 → null-terminated string in RAM
;
; Requires:  wait.inc  (uses waitMilliSeconds)
;======================================================================

; ---- I2C address default ----
#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR    0x27        ; PCF8574  A2=A1=A0=1 → 7-bit 0x27 → write 0x4E
#endif

; ---- Backlight / control bit masks ----
#define LCD_BL_MASK     0x08        ; P3  backlight
#define LCD_E_MASK      0x04        ; P2  enable
#define LCD_RW_MASK     0x02        ; P1  RW (always 0)
#define LCD_RS_MASK     0x01        ; P0  RS

; ---- Access-bank variables ----
; Each call level gets its own scratch byte so there are no conflicts:
;   _LCD_IO_Write  →  uses lcd_io_byte
;   _LCD_Write4Bit →  uses lcd_nib_byte  (calls _LCD_IO_Write)
;   _LCD_SendCmd / LCD_SendChar  →  use lcd_cmd_byte  (call _LCD_Write4Bit)
;
PSECT udata_acs
lcd_ack:        DS 1    ; ACK result from last I2C write: 0=ACK, 1=NACK
                        ; *** main.asm reads this directly after LCD_I2C_Scan ***
lcd_addr:       DS 1    ; I2C write address found by scan (e.g. 0x4E for 0x27)
lcd_bl:         DS 1    ; backlight byte: 0x08=on, 0x00=off
lcd_rs:         DS 1    ; RS byte to OR into nibble: 0x00=command, 0x01=data
lcd_cmd_byte:   DS 1    ; scratch for _LCD_SendCmd / LCD_SendChar  (level 3)
lcd_nib_byte:   DS 1    ; scratch for _LCD_Write4Bit               (level 2)
lcd_io_byte:    DS 1    ; scratch for _LCD_IO_Write                 (level 1)
lcd_i2c_sr:     DS 1    ; I2C shift register
lcd_i2c_bits:   DS 1    ; I2C bit counter
lcd_i2c_dly:    DS 1    ; I2C half-period counter
lcd_n_bit:      DS 1    ; LCD_SendDec: digit counter
lcd_tampon:     DS 1    ; LCD_SendDec: working value
lcd_flag:       DS 1    ; LCD_SendDec: leading-zero flag

PSECT code

;======================================================================
; I2C BIT-BANG LAYER
; RC3 = SCL  (LATC bit 3)
; RC4 = SDA  (LATC bit 4)
; Both are open-drain outputs (ODCONC bits 3,4 = 1).
; Writing LAT=1 releases the pin to the pull-up resistor (= HIGH).
; Writing LAT=0 actively pulls LOW.
; PORTC always reads the actual pin voltage regardless of TRIS.
;======================================================================

;----------------------------------------------------------------------
; _LCD_I2C_HalfPeriod  —  ~5 µs busy-wait  (100 kHz half-period)
; 27 × 3 cycles + 2 = 83 cyc × 62.5 ns ≈ 5.2 µs
;----------------------------------------------------------------------
_LCD_I2C_HalfPeriod:
    movlw   27
    movwf   lcd_i2c_dly, c
_lcd_hp_loop:
    decfsz  lcd_i2c_dly, f, c
    bra     _lcd_hp_loop
    return

;----------------------------------------------------------------------
; _LCD_I2C_Start  —  START condition
; SDA falls while SCL is HIGH.  Returns with SCL LOW.
;----------------------------------------------------------------------
_LCD_I2C_Start:
    BANKSEL LATC
    bsf     BANKMASK(LATC), 4, 1    ; SDA HIGH (ensure idle)
    bsf     BANKMASK(LATC), 3, 1    ; SCL HIGH
    call    _LCD_I2C_HalfPeriod
    bcf     BANKMASK(LATC), 4, 1    ; SDA LOW while SCL HIGH = START
    call    _LCD_I2C_HalfPeriod
    bcf     BANKMASK(LATC), 3, 1    ; SCL LOW
    call    _LCD_I2C_HalfPeriod
    return

;----------------------------------------------------------------------
; _LCD_I2C_Stop  —  STOP condition
; SDA rises while SCL is HIGH.  Bus idles HIGH after return.
;----------------------------------------------------------------------
_LCD_I2C_Stop:
    BANKSEL LATC
    bcf     BANKMASK(LATC), 4, 1    ; SDA LOW (setup before stop)
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 3, 1    ; SCL HIGH
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 4, 1    ; SDA HIGH while SCL HIGH = STOP
    call    _LCD_I2C_HalfPeriod
    return

;----------------------------------------------------------------------
; _LCD_I2C_WriteByte  —  transmit W MSB-first, capture ACK → lcd_ack
;   lcd_ack = 0 : slave ACKed
;   lcd_ack = 1 : NACK (no device or bus error)
;----------------------------------------------------------------------
_LCD_I2C_WriteByte:
    movwf   lcd_i2c_sr, c           ; shift register ← W
    movlw   8
    movwf   lcd_i2c_bits, c         ; 8 bits to send
_lcd_wb_loop:
    BANKSEL LATC
    bcf     BANKMASK(LATC), 3, 1    ; SCL LOW  — set up SDA
    btfss   lcd_i2c_sr, 7, c
    bcf     BANKMASK(LATC), 4, 1    ; MSB = 0 → SDA LOW
    btfsc   lcd_i2c_sr, 7, c
    bsf     BANKMASK(LATC), 4, 1    ; MSB = 1 → SDA HIGH
    rlncf   lcd_i2c_sr, f, c        ; shift: next bit → bit 7
    call    _LCD_I2C_HalfPeriod
    BANKSEL LATC
    bsf     BANKMASK(LATC), 3, 1    ; SCL HIGH — slave samples
    call    _LCD_I2C_HalfPeriod
    decfsz  lcd_i2c_bits, f, c
    bra     _lcd_wb_loop
    ; 9th clock: release SDA, pulse SCL, read ACK
    BANKSEL LATC
    bcf     BANKMASK(LATC), 3, 1    ; SCL LOW
    bsf     BANKMASK(LATC), 4, 1    ; release SDA → pull-up → slave may pull LOW
    call    _LCD_I2C_HalfPeriod
    bsf     BANKMASK(LATC), 3, 1    ; SCL HIGH
    call    _LCD_I2C_HalfPeriod
    ; Sample SDA (PORTC shares bank 4 with LATC — no extra BANKSEL needed)
    btfss   BANKMASK(PORTC), 4, 1   ; SDA = 1 → NACK → skip clrf
    clrf    lcd_ack, c              ; SDA = 0 → ACK
    btfsc   BANKMASK(PORTC), 4, 1   ; SDA = 0 → ACK → skip movlw/movwf
    movlw   1
    btfsc   BANKMASK(PORTC), 4, 1
    movwf   lcd_ack, c              ; SDA = 1 → NACK = 1
    bcf     BANKMASK(LATC), 3, 1    ; SCL LOW
    call    _LCD_I2C_HalfPeriod
    return

;======================================================================
; PCF8574 + HD44780 LAYER
; Translated directly from I2C_LCD.c — function names match 1:1.
;======================================================================

;----------------------------------------------------------------------
; _LCD_IO_Write  ≡  IO_Expander_Write(Data)
;
;   W = data byte to write.
;   Sends:  START + lcd_addr + (W | lcd_bl) + STOP
;   BL is always OR'd in so the backlight never glitches off mid-init.
;
; Scratch: lcd_io_byte  (level 1 — not touched by anything above)
;----------------------------------------------------------------------
_LCD_IO_Write:
    movwf   lcd_io_byte, c          ; save data — _LCD_I2C_WriteByte will overwrite W
    call    _LCD_I2C_Start
    movf    lcd_addr, w, c
    call    _LCD_I2C_WriteByte      ; send address byte
    movf    lcd_io_byte, w, c       ; reload data
    iorwf   lcd_bl, w, c            ; W = data | lcd_bl  (BackLight_State always on)
    call    _LCD_I2C_WriteByte      ; send data byte
    call    _LCD_I2C_Stop
    return

;----------------------------------------------------------------------
; _LCD_Write4Bit  ≡  write4bits(value) from LiquidCrystal_I2C.cpp
;
;   W = nibble byte with data in bits 7:4, E=0.
;   ORs in lcd_rs (0x00 or 0x01).
;
;   Sequence (THREE I2C transactions — matches Arduino library exactly):
;     1. _LCD_IO_Write( W )        — data setup,  E=0  (expanderWrite)
;     2. _LCD_IO_Write( W | E )    — E rising edge      (pulseEnable E=1)
;     3. _LCD_IO_Write( W & ~E )   — E falling edge     (pulseEnable E=0)
;     4. 1 ms settle  (HD44780 needs >37 µs; 1 ms is generous)
;
;   The setup write (step 1) ensures data is stable on D7:D4 BEFORE E
;   rises — this is required by the HD44780 tDSU spec (40 ns min).
;   Each I2C transaction takes ~300 µs at 100 kHz, far exceeding any
;   timing requirement.
;
; Scratch: lcd_nib_byte  (level 2 — not touched by _LCD_IO_Write)
;----------------------------------------------------------------------
_LCD_Write4Bit:
    movwf   lcd_nib_byte, c         ; save nibble
    movf    lcd_rs, w, c
    iorwf   lcd_nib_byte, f, c      ; nibble |= RS  (0x00 or 0x01)
    ; --- step 1: data setup  E=0 ---
    movf    lcd_nib_byte, w, c
    call    _LCD_IO_Write            ; expanderWrite(value)
    ; --- step 2: E = 1 ---
    movf    lcd_nib_byte, w, c
    iorlw   LCD_E_MASK              ; | 0x04
    call    _LCD_IO_Write            ; expanderWrite(value | En)
    ; --- step 3: E = 0  (HD44780 latches data on this falling edge) ---
    movf    lcd_nib_byte, w, c
    andlw   0xFB                    ; & ~0x04
    call    _LCD_IO_Write            ; expanderWrite(value & ~En)
    ; --- step 4: settle delay ---
    movlw   1
    call    waitMilliSeconds
    return

;----------------------------------------------------------------------
; _LCD_SendCmd  ≡  LCD_CMD(CMD)
;
;   W = 8-bit HD44780 command.  RS = 0 (command register).
;   Sends upper nibble (bits 7:4) then lower nibble (bits 3:0),
;   each with an E pulse via _LCD_Write4Bit.
;
;   *** Also called directly by main.asm (LCD_RefreshDisplay) ***
;
; Nibble construction (data in PCF bits 7:4):
;   upper:  CMD & 0xF0               → bits 7:4 of CMD into PCF bits 7:4
;   lower:  swapf(CMD) & 0xF0        → bits 3:0 of CMD into PCF bits 7:4
;           (same as (CMD << 4) & 0xF0 in C)
;
; Scratch: lcd_cmd_byte  (level 3 — not touched by _LCD_Write4Bit)
;----------------------------------------------------------------------
_LCD_SendCmd:
    movwf   lcd_cmd_byte, c
    clrf    lcd_rs, c               ; RS = 0 (command)
    ; upper nibble
    movf    lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    ; lower nibble
    swapf   lcd_cmd_byte, w, c      ; swap: original bits 3:0 → position 7:4
    andlw   0xF0
    call    _LCD_Write4Bit
    return

;======================================================================
; PUBLIC FUNCTIONS
;======================================================================

;----------------------------------------------------------------------
; LCD_I2C_Scan
;
; Probe PCF8574  (write addresses 0x40–0x4E, 7-bit 0x20–0x27) and
;      PCF8574A (write addresses 0x70–0x7E, 7-bit 0x38–0x3F).
; Sets lcd_addr to the first address that ACKs.
; If no device found: lcd_addr = 0, lcd_ack = 1.
; Call once before LCD_Init.
;----------------------------------------------------------------------
LCD_I2C_Scan:
    movlw   0x40                    ; start of PCF8574 range
    movwf   lcd_addr, c
_lcd_scan_try:
    call    _LCD_I2C_Start
    movf    lcd_addr, w, c
    call    _LCD_I2C_WriteByte      ; result → lcd_ack
    call    _LCD_I2C_Stop
    tstfsz  lcd_ack, c              ; skip if ACK (lcd_ack = 0)
    bra     _lcd_scan_next
    return                          ; ACK → lcd_addr is valid
_lcd_scan_next:
    movlw   2
    addwf   lcd_addr, f, c          ; +2 in 8-bit space = +1 in 7-bit
    movlw   0x50                    ; end of PCF8574 range
    cpfseq  lcd_addr, c
    bra     _lcd_scan_cont
    movlw   0x70                    ; jump to PCF8574A range
    movwf   lcd_addr, c
    bra     _lcd_scan_try
_lcd_scan_cont:
    movlw   0x80                    ; end of PCF8574A range
    cpfseq  lcd_addr, c
    bra     _lcd_scan_try
    clrf    lcd_addr, c             ; nothing found
    return

;----------------------------------------------------------------------
; LCD_Init  ≡  LCD_Init(I2C_Add) from I2C_LCD.c
;
; lcd_addr must be set (call LCD_I2C_Scan first).
; Matches the working C init sequence byte-for-byte.
;----------------------------------------------------------------------
LCD_Init:
    movlw   LCD_BL_MASK
    movwf   lcd_bl, c               ; backlight on (lcd_bl = 0x08)
    ; IO_Expander_Write(0x00) → sends 0x00 | 0x08 = 0x08  (just BL, all else 0)
    movlw   0x00
    call    _LCD_IO_Write
    movlw   30
    call    waitMilliSeconds        ; >30 ms power-on wait

    ; HD44780 8-bit reset preamble:
    ; LCD_CMD(0x03) sends two nibbles: upper=0x00 then lower=0x30
    ; The 0x30 nibble (DB5,DB4=1) = Function Set 8-bit mode preamble
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds        ; >4.1 ms
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds
    movlw   0x03
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds

    ; Switch to 4-bit mode (LCD_RETURN_HOME = 0x02):
    ; LCD_CMD(0x02) sends upper=0x00 then lower=0x20  (DB5=1 = 4-bit mode)
    movlw   0x02
    call    _LCD_SendCmd
    movlw   5
    call    waitMilliSeconds

    ; From here: HD44780 is in 4-bit mode, all commands are two nibbles.

    movlw   0x28                    ; Function Set: 4-bit bus, 2 lines, 5x8 font
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds

    movlw   0x0C                    ; Display ON, cursor OFF, blink OFF
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds

    movlw   0x01                    ; Clear Display
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds

    movlw   0x06                    ; Entry Mode: cursor right, no display shift
    call    _LCD_SendCmd
    movlw   50
    call    waitMilliSeconds

    return

;----------------------------------------------------------------------
; LCD_Clear  —  clear display and return cursor to line 1 col 1
;----------------------------------------------------------------------
LCD_Clear:
    movlw   0x01
    call    _LCD_SendCmd
    movlw   3
    call    waitMilliSeconds        ; Clear needs >1.52 ms
    movlw   0x80                    ; DDRAM address 0x00
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_GotoLine1  —  cursor to line 1, column 1  (DDRAM 0x00)
;----------------------------------------------------------------------
LCD_GotoLine1:
    movlw   0x80
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_GotoLine2  —  cursor to line 2, column 1  (DDRAM 0x40)
;----------------------------------------------------------------------
LCD_GotoLine2:
    movlw   0xC0
    call    _LCD_SendCmd
    return

;----------------------------------------------------------------------
; LCD_SendChar  ≡  LCD_Write_Char(Data)
;
;   W = ASCII character to display.
;   RS = 1 (data register).  Same nibble split as _LCD_SendCmd.
;----------------------------------------------------------------------
LCD_SendChar:
    movwf   lcd_cmd_byte, c
    movlw   LCD_RS_MASK
    movwf   lcd_rs, c               ; RS = 1 (data)
    ; upper nibble
    movf    lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    ; lower nibble
    swapf   lcd_cmd_byte, w, c
    andlw   0xF0
    call    _LCD_Write4Bit
    return

;----------------------------------------------------------------------
; LCD_SendDec  —  display W (0-255) as right-justified 3-char decimal
; Examples:   5 → "  5"     42 → " 42"     255 → "255"
;----------------------------------------------------------------------
LCD_SendDec:
    movwf   lcd_tampon, c
    clrf    lcd_flag, c             ; 0 = no non-zero digit printed yet
    ; --- hundreds ---
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
    ; --- tens ---
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
    ; --- units ---
_lcd_u:
    movf    lcd_tampon, w, c
    addlw   '0'
    call    LCD_SendChar
    return

;----------------------------------------------------------------------
; LCD_WriteString  —  FSR0 points to a null-terminated string in RAM
;----------------------------------------------------------------------
LCD_WriteString:
    movf    POSTINC0, w
    bz      _lcd_ws_done
    call    LCD_SendChar
    bra     LCD_WriteString
_lcd_ws_done:
    return
