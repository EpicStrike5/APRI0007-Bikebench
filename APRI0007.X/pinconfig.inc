;======================================================================
; pinconfig.inc - Centralised Pin Configuration (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz
;
; ALL pin I/O setup, pull-ups, and PPS (Peripheral Pin Select)
; remapping are consolidated here.  Peripheral-specific timer,
; interrupt and module configuration stays in its own .inc file.
;
; Call PinConfig_Init FIRST, before any peripheral _Init routine.
;
; ---------------------------------------------------------------
; PIN ASSIGNMENT TABLE
; ---------------------------------------------------------------
; Pin   Function         Dir   Analog  Pull-up  PPS
; ----  ---------------  ----  ------  -------  ----------------
; RA0   (unused)         in    no      YES      -
; RA1   (unused)         in    no      YES      -
; RA2   (unused)         in    no      YES      -
; RA3   (unused)         in    no      YES      -
; RA4   (unused)         in    no      YES      -
; RA5   (unused)         in    no      YES      -
; RA6   (unused)         in    no      YES      -
; RA7   (unused)         in    no      YES      -
;
; RB0   Button 1 (RIGHT) in   no      YES      -
; RB1   Button 2 (LEFT)  in   no      YES      -
; RB2   (unused)         in    no      YES      -
; RB3   (unused)         in    no      YES      -
; RB4   (unused)         in    no      YES      -
; RB5   (unused)         in    no      YES      -
; RB6   (unused)         in    no      YES      -
; RB7   (unused)         in    no      YES      -
;
; RC0   (unused)         in    no      YES      -
; RC1   (unused)         in    no      YES      -
; RC2   Servo PWM        out   no      YES      - (SW toggle)
; RC3   (unused)         in    no      YES      -
; RC4   (unused)         in    no      YES      -
; RC5   (unused)         in    no      YES      -
; RC6   (unused)         in    no      YES      -
; RC7   (unused)         in    no      YES      -
;
; RD0   LED feedback     out   no      YES      -
; RD1   DEBUG: init OK   out   no      YES      -  (wire LED)
; RD2   DEBUG: ISR fire  out   no      YES      -  (wire LED)
; RD3   (unused)         in    no      YES      -
; RD4   (unused)         in    no      YES      -
; RD5   (unused)         in    no      YES      -
; RD6   (unused)         in    no      YES      -
; RD7   (unused)         in    no      YES      -
;
; RE0   (unused)         in    no      YES      -
; RE1   (unused)         in    no      YES      -
; RE2   (unused)         in    no      YES      -
; RE3   MCLR             in    -       -        - (config bit)
;
; ---------------------------------------------------------------
; ALTERNATIVE: CAN BUS (uncomment PPS block below to enable)
;   RB0 -> CANTX  (out, digital, no pull-up)  RB0PPS  = 0x46
;   RB1 -> CANRX  (in,  digital, no pull-up)  CANRXPPS= 0x09
;   NOTE: conflicts with button pins - pick one or relocate
; ---------------------------------------------------------------
;
; ---------------------------------------------------------------
; PPS REFERENCE
; ---------------------------------------------------------------
;
; OUTPUT function codes (RxyPPS values):
;   0x00 = LATxy (default GPIO)
;   0x01 = CLC1OUT         0x02 = CLC2OUT
;   0x03 = CLC3OUT         0x04 = CLC4OUT
;   0x05 = CWG1A           0x06 = CWG1B
;   0x07 = CWG1C           0x08 = CWG1D
;   0x09 = CCP1
;   0x0B = PWM1S1P1_OUT    0x0C = PWM1S1P2_OUT
;   0x0D = PWM2S1P1_OUT    0x0E = PWM2S1P2_OUT
;   0x0F = PWM3S1P1_OUT    0x10 = PWM3S1P2_OUT
;   0x11 = NCO1
;   0x20 = SPI1_SCK        0x21 = SPI1_SDO
;   0x22 = SPI1_SS
;   0x33 = TX1 (UART1 TX)
;   0x46 = CANTX
;   (see datasheet Table "PPS Output Signal Routing" for full list)
;
; INPUT pin encoding (xxxPPS values):
;   RA0=0x00  RA1=0x01  RA2=0x02 ... RA7=0x07
;   RB0=0x08  RB1=0x09  RB2=0x0A ... RB7=0x0F
;   RC0=0x10  RC1=0x11  RC2=0x12 ... RC7=0x17
;   RD0=0x18  RD1=0x19  RD2=0x1A ... RD7=0x1F
;   RE0=0x20  RE1=0x21  RE2=0x22     (RE3=MCLR)
;
; INPUT PPS registers (select which pin feeds a peripheral):
;   CANRXPPS  = 0x23D     INT0PPS  = 0x23E
;   INT1PPS   = 0x23F     INT2PPS  = 0x240
;   T0CKIPPS  = 0x241     T1CKIPPS = 0x242
;   T1GPPS    = 0x243     T2INPPS  = 0x248
;   CCP1PPS   = 0x24F
;   SPI1SCKPPS= 0x253     SPI1SDIPPS=0x254
;   SPI1SSPPS = 0x255
;   RX1PPS    = 0x259     CTS1PPS  = 0x25A
;   CLCIN0PPS = 0x268     CLCIN1PPS= 0x269
;   CLCIN2PPS = 0x26A     CLCIN3PPS= 0x26B
;   (see datasheet Table "PPS Input Signal Routing" for full list)
;
; Usage: #include "pinconfig.inc" from main.asm
;        (main must already have #include <xc.inc>)
;======================================================================

; ---- Code ----
PSECT code

;----------------------------------------------------------------------
; PinConfig_Init
; Configures every I/O pin: analog select, direction, output latch,
; weak pull-ups, and PPS routing.
;
; All ANSEL/WPU registers are in bank 4 (0x400-0x421).
; All LAT/TRIS registers are also in bank 4 (0x4BE-0x4CA).
; All PPS registers are in bank 2 (0x200-0x24F).
;
; Call ONCE at startup, BEFORE any peripheral _Init routine.
;----------------------------------------------------------------------
PinConfig_Init:

    ; ==================================================================
    ; 1. ANALOG SELECT  (ANSELx : 1=analog, 0=digital)
    ;    POR default is all-analog.  Set everything to digital.
    ; ==================================================================
    BANKSEL ANSELA
    clrf    BANKMASK(ANSELA), 1         ; PORTA all digital
    clrf    BANKMASK(ANSELB), 1         ; PORTB all digital (buttons)
    clrf    BANKMASK(ANSELC), 1         ; PORTC all digital (servo RC2)
    clrf    BANKMASK(ANSELD), 1         ; PORTD all digital (LED RD0)
    clrf    BANKMASK(ANSELE), 1         ; PORTE all digital

    ; ==================================================================
    ; 2. WEAK PULL-UPS  (WPUx : 1=enabled, 0=disabled)
    ;    Enable on every pin to prevent floating inputs.
    ;    Pull-ups on output pins are harmless (overridden by driver).
    ; ==================================================================
    ; WPUA is in the same bank as ANSELA (bank 4) - no BANKSEL needed
    movlw   0xFF
    movwf   BANKMASK(WPUA), 1           ; PORTA all pull-ups ON
    movwf   BANKMASK(WPUB), 1           ; PORTB all pull-ups ON (buttons)
    movwf   BANKMASK(WPUC), 1           ; PORTC all pull-ups ON
    movwf   BANKMASK(WPUD), 1           ; PORTD all pull-ups ON
    movwf   BANKMASK(WPUE), 1           ; PORTE all pull-ups ON

    ; ==================================================================
    ; 3. OUTPUT LATCHES  (LATx : initial output state)
    ;    Set before changing TRIS to avoid glitches.
    ; ==================================================================
    ; LAT registers also in bank 4 - no BANKSEL needed
    clrf    BANKMASK(LATA), 1           ; PORTA = 0x00
    clrf    BANKMASK(LATB), 1           ; PORTB = 0x00
    clrf    BANKMASK(LATC), 1           ; PORTC = 0x00  (RC2 servo LOW)
    clrf    BANKMASK(LATD), 1           ; PORTD = 0x00  (RD0 LED OFF)
    clrf    BANKMASK(LATE), 1           ; PORTE = 0x00

    ; ==================================================================
    ; 4. DATA DIRECTION  (TRISx : 1=input, 0=output)
    ;    POR default is all-input (0xFF).
    ;    Only clear bits for pins used as outputs.
    ; ==================================================================
    ; TRIS registers also in bank 4 - no BANKSEL needed
    movlw   0xFF
    movwf   BANKMASK(TRISA), 1          ; PORTA all inputs
    movwf   BANKMASK(TRISB), 1          ; PORTB all inputs (RB0,RB1 buttons)
    movlw   0xFB                        ; bit 2 clear = RC2 output (servo)
    movwf   BANKMASK(TRISC), 1
    movlw   0xF8                        ; bits 0,1,2 clear = RD0,RD1,RD2 out
    movwf   BANKMASK(TRISD), 1
    movlw   0xFF
    movwf   BANKMASK(TRISE), 1          ; PORTE all inputs (RE3=MCLR)

    ; ==================================================================
    ; 5. PPS - PERIPHERAL PIN SELECT  (bank 2: 0x200-0x26B)
    ;    Configure remappable I/O routing here.
    ;    Currently no PPS needed (servo uses CCP1 SW toggle, buttons
    ;    are direct GPIO reads).
    ; ==================================================================
    ; BANKSEL PPSLOCK                   ; all PPS registers in bank 2

    ; --- CAN Bus PPS (uncomment when using CAN) ----------------------
    ; BANKSEL CANRXPPS
    ; movlw   0x09                      ; RB1 -> CAN RX input
    ; movwf   BANKMASK(CANRXPPS), 1
    ; movlw   0x46                      ; CAN TX -> RB0 output
    ; movwf   BANKMASK(RB0PPS), 1

    ; --- UART1 PPS (example: TX on RC6, RX on RC7) -------------------
    ; BANKSEL RC6PPS
    ; movlw   0x33                      ; TX1 -> RC6 output
    ; movwf   BANKMASK(RC6PPS), 1
    ; movlw   0x17                      ; RC7 -> RX1 input
    ; movwf   BANKMASK(RX1PPS), 1

    ; --- SPI1 PPS (example: SCK=RC3, SDO=RC5, SDI=RC4, SS=RA5) ------
    ; BANKSEL RC3PPS
    ; movlw   0x20                      ; SPI1_SCK -> RC3 output
    ; movwf   BANKMASK(RC3PPS), 1
    ; movlw   0x21                      ; SPI1_SDO -> RC5 output
    ; movwf   BANKMASK(RC5PPS), 1
    ; movlw   0x14                      ; RC4 -> SPI1_SDI input
    ; movwf   BANKMASK(SPI1SDIPPS), 1
    ; movlw   0x05                      ; RA5 -> SPI1_SS input
    ; movwf   BANKMASK(SPI1SSPPS), 1

    ; --- CCP1 output PPS (DISABLED: using software toggle via LATC) ---
    ; Note: CCP compare-toggle output may not drive through PPS on
    ;       PIC18-Q84.  We toggle RC2 manually in Servo_ISR instead.
    ; BANKSEL RC2PPS
    ; movlw   0x09                      ; CCP1 -> RC2 output
    ; movwf   BANKMASK(RC2PPS), 1

    ; --- PPS Lock (uncomment to prevent accidental PPS changes) -------
    ; BANKSEL PPSLOCK
    ; movlw   0x55
    ; movwf   BANKMASK(PPSLOCK), 1
    ; movlw   0xAA
    ; movwf   BANKMASK(PPSLOCK), 1
    ; bsf     BANKMASK(PPSLOCK), 0, 1   ; PPSLOCKED = 1

    return
