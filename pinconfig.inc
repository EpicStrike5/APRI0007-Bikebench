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
; Pin   Function                                 Dir   Analog  Pull-up  PPS
; ----  ---------------------------------------  ----  ------  -------  ----------------
; RA0   Button / mode input                      in    no      YES      -
; RA1   LED / general output                     out   no      no       -
; RA2   Button / general input                   in    no      YES      -
; RA3   Button / general input                   in    no      YES      -
; RA4   Button / general input                   in    no      YES      -
; RA5   LED / general output                     out   no      no       -
; RA6   unused / LCD direct RS                   in/out no      YES      -
; RA7   unused                                   in    no      YES      -
;
; RB0   Button 1 (RIGHT) in   no      YES      -
; RB1   Button 2 (LEFT)  in   no      YES      -
; RB2   Hall input       in    no      YES      -
; RB3   CANRX            in    no      no       CANRXPPS = 0x0B
; RB4   CANTX            out   no      no       RB4PPS   = 0x46
; RB5   Servo PWM        out   no      no       -
; RB6   (unused)         in    no      YES      -
; RB7   (unused)         in    no      YES      -
;
; RC0   unused / LCD direct RW                   in/out no      no       -
; RC1   unused / LCD direct E                    in/out no      no       -
; RC2   unused / LCD direct DB0                  in/out no      no       -
; RC3   LCD I2C SCL / LCD direct DB1             in/out no      no       -
; RC4   LCD I2C SDA / LCD direct DB7             in/out no      no       -
; RC5   unused / LCD direct DB6                  in/out no      no       -
; RC6   (unused)         in    no      YES      -
; RC7   (unused)         in    no      YES      -
;
; RD0   unused / LCD direct DB2                  out   no      no       -
; RD1   unused / LCD direct DB3                  out   no      no       -
; RD2   unused / LCD direct BL-                  out   no      no       -
; RD3   unused / LCD direct BL+                  out   no      no       -
; RD4   (unused)         in    no      YES      -
; RD5   (unused)         in    no      YES      -
; RD6   unused / LCD direct DB5                  out   no      no       -
; RD7   unused / LCD direct DB4                  out   no      no       -
;
; RE0   LED / general output                     out   no      no       -
; RE1   general output                           out   no      no       -
; RE2   general output                           out   no      no       -
; RE3   MCLR             in    -       -        - (config bit)
;
; ---------------------------------------------------------------
; CAN BUS USED IN THIS PROJECT
;   RB4 -> CANTX  (out, digital, no pull-up)  RB4PPS   = 0x46
;   RB3 -> CANRX  (in,  digital, no pull-up)  CANRXPPS = 0x0B
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
    clrf    BANKMASK(ANSELC), 1         ; PORTC all digital
    clrf    BANKMASK(ANSELD), 1         ; PORTD all digital
    clrf    BANKMASK(ANSELE), 1         ; PORTE all digital

    ; ==================================================================
    ; 2. WEAK PULL-UPS  (WPUx : 1=enabled, 0=disabled)
    ;    Only enable on button pins (RB0, RB1).  All others off.
    ; ==================================================================
    ; WPUA is in the same bank as ANSELA (bank 4) - no BANKSEL needed
    movlw   0xDD                        ; buttons only
    movwf   BANKMASK(WPUA), 1 
    movlw   0x07                        ; RB0,RB1 buttons + RB2 hall pull-up
    movwf   BANKMASK(WPUB), 1           ; no pull-up on RB3/RB4 CAN pins
    clrf    BANKMASK(WPUC), 1           ; PORTC no pull-ups
    clrf    BANKMASK(WPUD), 1           ; PORTD no pull-ups
    clrf    BANKMASK(WPUE), 1           ; PORTE no pull-ups

    ; ==================================================================
    ; 3. OUTPUT LATCHES  (LATx : initial output state)
    ;    Set before changing TRIS to avoid glitches.
    ; ==================================================================
    ; LAT registers also in bank 4 - no BANKSEL needed

    clrf    BANKMASK(LATA), 1         ; RA6 = RS starts LOW, RA7 left for external contrast test
    clrf    BANKMASK(LATB), 1
    clrf    BANKMASK(LATC), 1         ; RC0..RC5 control/data start LOW
    clrf    BANKMASK(LATD), 1         ; RD0,RD1,RD6,RD7 data + RD2,RD3 BL
    clrf    BANKMASK(LATE), 1

    ; ==================================================================
    ; 4. DATA DIRECTION  (TRISx : 1=input, 0=output)
    ;    POR default is all-input (0xFF).
    ;    Only clear bits for pins used as outputs.
    ; ==================================================================
    ; TRIS registers also in bank 4 - no BANKSEL needed
    movlw   0x9D
    movwf   BANKMASK(TRISA), 1        ; RA7=input for external contrast, RA6=RS out, RA5/RA1 outputs kept
    movlw   0xCF
    movwf   BANKMASK(TRISB), 1        ; same CAN/servo setup as above
    movlw   0xC0
    movwf   BANKMASK(TRISC), 1        ; RC0..RC5 LCD outputs, RC6..RC7 inputs
    movlw   0x30
    movwf   BANKMASK(TRISD), 1        ; RD0,RD1,RD2,RD3,RD6,RD7 LCD outputs
    movlw   0x00
    movwf   BANKMASK(TRISE), 1

    ; ==================================================================
    ; 5. PPS - PERIPHERAL PIN SELECT  (bank 2: 0x200-0x26B)
    ;    Configure remappable I/O routing here.
    ;    Currently no PPS needed (servo uses CCP1 SW toggle, buttons
    ;    are direct GPIO reads).
    ; ==================================================================
    BANKSEL ODCONC
    clrf    BANKMASK(ODCONC), 1       ; direct LCD uses normal push-pull outputs

    ; --- CAN Bus PPS -------------------------------------------------
    BANKSEL CANRXPPS
    movlw   0x0B                        ; RB3 -> CANRX peripheral input
    movwf   BANKMASK(CANRXPPS), 1
    movlw   0x46                        ; CANTX peripheral output -> RB4
    movwf   BANKMASK(RB4PPS), 1


    return
