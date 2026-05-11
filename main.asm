;======================================================================
; main.asm - Servo Control with Button Input
; Processor: PIC18F47Q84 @ 64MHz (Internal HFINTOSC)
;
; Hardware connections:
;   RC2 = Servo PWM output (Timer1 overflow interrupt)
;   RB0 = Button 1 - rotate RIGHT  (active LOW, internal pull-up)
;   RB1 = Button 2 - rotate LEFT   (active LOW, internal pull-up)
;   RD0 = LED feedback
;
; Diagnostic LEDs (active when SERVO_DEBUG=1 in servo_hw.inc):
;   RD1 = ON steady   -> Servo_Init completed successfully
;   RD2 = dim glow    -> Servo_ISR is firing (~100Hz toggle)
;   RC2 = two 250ms pulses at startup -> pin and wiring OK
;   If RD1 ON but RD2 dark -> interrupt not firing (IVT/config)
;   If RD2 glows but no scope on RC2 -> pin or scope issue
;
; How it works:
;   Servo PWM runs autonomously via Timer1 overflow interrupt (50 Hz).
;   The main loop detects released->pressed button edges every ~20ms.
;   Edge detection prevents a stuck/shorted pin from continuously
;   driving the servo.  CPU is free for CAN and other tasks.
;
; Servo position (0-255) maps to ~0-270 degrees:
;   0   -> ~0 deg   (400us HIGH pulse)
;   128 -> ~135 deg (~1.49ms HIGH pulse)
;   255 -> ~270 deg (~2.57ms HIGH pulse)
;======================================================================

PROCESSOR 18F47Q84

#include <xc.inc>

; ---- Configuration Bits ----
; Internal oscillator 64MHz, no watchdog, low-voltage programming on
CONFIG "FEXTOSC = OFF"              ; no external oscillator
CONFIG "RSTOSC = HFINTOSC_64MHZ"    ; internal 64 MHz at startup
CONFIG "CLKOUTEN = OFF"             ; clock out disabled
CONFIG "WDTE = OFF"                 ; watchdog timer disabled
CONFIG "LVP = ON"                   ; low-voltage programming enabled
CONFIG "MCLRE = EXTMCLR"           ; external master clear
CONFIG "MVECEN = ON"               ; multi-vector interrupts (REQUIRED)
CONFIG "JTAGEN = OFF"		    ;WTF IS THAT BUT DOES NOT WORK WITHOUT
CONFIG "XINST = OFF"                ; extended instruction set off
CONFIG "DEBUG = OFF"                ; background debugger disabled


; ---- Reset Vector (linked to address 0 via linker option) ----
PSECT resetVec, class=CODE, reloc=2
resetVec:
    goto    start

; ---- Include Libraries ----
; These files add their own PSECT udata_acs (variables) and
; PSECT code (functions). The linker auto-places everything.
#include "wait.inc"
#include "pinconfig.inc"
#include "servo_hw.inc"
#include "can_torque.inc"
#include "Control.inc"
#include "debounce.inc"
#include "hall.inc"
#include "lcd_direct.inc"

; ---- Main variables ----
; Button state now lives in debounce.inc and torque processing state now
; lives in can_torque.inc.
PSECT udata_acs
lcd_refresh_tick: DS 1          ; slows LCD updates so the value is readable
lcd_screen_mode: DS 1           ; 0 = torque/RPM screen, 1 = power screen
lcd_value_l:     DS 1           ; 16-bit LCD decimal helper scratch low byte
lcd_value_h:     DS 1           ; 16-bit LCD decimal helper scratch high byte
lcd_digit:       DS 1           ; digit accumulator for LCD decimal helper
lcd_started:     DS 1           ; leading-space suppression flag for 16-bit decimal
lcd_div_l:       DS 1           ; current 16-bit decimal divisor low byte
lcd_div_h:       DS 1           ; current 16-bit decimal divisor high byte

; ---- Main Code ----
PSECT code

start:
    ; ==========================================================
    ; I/O INITIALISATION
    ; ==========================================================

    ; --- All pins: ANSEL, TRIS, LAT, WPU, PPS (one place) ---
    call    PinConfig_Init
    call    LCD_Init
    call    Debounce_Init
    call    Buttons_Init
    call    Hall_Init
    call    CAN_Init
    call    CAN_Torque_Init

    ; --- Startup LCD banner before servo power/activity ---
    call    LCD_GotoLine1
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   'S'
    call    LCD_SendChar
    movlw   'Y'
    call    LCD_SendChar
    movlw   'S'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'M'
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar

    call    LCD_GotoLine2
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   ' '
    call    LCD_SendChar
    movlw   'B'
    call    LCD_SendChar
    movlw   'O'
    call    LCD_SendChar
    movlw   'O'
    call    LCD_SendChar
    movlw   'T'
    call    LCD_SendChar
    movlw   'I'
    call    LCD_SendChar
    movlw   'N'
    call    LCD_SendChar
    movlw   'G'
    call    LCD_SendChar

    ; Delay servo start to reduce peak startup current demand on the battery.
    ; 2 s is a conservative compromise: enough for supply/BMS settling
    ; without making the system feel slow to boot.
    movlw   2
    call    waitSeconds

    ; Continue with the higher-current / motion-capable peripherals.
    call    Servo_Init
    call    Control_Init

    ; Update the startup banner once the servo and the rest are active.
    call    LCD_GotoLine2
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
    movlw   'R'
    call    LCD_SendChar
    movlw   'E'
    call    LCD_SendChar
    movlw   'A'
    call    LCD_SendChar
    movlw   'D'
    call    LCD_SendChar
    movlw   'Y'
    call    LCD_SendChar
    movlw   '!'
    call    LCD_SendChar
    
    movlw   1
    call    waitSeconds

    ; --- Startup LED blink: confirms the system reached full init ---
    call ledE0
    
    ; --- Initialise peripherals and enable interrupts ---
    
    clrf    lcd_refresh_tick, c         ; LCD refresh divider starts from 0
    clrf    lcd_screen_mode, c          ; start on the torque/RPM screen
    
    
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 6, 1     ; GIEL = 1  (low-priority interrupts)
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE/GIEH = 1 (high-priority)
    movlw   100
    call    waitMilliSeconds

    ; ==========================================================
    ; MAIN LOOP
    ; Servo PWM runs in background via Timer1 overflow interrupt.
    ; Timer2 ISR (debounce.inc) updates db_stable every 10ms.
    ;
    ; CAN and torque processing are handled by can_torque.inc.
    ; Button edge detection and actions are handled by debounce.inc.
    ; ==========================================================
mainLoop:
    call    CAN_ServiceTorqueSensor
    call    ComputePower
    ; Update the LCD at a human-readable rate.
    call    LCD_RefreshDisplay
    call    Buttons_HandleMainLoop
    goto    mainLoop
    
ledE0:
    BANKSEL LATE
    bsf     BANKMASK(LATE), 0, 1        ; LED ON
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATE), 0, 1        ; LED OFF
    
    return
    
ledA5:
    BANKSEL LATA
    btg     BANKMASK(LATA), 5, 1        ; LED ON
    movlw   250
    call    waitMilliSeconds
    btg     BANKMASK(LATA), 5, 1        ; LED OFF
    
    return

; ------------------------------------------------------------------
; LCD helpers
; The CAN parser runs every loop, but the LCD is refreshed more slowly
; so the user can actually read the values.
; ------------------------------------------------------------------
LCD_RefreshDisplay:
    incf    lcd_refresh_tick, f, c
    movlw   20
    cpfseq  lcd_refresh_tick, c
    return
    clrf    lcd_refresh_tick, c
    call    LCD_PrintTargetFrame
    return

; ------------------------------------------------------------------
; LCD_PrintTargetFrame
; Line 1: TQ[Nm]:value
; Line 2: RPM[rpm]:value
; ------------------------------------------------------------------
LCD_PrintTargetFrame:
    btfsc   flag_cadence_setting, 0, c
    bra     LCD_PrintCadenceSettingFrame

    movf    lcd_screen_mode, w, c
    bnz     LCD_PrintPowerFrame

    
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
    return

LCD_PrintCadenceSettingFrame:
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
    return

LCD_PrintPowerFrame:
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
    movlw   ' '
    call    LCD_SendChar
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
    movlw   ' '
    call    LCD_SendChar
    movlw   '['
    call    LCD_SendChar
    movlw   'k'
    call    LCD_SendChar
    movlw   'm'
    call    LCD_SendChar
    movlw   '/'
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
LCD_SendDec16:
    clrf    lcd_started, c
    movlw   high(10000)
    movwf   lcd_div_h, c
    movlw   low(10000)
    movwf   lcd_div_l, c
    call    _LCD_SendDec16Digit

    movlw   high(1000)
    movwf   lcd_div_h, c
    movlw   low(1000)
    movwf   lcd_div_l, c
    call    _LCD_SendDec16Digit

    movlw   high(100)
    movwf   lcd_div_h, c
    movlw   low(100)
    movwf   lcd_div_l, c
    call    _LCD_SendDec16Digit

    movlw   high(10)
    movwf   lcd_div_h, c
    movlw   low(10)
    movwf   lcd_div_l, c
    call    _LCD_SendDec16Digit

    movf    lcd_value_l, w, c
    addlw   '0'
    call    LCD_SendChar
    return

_LCD_SendDec16Digit:
    clrf    lcd_digit, c

_LCD_SendDec16Loop:
    movf    lcd_div_l, w, c
    subwf   lcd_value_l, f, c
    movf    lcd_div_h, w, c
    subwfb  lcd_value_h, f, c
    bc      _LCD_SendDec16Subtracted

    movf    lcd_div_l, w, c
    addwf   lcd_value_l, f, c
    movf    lcd_div_h, w, c
    addwfc  lcd_value_h, f, c
    bra     _LCD_SendDec16Emit

_LCD_SendDec16Subtracted:
    incf    lcd_digit, f, c
    bra     _LCD_SendDec16Loop

_LCD_SendDec16Emit:
    movf    lcd_digit, w, c
    bnz     _LCD_SendDec16EmitDigit
    movf    lcd_started, w, c
    bnz     _LCD_SendDec16EmitDigit
    return

_LCD_SendDec16EmitDigit:
    movlw   1
    movwf   lcd_started, c
    movf    lcd_digit, w, c
    addlw   '0'
    call    LCD_SendChar
    return

    END     resetVec
   
