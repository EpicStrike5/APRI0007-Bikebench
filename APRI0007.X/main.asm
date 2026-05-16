; ==========================================================
; main.asm - Top-level firmware flow
; Processor: PIC18F47Q84 @ 64 MHz
;
; Coordinates startup, periodic servicing, LCD refresh,
; button handling, and automatic control.
; ==========================================================

PROCESSOR 18F47Q84

#include <xc.inc>

; ---- Configuration bits ----
CONFIG "FEXTOSC = OFF"              ; no external oscillator
CONFIG "RSTOSC = HFINTOSC_64MHZ"    ; internal 64 MHz at startup
CONFIG "CLKOUTEN = OFF"             ; clock out disabled
CONFIG "WDTE = OFF"                 ; watchdog timer disabled
CONFIG "LVP = ON"                   ; low-voltage programming enabled
CONFIG "MCLRE = EXTMCLR"           ; external master clear
CONFIG "MVECEN = ON"               ; multi-vector interrupts (REQUIRED)
CONFIG "JTAGEN = OFF"              ; JTAG disabled
CONFIG "XINST = OFF"                ; extended instruction set off
CONFIG "DEBUG = OFF"                ; background debugger disabled


; ---- Reset Vector (linked to address 0 via linker option) ----
PSECT resetVec, class=CODE, reloc=2
resetVec:
    goto    start

; ---- Included modules ----
#include "wait.inc"
#include "pinconfig.inc"
#include "servo_hw.inc"
#include "can_torque.inc"
#include "Control.inc"
#include "debounce.inc"
#include "hall.inc"
#include "lcd_direct.inc"

; ---- Main variables ----
; This file only keeps the display state needed by the top-level loop.
PSECT udata_acs
lcd_screen_mode:     DS 1       ; bit 0 selects torque/cadence vs power/speed page
control_delay_ticks: DS 1       ; shared non-blocking control cooldown/countdown (20 ms per tick)
lcd_refresh_ticks:   DS 1       ; LCD refresh countdown (20 ms per tick)
lcd_clear_request:   DS 1       ; bit 0 requests a one-shot LCD_Clear on the next refresh

; ---- Timer4 scheduler tick ----
; Fosc/4 = 16 MHz, prescaler 1:128, T4PR = 249, postscaler 1:10
; -> one interrupt every 20 ms
T4_T4CON_VAL           equ 0xF9
T4_T4CLK_VAL           equ 0x01
T4_T4HLT_VAL           equ 0x00
T4_T4RST_VAL           equ 0x00
T4_T4PR_VAL            equ 249
LCD_REFRESH_TICKS      equ 10        ; 10 x 20 ms = 200 ms

; ---- Main Code ----
PSECT code

; ----------------------------------------------------------
; Timer4_ISR
; 20 ms tick for the software counters.
; ----------------------------------------------------------
PSECT isrTimer4, class=CODE, reloc=4
Timer4_ISR:
    BANKSEL PIR11
    bcf     BANKMASK(PIR11), 3, 1      ; clear TMR4IF

    movf    control_delay_ticks, f, c
    bz      _timer4_skip_control
    decf    control_delay_ticks, f, c

_timer4_skip_control:
    movf    cadence_saved_ticks, f, c
    bz      _timer4_skip_cadence_saved
    decf    cadence_saved_ticks, f, c

_timer4_skip_cadence_saved:
    movf    lcd_refresh_ticks, f, c
    bz      _timer4_done
    decf    lcd_refresh_ticks, f, c

_timer4_done:
    retfie  1

PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     91*2
    DW      Timer4_ISR >> 2

PSECT code

; ----------------------------------------------------------
; Timer4_Init
; Configure Timer4 as the 20 ms scheduler tick.
; ----------------------------------------------------------
Timer4_Init:
    BANKSEL T4CON
    clrf    BANKMASK(T4CON), 1
    movlw   T4_T4CLK_VAL
    movwf   BANKMASK(T4CLKCON), 1
    movlw   T4_T4HLT_VAL
    movwf   BANKMASK(T4HLT), 1
    movlw   T4_T4RST_VAL
    movwf   BANKMASK(T4RST), 1
    movlw   T4_T4PR_VAL
    movwf   BANKMASK(T4PR), 1
    clrf    BANKMASK(T4TMR), 1
    movlw   T4_T4CON_VAL
    movwf   BANKMASK(T4CON), 1

    BANKSEL PIR11
    bcf     BANKMASK(PIR11), 3, 1
    BANKSEL PIE11
    bsf     BANKMASK(PIE11), 3, 1
    return

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
    call    Timer4_Init

    ; --- Startup LCD banner shown before the servo is enabled ---
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

    ; Continue with the actuator and control initialization.
    call    Servo_Init
    call    Control_Init


    ; --- Startup LED blink: confirms the system reached full init ---
    call ledE0
    
    ; --- Initialise scheduler/display state and enable interrupts ---
    movlw   1
    movwf   lcd_screen_mode, c          ; start on the power/speed screen
    clrf    control_delay_ticks, c      ; allow the first automatic control step immediately
    clrf    lcd_refresh_ticks, c        ; force the first LCD refresh immediately
    clrf    lcd_clear_request, c        ; no pending clear request at startup
    
    
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 6, 1     ; GIEL = 1  (low-priority interrupts)
    bsf     BANKMASK(INTCON0), 7, 1     ; GIE/GIEH = 1 (high-priority)
    movlw   100
    call    waitMilliSeconds

    ; ==========================================================
    ; MAIN LOOP
    ; Servo output runs in background via Timer1 overflow interrupt.
    ; Timer2 ISR (debounce.inc) updates the debounced button state.
    ; The foreground loop services sensors, refreshes the LCD, and
    ; applies user actions / automatic control decisions.
    ; ==========================================================
mainLoop:
    call    CAN_ServiceTorqueSensor
    call    ComputePower
    ; Update the LCD at a human-readable rate.
    call    LCD_RefreshDisplay
    call    Buttons_HandleMainLoop
    
    btfsc   flag_auto, 0, c
    call    GearShiftControl
    goto    mainLoop
    
ledE0:
    ; Short startup confirmation blink on RE0.
    BANKSEL LATE
    bsf     BANKMASK(LATE), 0, 1        ; LED ON
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATE), 0, 1        ; LED OFF
    return

; ------------------------------------------------------------------
; LCD refresh gate
; The main loop calls this every pass, but the LCD is only redrawn when
; the Timer4-driven refresh countdown expires.
; ------------------------------------------------------------------
LCD_RefreshDisplay:
    btfsc   lcd_clear_request, 0, c
    bra     _lcd_refresh_clear_now

    movf    lcd_refresh_ticks, f, c
    bnz     _lcd_refresh_not_due
    movlw   LCD_REFRESH_TICKS
    movwf   lcd_refresh_ticks, c
    call    LCD_PrintTargetFrame
    return

_lcd_refresh_clear_now:
    bcf     lcd_clear_request, 0, c
    movlw   LCD_REFRESH_TICKS
    movwf   lcd_refresh_ticks, c
    call    LCD_Clear
    call    LCD_PrintTargetFrame
    return

_lcd_refresh_not_due:
    return

    END     resetVec
   
