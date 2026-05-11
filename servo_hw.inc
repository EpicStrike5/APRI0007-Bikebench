;======================================================================
; servo_hw.inc - Timer1 Toggle Servo Control (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz
;
; Timer1: Fosc/4, 1:8 prescaler = 2 MHz (0.5us/tick)
; RC2 is toggled on every Timer1 overflow -> 50% duty cycle.
; The shifter now uses a discrete 8-gear lookup table instead of a
; linear position ramp. Each gear maps to one measured output frequency
; and therefore to one Timer1 preload value.
;
; Provides: Servo_Init, Servo_SetGear, Servo_SetPos, Servo_IncPos, Servo_DecPos
; Requires: pinconfig.inc (RB5 output), wait.inc (debug delays)
;           Caller enables GIE/GIEL after Servo_Init
;           Linker option: -pivt=08h
;======================================================================

; ---- Diagnostic mode ----
; SERVO_DEBUG 1: two 250ms HIGH pulses on RC2 at startup (scope test)
;                RD1 ON = Servo_Init complete
;                RD2 toggling = ISR firing
#define SERVO_DEBUG 0

; ---- Configuration ----
SERVO_PIN               equ 5         ; RB5 bit in LATB
SERVO_GEAR_MIN          equ 1         ; gear 1 = easiest gear
SERVO_GEAR_MAX          equ 8         ; gear 8 = hardest gear
SERVO_DEFAULT_GEAR      equ 3         ; start in the easiest gear
SERVO_GEAR1_PRELOAD     equ 62400     ; ~334.35 Hz
SERVO_GEAR2_PRELOAD     equ 62152     ; ~295.50 Hz
SERVO_GEAR3_PRELOAD     equ 61832     ; ~274.97 Hz
SERVO_GEAR4_PRELOAD     equ 61657     ; ~257.80 Hz
SERVO_GEAR5_PRELOAD     equ 61400     ; ~247.00 Hz
SERVO_GEAR6_PRELOAD     equ 61200     ; ~236.60 Hz
SERVO_GEAR7_PRELOAD     equ 61000     ; ~226.40 Hz
SERVO_GEAR8_PRELOAD     equ 60800     ; ~215.00 Hz
SERVO_DOWNSHIFT_BOOST   equ 500       ; temporary preload offset before settling
SERVO_DOWNSHIFT_BOOST_MS equ 400      ; boost duration in milliseconds

; ---- Variables (Access Bank) ----
PSECT udata_acs
servo_gear:     DS 1                ; current gear number (1..8)
servo_ticks_l:  DS 1                ; TMR1 preload, low byte
servo_ticks_h:  DS 1                ; TMR1 preload, high byte
servo_target_l: DS 1                ; saved normal preload low byte during boost
servo_target_h: DS 1                ; saved normal preload high byte during boost

; ---- ISR (4-byte aligned for IVT address lookup) ----
PSECT isrServo, class=CODE, reloc=4

;----------------------------------------------------------------------
; Servo_ISR - Timer1 overflow handler
; Toggles RC2 on every overflow (50% duty cycle).
; Reloads TMR1 with servo_ticks for the chosen frequency.
;----------------------------------------------------------------------
Servo_ISR:
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1        ; clear TMR1IF

#if SERVO_DEBUG
    BANKSEL LATE
    btg     BANKMASK(LATE), 0, 1        ; toggle RD2 (ISR heartbeat LED)
#endif

    ; Toggle RB5 - generates 50% duty cycle at current frequency
    BANKSEL LATB
    btg     BANKMASK(LATB), SERVO_PIN, 1

    ; Reload TMR1 (write TMR1L first, then TMR1H - required for RD16=1)
    BANKSEL TMR1L
    movf    servo_ticks_l, w, c
    movwf   BANKMASK(TMR1L), 1
    movf    servo_ticks_h, w, c
    movwf   BANKMASK(TMR1H), 1

    retfie  1

; ---- IVT entry: TMR1 overflow = IRQ 28 ----
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     28*2
    DW      Servo_ISR >> 2

; ---- Regular code ----
PSECT code

;----------------------------------------------------------------------
; Servo_Init
; Sets up Timer1, loads the default gear preload, enables TMR1 interrupt.
; Pin direction for RC2 is handled by pinconfig.inc - call that first.
; Enable GIE/GIEL separately after this returns.
;----------------------------------------------------------------------
Servo_Init:

#if SERVO_DEBUG 
    ; Blocking pin test: two 250ms HIGH pulses on RB5.
    ; Visible on scope before any interrupts start.
    ; If HIGH appears: pin, wiring, and probe are correct.
    BANKSEL LATB
    bsf     BANKMASK(LATB), SERVO_PIN, 1    ; RB5 HIGH
    movlw   250
    call    waitMilliSeconds
    BANKSEL LATB
    bcf     BANKMASK(LATB), SERVO_PIN, 1    ; RB5 LOW
    movlw   250
    call    waitMilliSeconds
    BANKSEL LATB
    bsf     BANKMASK(LATB), SERVO_PIN, 1    ; RB5 HIGH
    movlw   250
    call    waitMilliSeconds
    BANKSEL LATB
    bcf     BANKMASK(LATB), SERVO_PIN, 1    ; RB5 LOW
    movlw   250
    call    waitMilliSeconds
#endif

    ; Configure Timer1: Fosc/4, 1:8 prescaler = 2 MHz (0.5 us/tick)
    BANKSEL T1CON
    clrf    BANKMASK(T1CON), 1              ; stop timer, clear config
    clrf    BANKMASK(T1GCON), 1             ; gate disabled
    movlw   0x01
    movwf   BANKMASK(T1CLK), 1              ; clock = Fosc/4

    ; Load the default gear and its measured preload.
    movlw   SERVO_DEFAULT_GEAR
    movwf   servo_gear, c
    call    _servo_load_ticks_from_gear

    ; Load initial preload while timer is stopped and RD16=0
    ; (direct write, no buffer mechanism yet)
    BANKSEL TMR1L
    movf    servo_ticks_l, w, c
    movwf   BANKMASK(TMR1L), 1
    movf    servo_ticks_h, w, c
    movwf   BANKMASK(TMR1H), 1

    ; Start Timer1: TMR1ON=1, RD16=1, CKPS=11 (1:8 prescaler)
    BANKSEL T1CON
    movlw   0x33
    movwf   BANKMASK(T1CON), 1

    ; Enable TMR1 overflow interrupt (PIE3 bit 4)
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1            ; clear any pending TMR1IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 4, 1            ; TMR1IE = 1

#if SERVO_DEBUG
    BANKSEL LATE
    bsf     BANKMASK(LATE), 0, 1            ; RD1 ON = init complete
#endif

    return

;----------------------------------------------------------------------
; Servo_SetGear / Servo_SetPos
; Load one discrete gear from W (1..8). Servo_SetPos is kept as a
; compatibility alias because the rest of the project already calls it.
; Disables GIE during the preload update for atomic writes.
;----------------------------------------------------------------------
Servo_SetGear:
Servo_SetPos:
    movwf   servo_gear, c
    movlw   SERVO_GEAR_MIN
    subwf   servo_gear, w, c
    bc      _srvhw_setgear_check_max
    movlw   SERVO_GEAR_MIN
    movwf   servo_gear, c
    bra     _srvhw_setgear_apply

_srvhw_setgear_check_max:
    movlw   SERVO_GEAR_MAX + 1
    subwf   servo_gear, w, c
    bc      _srvhw_setgear_clamp_max
    bra     _srvhw_setgear_apply

_srvhw_setgear_clamp_max:
    movlw   SERVO_GEAR_MAX
    movwf   servo_gear, c

_srvhw_setgear_apply:
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1         ; GIE off (atomic update)
    call    _servo_load_ticks_from_gear
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1         ; GIE on
    return

;----------------------------------------------------------------------
; Servo_IncPos - move to the next easier gear, clamp at gear 1
;----------------------------------------------------------------------
Servo_IncPos:
    movlw   SERVO_GEAR_MIN
    subwf   servo_gear, w, c
    bz      _srvhw_inc_clamp
    movlw   1
    subwf   servo_gear, f, c
    call    _servo_apply_downshift_boost
    return

_srvhw_inc_clamp:
    movlw   SERVO_GEAR_MIN
    goto    Servo_SetGear

;----------------------------------------------------------------------
; Servo_DecPos - move to the next harder gear, clamp at gear 8
;----------------------------------------------------------------------
Servo_DecPos:
    movlw   SERVO_GEAR_MAX
    subwf   servo_gear, w, c
    bz      _srvhw_dec_clamp
    movlw   1
    addwf   servo_gear, f, c
    movf    servo_gear, w, c
    goto    Servo_SetGear

_srvhw_dec_clamp:
    movlw   SERVO_GEAR_MAX
    goto    Servo_SetGear

;----------------------------------------------------------------------
; _servo_apply_downshift_boost (internal)
; For easier-gear shifts, briefly drive a slightly higher preload before
; returning to the measured cruising preload for the new gear.
;----------------------------------------------------------------------
_servo_apply_downshift_boost:
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1         ; GIE off during tick update
    call    _servo_load_ticks_from_gear
    movf    servo_ticks_l, w, c
    movwf   servo_target_l, c
    movf    servo_ticks_h, w, c
    movwf   servo_target_h, c
    movlw   low(SERVO_DOWNSHIFT_BOOST)
    addwf   servo_ticks_l, f, c
    movlw   high(SERVO_DOWNSHIFT_BOOST)
    addwfc  servo_ticks_h, f, c
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1         ; GIE on so ISR uses boosted value

    movlw   SERVO_DOWNSHIFT_BOOST_MS
    call    waitMilliSeconds

    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1         ; restore normal preload atomically
    movf    servo_target_l, w, c
    movwf   servo_ticks_l, c
    movf    servo_target_h, w, c
    movwf   servo_ticks_h, c
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1
    return

;----------------------------------------------------------------------
; _servo_load_ticks_from_gear (internal)
; Loads the measured Timer1 preload for the current gear number.
;----------------------------------------------------------------------
_servo_load_ticks_from_gear:
    movlw   1
    cpfseq  servo_gear, c
    bra     _servo_load_gear2
    movlw   low(SERVO_GEAR1_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR1_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear2:
    movlw   2
    cpfseq  servo_gear, c
    bra     _servo_load_gear3
    movlw   low(SERVO_GEAR2_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR2_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear3:
    movlw   3
    cpfseq  servo_gear, c
    bra     _servo_load_gear4
    movlw   low(SERVO_GEAR3_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR3_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear4:
    movlw   4
    cpfseq  servo_gear, c
    bra     _servo_load_gear5
    movlw   low(SERVO_GEAR4_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR4_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear5:
    movlw   5
    cpfseq  servo_gear, c
    bra     _servo_load_gear6
    movlw   low(SERVO_GEAR5_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR5_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear6:
    movlw   6
    cpfseq  servo_gear, c
    bra     _servo_load_gear7
    movlw   low(SERVO_GEAR6_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR6_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear7:
    movlw   7
    cpfseq  servo_gear, c
    bra     _servo_load_gear8
    movlw   low(SERVO_GEAR7_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR7_PRELOAD)
    movwf   servo_ticks_h, c
    return

_servo_load_gear8:
    movlw   low(SERVO_GEAR8_PRELOAD)
    movwf   servo_ticks_l, c
    movlw   high(SERVO_GEAR8_PRELOAD)
    movwf   servo_ticks_h, c
    return
