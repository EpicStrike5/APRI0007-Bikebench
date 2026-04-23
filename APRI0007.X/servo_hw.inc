;======================================================================
; servo_hw.inc - Timer1 Toggle Servo Control (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz
;
; Timer1: Fosc/4, 1:8 prescaler = 2 MHz (0.5us/tick)
; RC2 is toggled on every Timer1 overflow -> 50% duty cycle.
; Servo position (0-255) maps to output frequency:
;
;   pos =   0 ->  5000 ticks half-period ->  200 Hz
;   pos = 255 ->  1175 ticks half-period -> ~851 Hz
;
; Preload formula: 60536 + pos * 15
;   60536 = 65536 - 5000  (200 Hz base)
;   pos * 15 = pos * 16 - pos (shift then subtract)
;
; Provides: Servo_Init, Servo_SetPos, Servo_IncPos, Servo_DecPos
; Requires: pinconfig.inc (RC2 output), wait.inc (debug delays)
;           Caller enables GIE/GIEL after Servo_Init
;           Linker option: -pivt=08h
;======================================================================

; ---- Diagnostic mode ----
; SERVO_DEBUG 1: two 250ms HIGH pulses on RC2 at startup (scope test)
;                RD1 ON = Servo_Init complete
;                RD2 toggling = ISR firing
#define SERVO_DEBUG 1

; ---- Configuration ----
SERVO_PIN           equ 5           ; RB5 bit in LATB
SERVO_STEP          equ 20          ; position increment per button press
SERVO_DEFAULT       equ 128         ; starting position
SERVO_PRELOAD_BASE  equ 60536       ; TMR1 preload at pos=0  (200 Hz)
                                    ; 60536 = 0xEC78

; ---- Variables (Access Bank) ----
PSECT udata_acs
servo_pos:      DS 1                ; current position (0-255)
servo_ticks_l:  DS 1                ; TMR1 preload, low byte
servo_ticks_h:  DS 1                ; TMR1 preload, high byte

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
    BANKSEL LATD
    btg     BANKMASK(LATD), 2, 1        ; toggle RD2 (ISR heartbeat LED)
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
; Sets up Timer1, computes initial preload, enables TMR1 interrupt.
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

    ; Compute preload for default position
    movlw   SERVO_DEFAULT
    movwf   servo_pos, c
    call    _servo_calc_ticks

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
    BANKSEL LATD
    bsf     BANKMASK(LATD), 1, 1            ; RD1 ON = init complete
#endif

    return

;----------------------------------------------------------------------
; Servo_SetPos - set position from W (0-255)
; Disables GIE during preload update for atomic write.
;----------------------------------------------------------------------
Servo_SetPos:
    movwf   servo_pos, c
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1         ; GIE off (atomic update)
    call    _servo_calc_ticks
    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1         ; GIE on
    return

;----------------------------------------------------------------------
; Servo_IncPos - increase position by SERVO_STEP, clamp at 255
;----------------------------------------------------------------------
Servo_IncPos:
    movlw   SERVO_STEP
    addwf   servo_pos, f, c
    bc      _srvhw_inc_clamp
    movf    servo_pos, w, c
    goto    Servo_SetPos
_srvhw_inc_clamp:
    movlw   255
    goto    Servo_SetPos

;----------------------------------------------------------------------
; Servo_DecPos - decrease position by SERVO_STEP, clamp at 0
;----------------------------------------------------------------------
Servo_DecPos:
    movlw   SERVO_STEP
    subwf   servo_pos, f, c
    bnc     _srvhw_dec_clamp
    movf    servo_pos, w, c
    goto    Servo_SetPos
_srvhw_dec_clamp:
    movlw   0
    goto    Servo_SetPos

;----------------------------------------------------------------------
; _servo_calc_ticks (internal)
; Computes TMR1 preload for current servo_pos:
;
;   preload = 60536 + pos * 15
;   pos * 15 = pos * 16 - pos  (4 left-shifts, then subtract pos)
;
; Result -> servo_ticks_h:servo_ticks_l
;----------------------------------------------------------------------
_servo_calc_ticks:

    ; Step 1: servo_ticks = pos * 16
    clrf    servo_ticks_h, c
    movf    servo_pos, w, c
    movwf   servo_ticks_l, c

    bcf     STATUS, 0, c
    rlcf    servo_ticks_l, f, c             ; x2
    rlcf    servo_ticks_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_ticks_l, f, c             ; x4
    rlcf    servo_ticks_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_ticks_l, f, c             ; x8
    rlcf    servo_ticks_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_ticks_l, f, c             ; x16
    rlcf    servo_ticks_h, f, c

    ; Step 2: servo_ticks = pos * 15  (subtract pos from pos*16)
    movf    servo_pos, w, c
    subwf   servo_ticks_l, f, c             ; low byte: ticks_l -= pos
    btfss   STATUS, 0, c                    ; C=0 = borrow occurred
    decf    servo_ticks_h, f, c             ; propagate borrow to high byte

    ; Step 3: servo_ticks += 60536  (0xEC78)
    movlw   low(SERVO_PRELOAD_BASE)         ; 0x78
    addwf   servo_ticks_l, f, c
    movlw   high(SERVO_PRELOAD_BASE)        ; 0xEC
    addwfc  servo_ticks_h, f, c

    return
