;======================================================================
; servo_hw.inc - Timer1 Overflow Servo Control Library (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Hardware: Timer1 overflow interrupt for interrupt-driven servo PWM
;           Output pin: RC2 (toggled via LATC in ISR)
;           Timer1: Fosc/4 with 1:8 prescaler = 2 MHz (0.5us/tick)
;
; Approach:
;   Timer1 overflows at the end of each phase (HIGH or LOW).
;   The ISR toggles RC2 and preloads TMR1 with the duration of
;   the next phase.  This generates a 50 Hz servo signal without
;   using the CCP module at all.
;
;   Inspired by the "software PWM via Timer1 overflow" technique
;   but using one-shot preloads for full 16-bit timing precision
;   instead of a coarse step counter.
;
; Servo: 270 degree range, position 0-255
; Position mapping:
;   pos = 0   -> 0 deg   (0.5ms pulse = 1000 ticks)
;   pos = 128 -> ~135 deg (~1.52ms pulse, center)
;   pos = 255 -> ~270 deg (~2.54ms pulse = 5080 ticks)
;
; Period: 20ms (50 Hz) = 40000 ticks at 2 MHz
;
; TMR1 preload = 65536 - ticks  (two's complement)
;   Timer counts from preload up to overflow (65536) = exact ticks.
;
; The servo PWM runs via Timer1 overflow interrupt - CPU is free
; for CAN polling and other tasks between ISR calls.
;
; Provides:
;   Servo_Init    - configure Timer1, enable TMR1 interrupt
;   Servo_SetPos  - set position from W register (0-255)
;   Servo_IncPos  - increase position by SERVO_STEP, clamp at 255
;   Servo_DecPos  - decrease position by SERVO_STEP, clamp at 0
;   Servo_ISR     - interrupt handler (called from TMR1 vector)
;
; Replaces: servo.inc (software bit-bang version)
; Usage: #include "servo_hw.inc" from main.asm
;        Requires: #include <xc.inc> before this file
;        Requires: wait.inc (for startup delay)
;        Requires: pinconfig.inc (RC2 pin setup done externally)
;        Linker option: -pivt=08h  (shared IVT PSECT)
;        Caller must enable GIE/GIEL after Servo_Init returns
;======================================================================

; ---- Diagnostic mode ----
; Set to 1 to enable diagnostic LEDs (wire LEDs to RD1 and RD2):
;   - At startup: two ~250ms pulses on RC2 (visible on scope)
;   - RD1 ON steady  = Servo_Init completed OK
;   - RD2 toggling   = Servo_ISR is firing (appears as dim glow)
; Set to 0 to disable all diagnostics.
#define SERVO_DEBUG 1

; ---- Configuration ----
SERVO_PIN       equ 2           ; bit number on PORTC (RC2)
SERVO_STEP      equ 5           ; increment per button press (~5.3 deg)
SERVO_DEFAULT   equ 128         ; center position (~135 deg)
SERVO_PERIOD    equ 40000       ; 20ms at 2 MHz (0.5us/tick)
SERVO_MIN       equ 1000        ; 0.5ms minimum pulse width (ticks)

; TMR1 preload for initial 20ms delay before first pulse
SERVO_PRELOAD   equ 0x10000 - SERVO_PERIOD  ; = 25536 = 0x63C0

; ---- Variables (Access Bank) ----
PSECT udata_acs
servo_pos:      DS 1            ; current position (0-255)
servo_state:    DS 1            ; ISR state: 0=next rising, 1=next falling
servo_high_l:   DS 1            ; TMR1 preload for HIGH phase, low byte
servo_high_h:   DS 1            ; TMR1 preload for HIGH phase, high byte
servo_low_l:    DS 1            ; TMR1 preload for LOW phase, low byte
servo_low_h:    DS 1            ; TMR1 preload for LOW phase, high byte

; ---- ISR Code (4-byte aligned for IVT address lookup) ----
PSECT isrServo, class=CODE, reloc=4

;----------------------------------------------------------------------
; Servo_ISR
; Timer1 overflow interrupt handler.
; Toggles RC2 via LATC and preloads TMR1 for the next phase.
; Uses FAST context save/restore (shadow registers via retfie 1).
;
; Called automatically from the TMR1 IVT entry (IRQ 28).
;----------------------------------------------------------------------
Servo_ISR:
    ; Clear TMR1 interrupt flag (PIR3 bit 4, bank 4)
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1

#if SERVO_DEBUG
    ; DEBUG: toggle RD2 every ISR call (~100Hz -> appears as dim glow)
    ; LATD (0x04C2) is in bank 4, same as PIR3 - no extra BANKSEL
    btg     BANKMASK(LATD), 2, 1
#endif

    ; Branch on servo state
    btfsc   servo_state, 0, c           ; 0 = time for rising edge
    goto    _isr_falling

    ; === Rising edge: drive RC2 HIGH, preload for HIGH duration ===
    ; LATC is also in bank 4 (0x04C0) - same bank as PIR3
    bsf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 = HIGH (pulse start)

    ; Preload TMR1 for HIGH phase duration (TMR1 regs in bank 3)
    ; With RD16=1: write TMR1L (buffer), write TMR1H (transfers both)
    BANKSEL TMR1L
    movf    servo_high_l, w, c
    movwf   BANKMASK(TMR1L), 1          ; write to buffer
    movf    servo_high_h, w, c
    movwf   BANKMASK(TMR1H), 1          ; transfers both to counter

    bsf     servo_state, 0, c           ; state = 1 (falling next)
    retfie  1                            ; FAST return

_isr_falling:
    ; === Falling edge: drive RC2 LOW, preload for LOW duration ===
    ; Still in bank 4 from BANKSEL PIR3 above
    bcf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 = LOW (pulse end)

    ; Preload TMR1 for LOW phase duration
    BANKSEL TMR1L
    movf    servo_low_l, w, c
    movwf   BANKMASK(TMR1L), 1
    movf    servo_low_h, w, c
    movwf   BANKMASK(TMR1H), 1

    bcf     servo_state, 0, c           ; state = 0 (rising next)
    retfie  1                            ; FAST return

; ---- TMR1 Interrupt Vector Table Entry ----
; TMR1 overflow is IRQ 28 (PIR3 bit 4).
; IVT entry = 2-byte address word at IVTBASE + 2*28.
PSECT ivt, class=CODE, reloc=2, ovrld
    ORG     28*2                             ; byte offset for IRQ 28
    DW      Servo_ISR >> 2                   ; ISR address / 4

; ---- Regular code ----
PSECT code

;----------------------------------------------------------------------
; Servo_Init
; Configures Timer1 (2 MHz), enables TMR1 overflow interrupt,
; sets default center position.
;
; No CCP module used - pure Timer1 overflow approach.
;
; Pin I/O for RC2 is handled by pinconfig.inc.
; Call PinConfig_Init BEFORE this routine.
; Enable GIE separately after.
;----------------------------------------------------------------------
Servo_Init:

#if SERVO_DEBUG
    ; === DEBUG: Pin test ===
    ; Generate two 250ms pulses on RC2 BEFORE Timer1 starts.
    ; If you see these on the scope, RC2 pin and wiring are OK.
    ; If not, it is a hardware/wiring problem.
    BANKSEL LATC
    bsf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 HIGH
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 LOW
    movlw   250
    call    waitMilliSeconds
    bsf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 HIGH
    movlw   250
    call    waitMilliSeconds
    bcf     BANKMASK(LATC), SERVO_PIN, 1    ; RC2 LOW
    movlw   250
    call    waitMilliSeconds
#endif

    ; --- Timer1: Fosc/4 (16 MHz), 1:8 prescaler = 2 MHz ---
    ; All Timer1 registers are in bank 3
    BANKSEL T1CON
    clrf    BANKMASK(T1CON), 1              ; stop Timer1 first
    clrf    BANKMASK(T1GCON), 1             ; gate disabled
    movlw   0x01
    movwf   BANKMASK(T1CLK), 1             ; clock = Fosc/4

    ; Preload TMR1 for initial 20ms delay before first pulse
    ; Timer stopped + RD16 not yet set -> writes go directly to counter
    movlw   low(SERVO_PRELOAD)              ; 0xC0
    movwf   BANKMASK(TMR1L), 1
    movlw   high(SERVO_PRELOAD)             ; 0x63
    movwf   BANKMASK(TMR1H), 1

    ; Start Timer1: ON=1, RD16=1, CKPS=11 (1:8 prescaler)
    movlw   0x33
    movwf   BANKMASK(T1CON), 1

    ; --- Default position + pre-compute TMR1 preloads ---
    clrf    servo_state, c                  ; start: rising edge next
    movlw   SERVO_DEFAULT
    movwf   servo_pos, c
    call    _servo_calc_pulse

    ; --- Enable TMR1 overflow interrupt (PIR3/PIE3 bit 4, bank 4) ---
    BANKSEL PIR3
    bcf     BANKMASK(PIR3), 4, 1           ; clear any pending TMR1IF
    BANKSEL PIE3
    bsf     BANKMASK(PIE3), 4, 1           ; TMR1IE = 1

#if SERVO_DEBUG
    ; DEBUG: RD1 ON = Servo_Init completed (Timer1 running, TMR1IE set)
    ; LATD is in bank 4, same as PIE3 - no extra BANKSEL
    bsf     BANKMASK(LATD), 1, 1
#endif

    ; Caller must enable GIE after this returns
    return

;----------------------------------------------------------------------
; Servo_SetPos
; Input: W = new position (0-255)
; Recalculates TMR1 preloads. Briefly disables GIE for atomicity.
;----------------------------------------------------------------------
Servo_SetPos:
    movwf   servo_pos, c

    ; Disable interrupts while updating timing values
    BANKSEL INTCON0
    bcf     BANKMASK(INTCON0), 7, 1        ; GIE = 0

    call    _servo_calc_pulse

    BANKSEL INTCON0
    bsf     BANKMASK(INTCON0), 7, 1        ; GIE = 1
    return

;----------------------------------------------------------------------
; Servo_IncPos
; Increases servo_pos by SERVO_STEP, clamped at 255, then recalculates.
;----------------------------------------------------------------------
Servo_IncPos:
    movlw   SERVO_STEP
    addwf   servo_pos, f, c
    bc      _shw_inc_clamp                  ; carry = overflow past 255
    movf    servo_pos, w, c
    goto    Servo_SetPos
_shw_inc_clamp:
    movlw   255
    goto    Servo_SetPos

;----------------------------------------------------------------------
; Servo_DecPos
; Decreases servo_pos by SERVO_STEP, clamped at 0, then recalculates.
;----------------------------------------------------------------------
Servo_DecPos:
    movlw   SERVO_STEP
    subwf   servo_pos, f, c
    bnc     _shw_dec_clamp                  ; no carry = borrow = underflow
    movf    servo_pos, w, c
    goto    Servo_SetPos
_shw_dec_clamp:
    movlw   0
    goto    Servo_SetPos

;----------------------------------------------------------------------
; _servo_calc_pulse (internal)
; Computes TMR1 preload values from servo_pos.
;
; pulse_ticks  = SERVO_MIN + servo_pos * 16
;              = 1000 + pos * 16   (range 1000..5080)
; remain_ticks = SERVO_PERIOD - pulse_ticks
;              = 40000 - pulse_ticks  (range 34920..39000)
;
; TMR1 preload = 65536 - ticks  (two's complement negation)
;   comf + comf + infsnz/incf  implements 16-bit negation.
;
; Results stored in servo_high_h:servo_high_l (HIGH phase preload)
;                and servo_low_h:servo_low_l  (LOW phase preload)
;----------------------------------------------------------------------
_servo_calc_pulse:
    ; --- Step 1: 16-bit result = servo_pos * 16 (shift left 4) ---
    clrf    servo_high_h, c
    movf    servo_pos, w, c
    movwf   servo_high_l, c

    bcf     STATUS, 0, c                    ; clear carry
    rlcf    servo_high_l, f, c              ; x2
    rlcf    servo_high_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_high_l, f, c              ; x4
    rlcf    servo_high_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_high_l, f, c              ; x8
    rlcf    servo_high_h, f, c
    bcf     STATUS, 0, c
    rlcf    servo_high_l, f, c              ; x16
    rlcf    servo_high_h, f, c

    ; --- Step 2: add SERVO_MIN (1000 = 0x03E8) ---
    movlw   low(SERVO_MIN)                  ; 0xE8
    addwf   servo_high_l, f, c
    movlw   high(SERVO_MIN)                 ; 0x03
    addwfc  servo_high_h, f, c              ; + carry from low byte
    ; servo_high = pulse_ticks (1000..5080)

    ; --- Step 3: servo_low = SERVO_PERIOD - servo_high ---
    ; Low byte: W = low(40000) - servo_high_l
    movf    servo_high_l, w, c
    sublw   low(SERVO_PERIOD)               ; W = 0x40 - high_l
    movwf   servo_low_l, c
    ; High byte: W = high(40000) - servo_high_h - borrow
    movf    servo_high_h, w, c
    btfss   STATUS, 0, c                    ; skip if C=1 (no borrow)
    addlw   1                               ; compensate for borrow
    sublw   high(SERVO_PERIOD)              ; W = 0x9C - adjusted
    movwf   servo_low_h, c
    ; servo_low = remaining ticks (34920..39000)

    ; --- Step 4: negate servo_high -> TMR1 preload = 65536 - pulse_ticks ---
    comf    servo_high_l, f, c              ; one's complement low
    comf    servo_high_h, f, c              ; one's complement high
    infsnz  servo_high_l, f, c              ; +1 low; skip if no wrap to 0
    incf    servo_high_h, f, c              ; propagate carry to high

    ; --- Step 5: negate servo_low -> TMR1 preload = 65536 - remain_ticks ---
    comf    servo_low_l, f, c
    comf    servo_low_h, f, c
    infsnz  servo_low_l, f, c
    incf    servo_low_h, f, c

    return
