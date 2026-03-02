;======================================================================
; servo.asm - Servo Control Library (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Hardware: Software bit-bang PWM on pin RC2
; Servo:   270 degree range
;
; Position mapping (0-255):
;   pos = 0   -> ~0 deg   (0.5ms pulse)
;   pos = 128 -> ~135 deg (1.5ms pulse, center)
;   pos = 255 -> ~270 deg (2.5ms pulse)
;
; Provides:
;   Servo_Init    - configure RC2 as output, set center position
;   Servo_SetPos  - set position from W register (0-255)
;   Servo_Pulse   - generate one complete servo pulse (~20ms)
;   Servo_IncPos  - increase position by SERVO_STEP, clamp at 255
;   Servo_DecPos  - decrease position by SERVO_STEP, clamp at 0
;
; Usage: #include "servo.asm" from your main source file
;        Requires wait.asm (for waitMilliSeconds)
;        Main must already have #include <xc.inc>
;======================================================================

; ---- Configuration ----
#define SERVO_PIN       2       ; bit number on PORTC (RC2)
#define SERVO_STEP      5       ; position increment per button press (~5.3 deg)
#define SERVO_DEFAULT   128     ; center position (~135 deg)

; ---- Variables (Access Bank - auto allocated by linker) ----
PSECT udata_acs
servo_pos:          DS 1        ; current position (0-255)
servo_delayOuter:   DS 1        ; temp for delay loops
servo_delayInner:   DS 1        ; temp for delay loops

; ---- Code ----
PSECT code

;----------------------------------------------------------------------
; Servo_Init
; Configures RC2 as a digital output and sets center position.
;----------------------------------------------------------------------
Servo_Init:
    ; RC2 = digital (clear analog select)
    BANKSEL ANSELC
    bcf     BANKMASK(ANSELC), SERVO_PIN, b

    ; RC2 = output
    BANKSEL TRISC
    bcf     BANKMASK(TRISC), SERVO_PIN, b

    ; RC2 = start LOW
    BANKSEL LATC
    bcf     BANKMASK(LATC), SERVO_PIN, b

    ; default to center position
    movlw   SERVO_DEFAULT
    movwf   servo_pos, c
    return

;----------------------------------------------------------------------
; Servo_SetPos
; Input: W = new position (0-255)
;----------------------------------------------------------------------
Servo_SetPos:
    movwf   servo_pos, c
    return

;----------------------------------------------------------------------
; Servo_Pulse
; Generates one complete servo pulse on RC2.
;
; Pulse HIGH = base 0.5ms + variable 0-2ms (depends on servo_pos)
; Pulse LOW  = fixed 18ms
; Total period ~ 18.5 - 20.5 ms (49-54 Hz) -- fine for any servo
;
; Timing detail (at 16 MIPS, 1 cycle = 62.5 ns):
;   Base delay   : 32 x 248 = 7936 cycles  ~ 0.496 ms
;   Variable delay: pos x 123 cycles        ~ 0 to 1.96 ms
;   Total HIGH   : 0.496 to 2.46 ms         ~ matches 0.5-2.5 ms spec
;----------------------------------------------------------------------
Servo_Pulse:
    ; === Set pin HIGH ===
    BANKSEL LATC
    bsf     BANKMASK(LATC), SERVO_PIN, b

    ; === Base delay ~0.5ms ===
    ; 32 outer iterations x ~248 cycles = ~7936 cycles
    movlw   32
    movwf   servo_delayOuter, c
_sp_baseOuter:
    movlw   82                              ; inner count
    movwf   servo_delayInner, c             ; 1
_sp_baseInner:
    decfsz  servo_delayInner, f, c          ; 1
    goto    _sp_baseInner                   ; 2  -> (82-1)*3 + 2 = 245
    decfsz  servo_delayOuter, f, c          ; 1  -> 246
    goto    _sp_baseOuter                   ; 2  -> 248 per outer iteration
    ; Total: ~32 * 248 = 7936 cycles ~ 0.496 ms

    ; === Variable delay: servo_pos x ~123 cycles ===
    ; Adds 0 to ~1.96ms depending on position value
    movf    servo_pos, w, c
    bz      _sp_varDone                     ; if pos = 0 skip entirely
    movwf   servo_delayOuter, c
_sp_varOuter:
    movlw   39                              ; 1
    movwf   servo_delayInner, c             ; 1
_sp_varInner:
    decfsz  servo_delayInner, f, c          ; 1
    goto    _sp_varInner                    ; 2  -> (39-1)*3 + 2 = 116
    nop                                     ; 117
    nop                                     ; 118
    nop                                     ; 119
    decfsz  servo_delayOuter, f, c          ; 120
    goto    _sp_varOuter                    ; 122
    ; + movlw(1) + movwf(1) = 124 per outer - close enough
    ; 255 * ~123 = 31365 cycles ~ 1.96 ms
_sp_varDone:

    ; === Set pin LOW ===
    BANKSEL LATC
    bcf     BANKMASK(LATC), SERVO_PIN, b

    ; === Wait remaining period (~18ms) ===
    movlw   18
    call    waitMilliSeconds
    return

;----------------------------------------------------------------------
; Servo_IncPos
; Increases servo_pos by SERVO_STEP, clamped at 255 (max angle).
;----------------------------------------------------------------------
Servo_IncPos:
    movlw   SERVO_STEP
    addwf   servo_pos, f, c         ; servo_pos += SERVO_STEP
    bc      _inc_overflow            ; carry = overflowed past 255
    return
_inc_overflow:
    movlw   255
    movwf   servo_pos, c            ; clamp to max
    return

;----------------------------------------------------------------------
; Servo_DecPos
; Decreases servo_pos by SERVO_STEP, clamped at 0 (min angle).
;----------------------------------------------------------------------
Servo_DecPos:
    movlw   SERVO_STEP
    subwf   servo_pos, f, c         ; servo_pos -= SERVO_STEP
    bnc     _dec_underflow           ; no carry = borrow = underflow
    return
_dec_underflow:
    clrf    servo_pos, c            ; clamp to min
    return
