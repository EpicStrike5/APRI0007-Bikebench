;======================================================================
; wait.asm - Delay Routines Library (pic-as syntax)
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Provides:
;   waitMilliSeconds  - busy wait for W milliseconds (1-255)
;   waitSeconds       - busy wait for W seconds (1-255)
;
; Usage: #include "wait.asm" from your main source file
;        (main must already have #include <xc.inc>)
;======================================================================

; ---- Variables (Access Bank - auto allocated by linker) ----
PSECT udata_acs
waitSecondsValue:   DS 1
waitMSeconds:       DS 1
waitInner:          DS 1
waitInnerMost:      DS 1
waitPad:            DS 1

; ---- Code ----
PSECT code

;----------------------------------------------------------------------
; waitMilliSeconds
; Input:  W = number of milliseconds to wait (1-255)
; Timing: calibrated for 64 MHz (Fosc/4 = 16 MIPS)
;         1 ms = 16,000 instruction cycles
;                                       ; cycles
;                                       ;  imost  inner  outer  total
;----------------------------------------------------------------------
waitMilliSeconds:                       ;  -      -      -      2 (call)
    nop                                 ;  -      -      -      3
    movwf   waitMSeconds, c             ;  -      -      -      4
wms_outerLoop:
    movlw   49                          ;  -      -      1      5
    movwf   waitInner, c                ;  -      -      2      6
wms_innerLoop:
    movlw   64                          ;  -      1      3      7
    movwf   waitInnerMost, c            ;  -      2      4      8
wms_innerMostLoop:
    nop                                 ;  1
    nop                                 ;  2
    decfsz  waitInnerMost, f, c         ;  3
    goto    wms_innerMostLoop           ;  5      315
    ; decfsz-branch                     ;  4      319
    nop                                 ;  -      320
    nop                                 ;  -      321
    decfsz  waitInner, f, c             ;  -      322
    goto    wms_innerLoop               ;  -      324    15648
    ; decfsz-branch                     ;  -      323    15973
    movlw   5                           ;  -      -      15974
    movwf   waitPad, c                  ;  -      -      15975
wms_padLoop:
    decfsz  waitPad, f, c              ;  1
    goto    wms_padLoop                ;  3      12
    ; decfsz-branch                    ;  2      14     15989
    nop                                ;  -      -      15990
    decfsz  waitMSeconds, f, c         ;  -      -      15991
    goto    wms_goOuterLoop            ;  -      -      15993
    ; decfsz-branch                    ;  -      -      15992  15996
    return                             ;  -      -      -      15998

wms_goOuterLoop:
    nop                                ;  15994  15999
    nop                                ;  15995  16000
    nop                                ;  15996  16001
    nop                                ;  15997  16002
    nop                                ;  15998  16003
    goto    wms_outerLoop              ;  16000  16005

;----------------------------------------------------------------------
; waitSeconds
; Input:  W = number of seconds to wait (1-255)
;----------------------------------------------------------------------
waitSeconds:
    nop                                 ; 1 (keeps original cycle count)
    movwf   waitSecondsValue, c         ; 1
ws_loop:
    movlw   250
    call    waitMilliSeconds
    movlw   250
    call    waitMilliSeconds
    movlw   250
    call    waitMilliSeconds
    movlw   250
    call    waitMilliSeconds
    decfsz  waitSecondsValue, f, c
    goto    ws_loop
    return
