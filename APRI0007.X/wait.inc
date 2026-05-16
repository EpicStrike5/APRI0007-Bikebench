; ==========================================================
; wait.inc - Blocking delay routines
; Processor: PIC18F47Q84 @ 64 MHz
;
; Provides simple microsecond, millisecond, and second
; delay routines used where short blocking waits are acceptable.
; ==========================================================

; ---- Variables ----
PSECT udata_acs
waitSecondsValue:   DS 1
waitMSeconds:       DS 1
waitInner:          DS 1
waitInnerMost:      DS 1
waitPad:            DS 1
waituSeconds:       DS 1
waitu:              DS 1

; ---- Regular code ----
PSECT code

; ---- waitMicroSeconds ----
waitMicroSeconds:
    movwf   waituSeconds, c
wus_loop:
    movlw   5
    movwf   waitu, c
wu_loop:
    decfsz  waitu, f, c
    goto    wu_loop
    decfsz  waituSeconds, f, c
    goto    wus_loop
    return

; ---- waitMilliSeconds ----
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
    nop                                 ;  -      320
    nop                                 ;  -      321
    decfsz  waitInner, f, c             ;  -      322
    goto    wms_innerLoop               ;  -      324    15648
    movlw   5                           ;  -      -      15974
    movwf   waitPad, c                  ;  -      -      15975
wms_padLoop:
    decfsz  waitPad, f, c              ;  1
    goto    wms_padLoop                ;  3      12
    nop                                ;  -      -      15990
    decfsz  waitMSeconds, f, c         ;  -      -      15991
    goto    wms_goOuterLoop            ;  -      -      15993
    return                             ;  -      -      -      15998

wms_goOuterLoop:
    nop                                ;  15994  15999
    nop                                ;  15995  16000
    nop                                ;  15996  16001
    nop                                ;  15997  16002
    nop                                ;  15998  16003
    goto    wms_outerLoop              ;  16000  16005

; ---- waitSeconds ----
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
