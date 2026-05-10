TORQUE_MIN_ACTIVE             equ 1
CADENCE_MIN_ACTIVE            equ 5
CADENCE_TARGET_LOW            equ 40
CADENCE_TARGET_HIGH           equ 80
TORQUE_HIGH_LIMIT             equ 55
STABILITY_THRESHOLD           equ 5


CONTROL_LOOP_PERIOD           equ 100
POST_SHIFT_DELAY             equ 3

PSECT udata_acs
high_power_counter:           DS 1
power_value_l:                DS 1
power_value_h:                DS 1
low_power_counter:            DS 1
HighTorqueLowCadence:         DS 1
LowTorqueHighCadence:         DS 1
power_raw_l:                  DS 1
power_raw_h:                  DS 1
power_acc_l:                  DS 1
power_acc_h:                  DS 1
power_acc_u:                  DS 1
power_tmp_l:                  DS 1
power_tmp_h:                  DS 1
power_tmp_u:                  DS 1

PSECT code

;----------------------------------------------------------------------
; Reset controller state at startup 
;----------------------------------------------------------------------
Control_Init:
    call    ResetCounters
    return
 
ComputePower:
    ; Mechanical power estimate:
    ;   P[W] = torque[Nm] * cadence[rpm] * (2*pi/60)
    ; Approximation used here:
    ;   P ~= torque * cadence * 27 / 256
    ; since 27/256 = 0.10547, very close to 2*pi/60 = 0.10472.
    movf    torque_nm, w, c
    mulwf   cadence, c
    movff   PRODL, power_raw_l
    movff   PRODH, power_raw_h

    clrf    power_acc_l, c
    clrf    power_acc_h, c
    clrf    power_acc_u, c

    movf    power_raw_l, w, c
    movwf   power_tmp_l, c
    movf    power_raw_h, w, c
    movwf   power_tmp_h, c
    clrf    power_tmp_u, c

    ; 27 = 1 + 2 + 8 + 16
    ; Add x
    call    _ComputePower_AddTmpToAcc

    ; Shift to 2x and add
    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    ; Shift to 4x (skip add)
    call    _ComputePower_ShiftTmpLeft

    ; Shift to 8x and add
    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    ; Shift to 16x and add
    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    ; Divide by 256 by dropping the low accumulator byte.
    movf    power_acc_h, w, c
    movwf   power_value_l, c
    movf    power_acc_u, w, c
    movwf   power_value_h, c
    return

_ComputePower_AddTmpToAcc:
    movf    power_tmp_l, w, c
    addwf   power_acc_l, f, c
    movf    power_tmp_h, w, c
    addwfc  power_acc_h, f, c
    movf    power_tmp_u, w, c
    addwfc  power_acc_u, f, c
    return

_ComputePower_ShiftTmpLeft:
    bcf     STATUS, 0, c
    rlcf    power_tmp_l, f, c
    rlcf    power_tmp_h, f, c
    rlcf    power_tmp_u, f, c
    return
    
;----------------------------------------------------------------------
; GearShiftControl - Main state machine
;----------------------------------------------------------------------
GearShiftControl:
    ; Keep pedaling detection enabled. It is a simple, direct guard that
    ; prevents the power controller from shifting on stale or near-zero
    ; values when the rider is not actually pedaling.
    movlw   TORQUE_MIN_ACTIVE
    subwf   torque_nm, w, c
    bc      _control_check_cadence
    bra     _control_inactive

_control_check_cadence:
    movlw   CADENCE_MIN_ACTIVE
    subwf   cadence, w, c
    bc      pedaling_active

_control_inactive:
    call    ResetCounters
    bra     _control_loop_delay

pedaling_active:
    call    ComputePower
    
    ; Cadence/torque split control:
    ; cadence too high -> shift up (harder gear)
    ; cadence too low  -> shift down (easier gear)
    ; torque too high  -> shift down (easier gear)
    movlw   CADENCE_TARGET_HIGH + 1
    subwf   cadence, w, c
    bc      _control_power_high_true
    bra     _control_power_high_false

_control_power_high_false:
    clrf    high_power_counter, c
    bra     _control_power_low_check

_control_power_high_true:
    incf    high_power_counter, f, c
    clrf    low_power_counter, c
    bra     _control_shift_up_check

_control_power_low_check:
    movlw   CADENCE_TARGET_LOW
    subwf   cadence, w, c
    bnc     _control_power_low_true
    bra     _control_torque_high_check

_control_torque_high_check:
    movlw   TORQUE_HIGH_LIMIT
    subwf   torque_nm, w, c
    bc      _control_power_low_true
    bra     _control_power_low_false
    
_control_power_low_true:
    incf    low_power_counter, f, c
    clrf    high_power_counter, c
    bra     _control_shift_down_check
    
_control_power_low_false:
    clrf    low_power_counter, c
    bra     _control_shift_down_check

    ; ------------------------------------------------------------------
    ; Power-only test mode:
    ; HTLC / LTHC torque-cadence balance logic intentionally disabled.
    ; Re-enable these blocks when you want to combine them with the power
    ; controller again.
    ; ------------------------------------------------------------------
;_control_htlc_check:
;    movlw   TORQUE_HIGH_LIMIT
;    subwf   torque_nm, w, c
;    bc      _control_htlc_check_cadence
;    clrf    HighTorqueLowCadence, c
;    bra     _control_lthc_check
;
;_control_htlc_check_cadence:
;    movlw   CADENCE_LOW_LIMIT + 1
;    subwf   cadence, w, c
;    bnc     _control_htlc_true
;    clrf    HighTorqueLowCadence, c
;    bra     _control_lthc_check
;
;_control_htlc_true:
;    incf    HighTorqueLowCadence, f, c
;
;_control_lthc_check:
;    movlw   TORQUE_LOW_LIMIT + 1
;    subwf   torque_nm, w, c
;    bnc     _control_lthc_check_cadence
;    clrf    LowTorqueHighCadence, c
;    bra     _control_shift_down_check
;
;_control_lthc_check_cadence:
;    movlw   CADENCE_HIGH_LIMIT
;    subwf   cadence, w, c
;    bc      _control_lthc_true
;    clrf    LowTorqueHighCadence, c
;    bra     _control_shift_down_check
;
;_control_lthc_true:
;    incf    LowTorqueHighCadence, f, c

_control_shift_down_check:
    ; Downshifts are requested by low cadence or high torque.
    movlw   STABILITY_THRESHOLD
    subwf   low_power_counter, w, c
    bc      _control_shift_down_execute
    bra     _control_shift_up_check

_control_shift_down_execute:
    call    Servo_IncPos
    call    ResetCounters
    movlw   POST_SHIFT_DELAY
    call    waitSeconds
    return

_control_shift_up_check:
    ; Upshifts are requested by high cadence only.
    movlw   STABILITY_THRESHOLD
    subwf   high_power_counter, w, c
    bc      _control_shift_up_execute
    bra     _control_loop_delay

_control_shift_up_execute:
    call    Servo_DecPos
    call    ResetCounters
    movlw   POST_SHIFT_DELAY
    call    waitSeconds
    return

_control_loop_delay:
    movlw   CONTROL_LOOP_PERIOD
    call    waitMilliSeconds
    return

;----------------------------------------------------------------------
; ResetCounters - Resets all counters
;----------------------------------------------------------------------
ResetCounters:
    clrf    high_power_counter, c
    clrf    low_power_counter, c
    clrf    HighTorqueLowCadence, c
    clrf    LowTorqueHighCadence, c
    clrf    power_value_l, c
    clrf    power_value_h, c
    return
