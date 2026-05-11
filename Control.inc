TORQUE_MIN_ACTIVE             equ 1
CADENCE_MIN_ACTIVE            equ 5
TORQUE_HIGH_LIMIT             equ 90
STABILITY_THRESHOLD           equ 5
CADENCE_TARGET_MIN            equ 60
CADENCE_TARGET_MAX            equ 120
CADENCE_TARGET_DEFAULT        equ 70
CADENCE_TARGET_HIGH_OFFSET    equ 20
CADENCE_WINDOW_HALF           equ 30

; Two-zone cadence strategy inspired by the cadence/power literature:
; at higher rider power, target a higher cadence band.
; The rider selects a base cadence target. The controller then uses:
;   - low-power target  = base target
;   - high-power target = base target + 20 rpm
; Around that active target, the controller builds a symmetric window:
;   - upshift   above target + 30 rpm
;   - downshift below target - 30 rpm
; Power-zone hysteresis:
;   - enter the high-power cadence band at 300 W
;   - return to the low-power cadence band at 200 W
POWER_ZONE_ENTER_HIGH_L       equ low(300)
POWER_ZONE_ENTER_HIGH_H       equ high(300)
POWER_ZONE_EXIT_HIGH_L        equ low(200)
POWER_ZONE_EXIT_HIGH_H        equ high(200)

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
power_zone_state:             DS 1    ; bit 0 = 1 when the high-power cadence band is active
cadence_target_base:          DS 1    ; user-selected base cadence target in rpm
cadence_target_active:        DS 1    ; active target after power-zone adjustment
cadence_target_low:           DS 1    ; active downshift threshold = target - 35
cadence_target_high:          DS 1    ; active upshift threshold = target + 35

PSECT code

;----------------------------------------------------------------------
; Reset controller state at startup.
; The cadence target starts from a default value and can later be
; changed by the user from the button-driven setting screen.
;----------------------------------------------------------------------
Control_Init:
    clrf    power_zone_state, c
    movlw   CADENCE_TARGET_DEFAULT
    movwf   cadence_target_base, c
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

_Control_PrepareCadenceWindow:
    ; Build the active cadence window from the active target:
    ;   cadence_target_low  = target - window_half
    ;   cadence_target_high = target + window_half
    movf    cadence_target_active, w, c
    movwf   cadence_target_low, c
    movwf   cadence_target_high, c

    movlw   CADENCE_WINDOW_HALF
    subwf   cadence_target_low, f, c

    movlw   CADENCE_WINDOW_HALF
    addwf   cadence_target_high, f, c
    return
    
;----------------------------------------------------------------------
; GearShiftControl
; Main automatic shifting state machine:
;   1. Require valid pedaling activity.
;   2. Estimate rider power.
;   3. Select the low-power or high-power cadence target with hysteresis.
;   4. Build the cadence window around that target.
;   5. Shift up if cadence stays too high.
;   6. Shift down if cadence stays too low or torque gets too high.
;----------------------------------------------------------------------
GearShiftControl:
    ; Keep pedaling detection enabled. It is a simple, direct guard that
    ; prevents the power controller from shifting on stale or near-zero
    ; values when the rider is not actually pedaling.
    movlw   TORQUE_MIN_ACTIVE
    subwf   torque_nm, w, c
    bc      _control_check_min_cadence
    bra     _control_not_pedaling

_control_check_min_cadence:
    movlw   CADENCE_MIN_ACTIVE
    subwf   cadence, w, c
    bc      _control_pedaling_active

_control_not_pedaling:
    call    ResetCounters
    bra     _control_loop_delay

_control_pedaling_active:
    call    ComputePower

    ; Update the active power zone with hysteresis.
    ; The zone only switches up at 300 W and only switches back down
    ; once power falls to 200 W or less.
    btfsc   power_zone_state, 0, c
    bra     _control_hold_high_power_zone

    ; Currently in the low-power zone: only switch up if power >= 300 W.
    movlw   POWER_ZONE_ENTER_HIGH_H
    subwf   power_value_h, w, c
    bz      _control_check_enter_high_l
    bc      _control_use_high_power_zone
    bra     _control_use_low_power_zone

_control_check_enter_high_l:
    movlw   POWER_ZONE_ENTER_HIGH_L
    subwf   power_value_l, w, c
    bc      _control_use_high_power_zone
    bra     _control_use_low_power_zone

_control_hold_high_power_zone:
    ; Currently in the high-power zone: stay there until power <= 200 W.
    movlw   POWER_ZONE_EXIT_HIGH_H
    subwf   power_value_h, w, c
    bz      _control_check_exit_high_l
    bc      _control_use_high_power_zone
    bra     _control_use_low_power_zone

_control_check_exit_high_l:
    movlw   POWER_ZONE_EXIT_HIGH_L
    subwf   power_value_l, w, c
    bc      _control_use_high_power_zone
    bra     _control_use_low_power_zone

_control_use_low_power_zone:
    ; Low-power mode uses the user-selected base cadence target directly.
    btfsc   power_zone_state, 0, c
    call    _control_blink_power_zone_led
    bcf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow
    bra     _control_check_cadence_high

_control_use_high_power_zone:
    ; High-power mode shifts the active target upward by 20 rpm.
    btfss   power_zone_state, 0, c
    call    _control_blink_power_zone_led
    bsf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    addlw   CADENCE_TARGET_HIGH_OFFSET
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow

_control_check_cadence_high:
    ; Above the active upper threshold, request the harder-gear side.
    movf    cadence_target_high, w, c
    addlw   1
    subwf   cadence, w, c
    bc      _control_cadence_too_high
    bra     _control_cadence_not_high

_control_blink_power_zone_led:
    BANKSEL LATE
    bsf     BANKMASK(LATE), 2, 1
    movlw   100
    call    waitMilliSeconds
    bcf     BANKMASK(LATE), 2, 1
    return

_control_cadence_not_high:
    clrf    high_power_counter, c
    bra     _control_check_cadence_low

_control_cadence_too_high:
    incf    high_power_counter, f, c
    clrf    low_power_counter, c
    bra     _control_check_upshift

_control_check_cadence_low:
    ; Below the active lower threshold, request the easier-gear side.
    movf    cadence_target_low, w, c
    subwf   cadence, w, c
    bnc     _control_downshift_condition_true
    bra     _control_check_torque_high

_control_check_torque_high:
    ; High torque can also force the easier-gear side, even if cadence
    ; has not yet dropped below the lower threshold.
    movlw   TORQUE_HIGH_LIMIT
    subwf   torque_nm, w, c
    bc      _control_downshift_condition_true
    bra     _control_downshift_condition_false
    
_control_downshift_condition_true:
    incf    low_power_counter, f, c
    clrf    high_power_counter, c
    bra     _control_check_downshift
    
_control_downshift_condition_false:
    clrf    low_power_counter, c
    bra     _control_check_downshift
    
_control_check_downshift:
    ; Downshifts are requested by low cadence or high torque.
    movlw   STABILITY_THRESHOLD
    subwf   low_power_counter, w, c
    bc      _control_execute_downshift
    bra     _control_check_upshift

_control_execute_downshift:
    call    Servo_IncPos
    call    ResetCounters
    movlw   POST_SHIFT_DELAY
    call    waitSeconds
    return

_control_check_upshift:
    ; Upshifts are requested by high cadence only.
    movlw   STABILITY_THRESHOLD
    subwf   high_power_counter, w, c
    bc      _control_execute_upshift
    bra     _control_loop_delay

_control_execute_upshift:
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
