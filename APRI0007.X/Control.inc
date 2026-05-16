; ==========================================================
; Control.inc - Automatic shifting logic
; Processor: PIC18F47Q84 @ 64 MHz
;
; Implements the cadence-based control strategy, power-based
; target modulation, and the protections around shift requests.
; ==========================================================

TORQUE_MIN_ACTIVE             equ 1
CADENCE_MIN_ACTIVE            equ 5
TORQUE_HIGH_LIMIT             equ 65
STABILITY_THRESHOLD           equ 5
CADENCE_TARGET_MIN            equ 60
CADENCE_TARGET_MAX            equ 120
CADENCE_TARGET_DEFAULT        equ 70
CADENCE_TARGET_HIGH_OFFSET    equ 20
CADENCE_WINDOW_HALF           equ 15

; Base target selected by the user:
;   - low power  -> target = base target
;   - high power -> target = base target + 20 rpm
; Cadence window:
;   - upshift above target + 15 rpm
;   - downshift below target - 15 rpm
; Power-zone hysteresis:
;   - enter high power at 800 W
;   - leave high power at 600 W
POWER_ZONE_ENTER_HIGH_L       equ low(800)
POWER_ZONE_ENTER_HIGH_H       equ high(800)
POWER_ZONE_EXIT_HIGH_L        equ low(600)
POWER_ZONE_EXIT_HIGH_H        equ high(600)

CONTROL_LOOP_TICKS            equ 5      ; 5 x 20 ms = 100 ms between control evaluations
POST_SHIFT_TICKS              equ 75     ;75 x 20 ms = 1.5 s post-shift cooldown

PSECT udata_acs
high_power_counter:           DS 1
power_value_l:                DS 1
power_value_h:                DS 1
low_power_counter:            DS 1
power_enter_counter:          DS 1
power_exit_counter:           DS 1
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
cadence_target_low:           DS 1    ; active downshift threshold = target - 15
cadence_target_high:          DS 1    ; active upshift threshold = target + 15

PSECT code

; ---- Control_Init ----
Control_Init:
    clrf    power_zone_state, c
    movlw   CADENCE_TARGET_DEFAULT
    movwf   cadence_target_base, c
    call    ResetCounters
    return
 
ComputePower:
    ; Power estimate:
    ;   P ~= torque * cadence * 27 / 256
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
    call    _ComputePower_AddTmpToAcc

    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    call    _ComputePower_ShiftTmpLeft

    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    call    _ComputePower_ShiftTmpLeft
    call    _ComputePower_AddTmpToAcc

    ; Divide by 256 by dropping the low byte.
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
    ; Build the active cadence window.
    movf    cadence_target_active, w, c
    movwf   cadence_target_low, c
    movwf   cadence_target_high, c

    movlw   CADENCE_WINDOW_HALF
    subwf   cadence_target_low, f, c

    movlw   CADENCE_WINDOW_HALF
    addwf   cadence_target_high, f, c
    return
    
; ----------------------------------------------------------
; GearShiftControl
; One non-blocking control step.
; ----------------------------------------------------------
GearShiftControl:
    ; Wait until the control delay expires.
    movf    control_delay_ticks, f, c
    bz      _control_step_due
    return

_control_step_due:
    ; Require an active pedalling state.
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
    bra     _control_reload_normal_delay

_control_pedaling_active:

    ; Update the power zone with hysteresis and stability.
    btfsc   power_zone_state, 0, c
    bra     _control_hold_high_power_zone

    ; Low-power zone: request high power only after repeated confirmation.
    movlw   POWER_ZONE_ENTER_HIGH_H
    subwf   power_value_h, w, c
    bz      _control_check_enter_high_l
    bc      _control_power_enter_true
    bra     _control_power_enter_false

_control_check_enter_high_l:
    movlw   POWER_ZONE_ENTER_HIGH_L
    subwf   power_value_l, w, c
    bc      _control_power_enter_true
    bra     _control_power_enter_false

_control_power_enter_true:
    incf    power_enter_counter, f, c
    clrf    power_exit_counter, c
    movlw   STABILITY_THRESHOLD
    subwf   power_enter_counter, w, c
    bc      _control_use_high_power_zone
    bra     _control_keep_low_power_zone

_control_power_enter_false:
    clrf    power_enter_counter, c
    clrf    power_exit_counter, c
    bra     _control_keep_low_power_zone

_control_hold_high_power_zone:
    ; High-power zone: fall back only after repeated confirmation.
    movlw   POWER_ZONE_EXIT_HIGH_H
    subwf   power_value_h, w, c
    bz      _control_check_exit_high_l
    bc      _control_power_exit_false
    bra     _control_power_exit_true

_control_check_exit_high_l:
    movlw   POWER_ZONE_EXIT_HIGH_L
    subwf   power_value_l, w, c
    bc      _control_power_exit_false
    bra     _control_power_exit_true

_control_power_exit_true:
    incf    power_exit_counter, f, c
    clrf    power_enter_counter, c
    movlw   STABILITY_THRESHOLD
    subwf   power_exit_counter, w, c
    bc      _control_use_low_power_zone
    bra     _control_keep_high_power_zone

_control_power_exit_false:
    clrf    power_exit_counter, c
    clrf    power_enter_counter, c
    bra     _control_keep_high_power_zone

_control_keep_low_power_zone:
    clrf    power_exit_counter, c
    bcf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow
    bra     _control_check_cadence_high

_control_keep_high_power_zone:
    clrf    power_enter_counter, c
    bsf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    addlw   CADENCE_TARGET_HIGH_OFFSET
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow
    bra     _control_check_cadence_high

_control_use_low_power_zone:
    ; Confirm the low-power transition and toggle the zone LED.
    btfsc   power_zone_state, 0, c
    call    _control_blink_power_zone_led
    clrf    power_enter_counter, c
    clrf    power_exit_counter, c
    bcf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow
    bra     _control_check_cadence_high

_control_use_high_power_zone:
    ; Confirm the high-power transition and toggle the zone LED.
    btfss   power_zone_state, 0, c
    call    _control_blink_power_zone_led
    clrf    power_enter_counter, c
    clrf    power_exit_counter, c
    bsf     power_zone_state, 0, c
    movf    cadence_target_base, w, c
    addlw   CADENCE_TARGET_HIGH_OFFSET
    movwf   cadence_target_active, c
    call    _Control_PrepareCadenceWindow

_control_check_cadence_high:
    ; Above the upper threshold -> harder gear side.
    movf    cadence_target_high, w, c
    addlw   1
    subwf   cadence, w, c
    bc      _control_cadence_too_high
    bra     _control_cadence_not_high

_control_blink_power_zone_led:
    BANKSEL LATE
    btg     BANKMASK(LATE), 2, 1
    return

_control_cadence_not_high:
    clrf    high_power_counter, c
    bra     _control_check_cadence_low

_control_cadence_too_high:
    incf    high_power_counter, f, c
    clrf    low_power_counter, c
    bra     _control_check_upshift

_control_check_cadence_low:
    ; Below the lower threshold -> easier gear side.
    movf    cadence_target_low, w, c
    subwf   cadence, w, c
    bnc     _control_downshift_condition_true
    bra     _control_check_torque_high

_control_check_torque_high:
    ; High torque can also force the easier-gear side.
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
    ; Easier gear request.
    movlw   STABILITY_THRESHOLD
    subwf   low_power_counter, w, c
    bc      _control_execute_downshift
    bra     _control_check_upshift

_control_execute_downshift:
    call    Servo_IncPos
    call    ResetCounters
    movlw   POST_SHIFT_TICKS
    movwf   control_delay_ticks, c
    return

_control_check_upshift:
    ; Harder gear request.
    movlw   STABILITY_THRESHOLD
    subwf   high_power_counter, w, c
    bc      _control_execute_upshift
    bra     _control_reload_normal_delay

_control_execute_upshift:
    call    Servo_DecPos
    call    ResetCounters
    movlw   POST_SHIFT_TICKS
    movwf   control_delay_ticks, c
    return

_control_reload_normal_delay:
    movlw   CONTROL_LOOP_TICKS
    movwf   control_delay_ticks, c
    return

; ---- ResetCounters ----
ResetCounters:
    clrf    high_power_counter, c
    clrf    low_power_counter, c
    clrf    power_enter_counter, c
    clrf    power_exit_counter, c
    clrf    power_value_l, c
    clrf    power_value_h, c
    return
