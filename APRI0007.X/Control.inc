TORQUE_MIN_ACTIVE:       equ 1            ; db_stable value from previous loop iteration
CADENCE_MIN_ACTIVE:       equ 1            ; scratch: bits that just transitioned released->pressed
upper_power_limit_adjusted:       equ 1
lower_power_limit_adjusted:	equ 1
TORQUE_HIGH_LIMIT:	equ 1
CADENCE_LOW_LIMIT:	equ 1
TORQUE_LOW_LIMIT:	equ 1
CADENCE_HIGH_LIMIT:	equ 1
STABILITY_THRESHOLD_HTLC:   equ 1
STABILITY_THRESHOLD_POWER:  equ 1

    
PSECT udata_acs
high_power_counter:    DS 1           
power_value:    DS 1          
low_power_counter:     DS 1  
HighTorqueLowCadence:	DS 1
LowTorqueHighCadence:	DS 1

PSECT code
 
ComputePower:
    ; Simplified: Power = (torque × cadence) / 100
    ; 16-bit × 16-bit multiplication
    movf    torque_h, w, c
    mulwf   cadence_value, c        ; torque_low × cadence_low
    movff   PRODL, power_value
    movff   PRODH, power_value+1
    
    ; Add cross terms (simplified - use proper 32-bit multiply in production)
    return
    
;----------------------------------------------------------------------
; GearShiftControl - Main state machine
;----------------------------------------------------------------------
GearShiftControl:
    
    ;Pedaling detection
    movf    torque_h, w, c
    subwf   TORQUE_MIN_ACTIVE, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    movf   cadence_h, w, c
    subwf   CADENCE_MIN_ACTIVE, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    bra     pedaling_active

pedaling_active:
    call    ComputePower
    
    ;Power conditions
    ;Compare power with upper limit
    BANKSEL power_value
    movf    power_value, w, c
    subwf   upper_power_limit_adjusted, w, c
    btfss   STATUS, 2, c
    bra     check_power_low
    ; Power > Upper limit
    incf    high_power_counter, f, c
    bra     power_high_done
    
check_power_low:
    clrf    high_power_counter, c
    
    ; Compare power with lower limit
    movf    power_value, w, c
    subwf   lower_power_limit_adjusted, w, c
    btfss   STATUS, 2, c
    bra     power_low_active
    clrf    low_power_counter, c
    bra     power_low_done

power_low_active:
    incf    low_power_counter, f, c

    ;Torque-cadence balance
    ; Check high torque, low cadence
    movf    torque_h, w, c
    subwf   TORQUE_HIGH_LIMIT, w, c
    btfss   STATUS, 4, c
    bra     GoodHTLCBalance
    
    movf    cadence_h, w, c
    subwf   CADENCE_LOW_LIMIT, w, c
    btfss   STATUS, 4, c
    clrf    HighTorqueLowCadence, c
GoodHTLCBalance:
    incf    HighTorqueLowCadence, f, c
    
    ; Check low torque, high cadence
    movf    torque_h, w, c
    subwf   TORQUE_LOW_LIMIT, w, c
    btfss   STATUS, 4, c
    bra     BADLTHCBalance
    
    movf    cadence_h, w, c
    subwf   CADENCE_HIGH_LIMIT, w, c
    btfss   STATUS, 4, c
    incf    LowTorqueHighCadence, c
BadLTHCBalance:
    clrf    LowTorqueHighCadence, f, c
    
    
    ; Shifting logic
    ; Check need for easier gear (downshift)
    movf    high_power_counter, w, c
    subwf   STABILITY_THRESHOLD_POWER, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    movf    HighTorqueLowCadence, w, c
    subwf   STABILITY_THRESHOLD_HTLC, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    call    Servo_DecPos
    bra     ResetCounters

    
    ; Check need for harder gear (upshift)
    movf    low_power_counter, w, c
    subwf   STABILITY_THRESHOLD_POWER, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    movf    LowTorqueHighCadence, w, c
    subwf   STABILITY_THRESHOLD_HTLC, w, c
    btfss   STATUS, 2, c
    bra     ResetCounters
    
    call    Servo_IncPos
    bra     ResetCounters
    

;----------------------------------------------------------------------
; ResetCounters - Resets all counters
;----------------------------------------------------------------------
ResetCounters:
    clrf    high_power_counter, c
    clrf    low_power_counter, c
    clrf    HighTorqueLowCadence, c
    clrf    LowTorqueHighCadence, c
    clrf    power_value, c
    ;delay
    movlw   200
    call    waitMilliSeconds
    return
    





