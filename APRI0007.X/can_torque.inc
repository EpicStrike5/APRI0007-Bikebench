; ==========================================================
; can_torque.inc - CAN torque and cadence acquisition
; Processor: PIC18F47Q84 @ 64 MHz
;
; Configures CAN communication, reads the Bafang sensor frame,
; extracts cadence, converts torque, and publishes a filtered
; torque value for the rest of the firmware.
; ==========================================================

; ---- Configuration constants ----
CAN_PPS_CANRX_RB3   equ 0x0B        ; CANRXPPS = RB1
CAN_PPS_CANTX       equ 0x46        ; RBxPPS output function for CANTX

; CAN operation modes
CAN_MODE_NORMAL_2_0 equ 0x06        ; CAN 2.0 Normal Operation
CAN_MODE_CONFIG     equ 0x04        ; Configuration Mode
CAN_MODE_DISABLE    equ 0x01        ; Disable Mode

; FIFO message RAM base
CAN_FIFO_BASE_L     equ 0x00        ; Low byte of 0x3800
CAN_FIFO_BASE_H     equ 0x38        ; High byte of 0x3800

; 500 kbps nominal bit rate at 64 MHz
CAN_NBTCFGL_VAL     equ 0x17        ; SJW = 23 (actual 24 Tq)
CAN_NBTCFGH_VAL     equ 0x17        ; TSEG2 = 23 (actual 24 Tq)
CAN_NBTCFGU_VAL     equ 0x26        ; TSEG1 = 38 (actual 39 Tq)
CAN_NBTCFGT_VAL     equ 0x01        ; BRP = 1 (prescaler = 2)

; ---- Variables ----
PSECT udata_acs
can_rx_buffer:      DS 8            ; Receive data buffer (8 bytes max for CAN 2.0)
can_msg_id_l:       DS 1            ; Received message ID (low byte)
can_msg_id_h:       DS 1            ; Received message ID (high byte) - only 3 bits used
can_dlc:            DS 1            ; Data Length Code (0-8)
can_temp:           DS 1            ; Temporary variable
can_fifo_ptr_l:     DS 1            ; FIFO pointer low byte
can_fifo_ptr_h:     DS 1            ; FIFO pointer high byte

; Latest decoded sensor fields from the accepted 0x67D frame.
torque_h:	    DS 1
torque_l:	    DS 1
cadence:	    DS 1
; ---- Code ----
PSECT code

; ---- CAN_Init ----
 
CAN_Init:
    ; Enable CAN.
    BANKSEL C1CONH
    bsf     BANKMASK(C1CONH), 7, 1      ; C1CONHbits.ON = 1

    ; Enter configuration mode.
    BANKSEL C1CONT
    movlw   CAN_MODE_CONFIG
    movwf   BANKMASK(C1CONT), 1         ; REQOP = Configuration Mode

    ; Wait for configuration mode.
_can_init_wait_config:
    BANKSEL C1CONU
    movf    BANKMASK(C1CONU), 0, 1     ; Read OPMOD into W
    andlw   0xE0                        ; Mask OPMOD bits (5-7)
    xorlw   0x80                        ; Compare with Config mode (0x04 << 5)
    bnz     _can_init_wait_config

    ; Configure FIFO base address.
    BANKSEL C1FIFOBAL
    movlw   CAN_FIFO_BASE_L
    movwf   BANKMASK(C1FIFOBAL), 1
    movlw   CAN_FIFO_BASE_H
    movwf   BANKMASK(C1FIFOBAH), 1

    ; Configure control registers.
    ; C1CONH: WAKFIL=1, WFT=11 (T11 filter), ON=1
    BANKSEL C1CONH
    movlw   0x87
    movwf   BANKMASK(C1CONH), 1
    ; C1CONL: DNCNT=0, CLKSEL0=0 (use Fosc)
    BANKSEL C1CONL
    clrf    BANKMASK(C1CONL), 1
    ; C1CONU: TXQEN=1 (enable TX queue), STEF=0
    BANKSEL C1CONU
    movlw   0x90
    movwf   BANKMASK(C1CONU), 1

    ; Configure bit rate.
    BANKSEL C1NBTCFGL
    movlw   CAN_NBTCFGL_VAL             ; SJW = 23
    movwf   BANKMASK(C1NBTCFGL), 1
    movlw   CAN_NBTCFGH_VAL             ; TSEG2 = 23
    movwf   BANKMASK(C1NBTCFGH), 1
    movlw   CAN_NBTCFGU_VAL             ; TSEG1 = 38
    movwf   BANKMASK(C1NBTCFGU), 1
    movlw   CAN_NBTCFGT_VAL             ; BRP = 1
    movwf   BANKMASK(C1NBTCFGT), 1

    ; Configure TX queue.
    ; C1TXQCONL: disabled interrupts
    BANKSEL C1TXQCONL
    movlw   0x80
    movwf   BANKMASK(C1TXQCONL), 1
    ; C1TXQCONH: FRESET=1 (reset FIFO)
    movlw   0x04
    movwf   BANKMASK(C1TXQCONH), 1
    ; C1TXQCONU: TXPRI=0, TXAT=11 (unlimited retransmit)
    movlw   0x60
    movwf   BANKMASK(C1TXQCONU), 1
    ; C1TXQCONT: FSIZE=5 (6 messages)
    movlw   0x05
    movwf   BANKMASK(C1TXQCONT), 1

    ; Configure RX FIFO1.
    ; C1FIFOCON1L: TFNRFNIE=1 (RX not empty interrupt enable)
    BANKSEL C1FIFOCON1L
    movlw   0x01
    movwf   BANKMASK(C1FIFOCON1L), 1
    ; C1FIFOCON1H: FRESET=1 (reset FIFO)
    movlw   0x04
    movwf   BANKMASK(C1FIFOCON1H), 1
    ; C1FIFOCON1U: TXAT=0, TXPRI=0 (RX mode)
    clrf    BANKMASK(C1FIFOCON1U), 1
    ; C1FIFOCON1T: FSIZE=5 (6 messages), PLSIZE=0 (8 bytes)
    movlw   0x05
    movwf   BANKMASK(C1FIFOCON1T), 1

    ; Configure filter 0 to accept all standard messages.
    BANKSEL C1FLTCON0L
    movlw   0x01                        ; F0BP = 1 (FIFO1)
    movwf   BANKMASK(C1FLTCON0L), 1

    ; Accept any SID.
    BANKSEL C1FLTOBJ0L
    clrf    BANKMASK(C1FLTOBJ0L), 1     ; SID[7:0] = 0
    clrf    BANKMASK(C1FLTOBJ0H), 1     ; SID[10:8] = 0
    clrf    BANKMASK(C1FLTOBJ0U), 1     ; EID = 0
    clrf    BANKMASK(C1FLTOBJ0T), 1     ; EXIDE=0 (standard ID only)

    ; Mask 0: accept all IDs.
    BANKSEL C1MASK0L
    clrf    BANKMASK(C1MASK0L), 1       ; MSID[7:0] = 0 (don't care)
    clrf    BANKMASK(C1MASK0H), 1       ; MSID[10:8] = 0 (don't care)
    clrf    BANKMASK(C1MASK0U), 1       ; MEID = 0
    clrf    BANKMASK(C1MASK0T), 1       ; accept both standard and extended IDs

    ; Enable filter 0.
    BANKSEL C1FLTCON0L
    bsf     BANKMASK(C1FLTCON0L), 7, 1  ; FLTEN0 = 1

    ; Switch to normal CAN 2.0 mode.
    BANKSEL C1CONT
    movlw   CAN_MODE_NORMAL_2_0
    movwf   BANKMASK(C1CONT), 1         ; REQOP = Normal 2.0 Mode

    ; Wait for normal mode.
_can_init_wait_normal:
    BANKSEL C1CONU
    movf    BANKMASK(C1CONU), 0, 1     ; Read OPMOD into W
    andlw   0xE0                        ; Mask OPMOD bits
    xorlw   0xC0                        ; Compare with Normal 2.0 (0x06 << 5)
    bnz     _can_init_wait_normal

    return

; ---- CAN_IsRxReady ----
CAN_IsRxReady:
    BANKSEL C1FIFOSTA1L
    movf    BANKMASK(C1FIFOSTA1L), 0, 1 ; read FIFO status byte
    andlw   0x01                        ; keep TFNRFNIF only, sets Z correctly
    return

; ---- CAN_Receive ----
CAN_Receive:
    ; Get the current FIFO1 message address.
    BANKSEL C1FIFOUA1L
    movf    BANKMASK(C1FIFOUA1L), 0, 1
    movwf   can_fifo_ptr_l, c
    movf    BANKMASK(C1FIFOUA1H), 0, 1
    movwf   can_fifo_ptr_h, c

    ; Set up FSR0.
    movf    can_fifo_ptr_l, w, c
    movwf   FSR0L, c
    movf    can_fifo_ptr_h, w, c
    movwf   FSR0H, c

    ; Read the message header.
    movf    POSTINC0, w, c              ; byte 0: SID low
    movwf   can_msg_id_l, c

    movf    POSTINC0, w, c              ; byte 1: SID high bits
    andlw   0x07
    movwf   can_msg_id_h, c

    movf    POSTINC0, w, c              ; skip byte 2
    movf    POSTINC0, w, c              ; skip byte 3

    movf    POSTINC0, w, c              ; byte 4: DLC / flags
    andlw   0x0F
    movwf   can_dlc, c

    ; Clamp DLC to 8 bytes max.
    movlw   8
    cpfsgt  can_dlc, c
    bra     _can_dlc_ok
    movlw   8
    movwf   can_dlc, c
_can_dlc_ok:

    movf    POSTINC0, w, c              ; skip byte 5
    movf    POSTINC0, w, c              ; skip byte 6
    movf    POSTINC0, w, c              ; skip byte 7

    ; Clear the software buffer first.
    clrf    can_rx_buffer+0, c
    clrf    can_rx_buffer+1, c
    clrf    can_rx_buffer+2, c
    clrf    can_rx_buffer+3, c
    clrf    can_rx_buffer+4, c
    clrf    can_rx_buffer+5, c
    clrf    can_rx_buffer+6, c
    clrf    can_rx_buffer+7, c

    ; Copy only the valid data bytes.
    lfsr    1, can_rx_buffer
    movf    can_dlc, w, c
    movwf   can_temp, c

_can_rx_copy_loop:
    movf    can_temp, f, c
    bz      _can_rx_done
    movf    POSTINC0, w, c
    movwf   POSTINC1, c
    decf    can_temp, f, c
    bra     _can_rx_copy_loop

_can_rx_done:
    ; Acknowledge the FIFO read.
    BANKSEL C1FIFOCON1H
    bsf     BANKMASK(C1FIFOCON1H), 0, 1 ; UINC = 1

    return


; ---- Torque processing ----
PSECT udata_acs
    torque_nm:           DS 1            ; filtered/displayed torque in whole Nm
    torque_sample:       DS 1            ; latest instantaneous torque sample in whole Nm
    torque_calc_l:       DS 1            ; generic torque scratch low byte
    torque_calc_h:       DS 1            ; generic torque scratch high byte
    torque_sum_l:        DS 1            ; low byte of the 128-sample running sum
    torque_sum_h:        DS 1            ; high byte of the 128-sample running sum
    torque_buf_idx:      DS 1            ; circular buffer index (0..127)
    torque_oldest:       DS 1            ; sample being replaced in the circular buffer
    torque_clear_count:  DS 1            ; startup loop counter used to clear the buffer

PSECT udata_can_torque, class=UDATA
    torque_history:      DS 128          ; last 128 torque samples

PSECT code

TORQUE_SAMPLE_MIN_VALID   equ 5
TORQUE_SAMPLE_MAX_VALID   equ 80

; ---- CAN_Torque_Init ----
CAN_Torque_Init:
    ; Clear decoded values and the published torque.
    clrf    torque_l, c
    clrf    torque_h, c
    clrf    cadence, c
    clrf    torque_nm, c
    clrf    torque_sample, c

    ; Reset moving-average state.
    clrf    torque_sum_l, c
    clrf    torque_sum_h, c
    clrf    torque_buf_idx, c
    clrf    torque_oldest, c

    ; Clear the history buffer.
    lfsr    0, torque_history
    movlw   128
    movwf   torque_clear_count, c

_can_torque_clear_history:
    clrf    POSTINC0, c
    decfsz  torque_clear_count, f, c
    bra     _can_torque_clear_history
    return

; ---- CAN_ServiceTorqueSensor ----
CAN_ServiceTorqueSensor:
_can_torque_scan:
    ; Stop if FIFO1 is empty.
    call    CAN_IsRxReady
    bz      _can_torque_done

    ; Read one frame.
    call    CAN_Receive

    ; Keep scanning until frame ID 0x67D is found.
    movf    can_msg_id_h, w, c
    xorlw   0x06
    bnz     _can_torque_scan
    movf    can_msg_id_l, w, c
    xorlw   0x7D
    bnz     _can_torque_scan

    ; Extract torque and cadence fields.
    movf    can_rx_buffer+3, w, c
    movwf   torque_l, c
    movf    can_rx_buffer+4, w, c
    movwf   torque_h, c
    movf    can_rx_buffer+5, w, c
    movwf   cadence, c

    ; Convert the raw torque word into a whole-Nm sample.
    call    _CAN_ComputeTorqueNm

    ; Treat very small values as zero.
    movlw   TORQUE_SAMPLE_MIN_VALID
    subwf   torque_sample, w, c
    bc      _can_torque_check_max
    clrf    torque_sample, c
    bra     _can_torque_accept_sample

_can_torque_check_max:
    ; Reject implausibly large samples.
    movlw   TORQUE_SAMPLE_MAX_VALID + 1
    subwf   torque_sample, w, c
    bc      _can_torque_done

_can_torque_accept_sample:
    ; Update the circular moving average.
    lfsr    0, torque_history
    movf    torque_buf_idx, w, c
    addwf   FSR0L, f, c
    movlw   0x00
    addwfc  FSR0H, f, c

    movf    INDF0, w, c
    movwf   torque_oldest, c

    ; Remove the oldest sample.
    movf    torque_oldest, w, c
    subwf   torque_sum_l, f, c
    movlw   0x00
    subwfb  torque_sum_h, f, c

    ; Store the new sample.
    movf    torque_sample, w, c
    movwf   INDF0, c

    ; Add the new sample.
    movf    torque_sample, w, c
    addwf   torque_sum_l, f, c
    movlw   0x00
    addwfc  torque_sum_h, f, c

    ; Advance and wrap the circular index.
    incf    torque_buf_idx, f, c
    movlw   128
    cpfseq  torque_buf_idx, c
    bra     _can_torque_average
    clrf    torque_buf_idx, c

_can_torque_average:
    ; Publish sum / 128 with seven right shifts.
    movf    torque_sum_l, w, c
    movwf   torque_calc_l, c
    movf    torque_sum_h, w, c
    movwf   torque_calc_h, c

    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c
    bcf     STATUS, 0, c
    rrcf    torque_calc_h, f, c
    rrcf    torque_calc_l, f, c

    movf    torque_calc_l, w, c
    movwf   torque_nm, c

_can_torque_done:
    return

; ---- _CAN_ComputeTorqueNm ----
; torque = 8 * (raw - 750) / 245
_CAN_ComputeTorqueNm:
    ; Copy the raw little-endian torque word into scratch registers.
    movf    torque_l, w, c
    movwf   torque_calc_l, c
    movf    torque_h, w, c
    movwf   torque_calc_h, c

    ; Subtract the 750 mV baseline.
    movlw   0xEE
    subwf   torque_calc_l, f, c
    movlw   0x02
    subwfb  torque_calc_h, f, c
    bc      _can_torque_positive
    clrf    torque_sample, c
    return

_can_torque_positive:
    ; Multiply by 8 with three left shifts.
    bcf     STATUS, 0, c
    rlcf    torque_calc_l, f, c
    rlcf    torque_calc_h, f, c
    bcf     STATUS, 0, c
    rlcf    torque_calc_l, f, c
    rlcf    torque_calc_h, f, c
    bcf     STATUS, 0, c
    rlcf    torque_calc_l, f, c
    rlcf    torque_calc_h, f, c

    clrf    torque_sample, c

_can_torque_div_loop:
    ; Divide by 245 with repeated subtraction.
    movf    torque_calc_h, w, c
    bnz     _can_torque_do_sub
    movlw   0xF5
    subwf   torque_calc_l, w, c
    bnc     _can_torque_div_done

_can_torque_do_sub:
    movlw   0xF5
    subwf   torque_calc_l, f, c
    movlw   0x00
    subwfb  torque_calc_h, f, c
    incf    torque_sample, f, c
    bra     _can_torque_div_loop

_can_torque_div_done:
    return
