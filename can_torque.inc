;======================================================================
; can_torque.inc - CAN Bus Interface for BAFANG PA261 Torque Sensor
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Hardware: CAN module uses hardware CAN FD peripheral in CAN 2.0 mode
;           RB4 = CANTX (output to CAN transceiver)
;           RB3 = CANRX (input from CAN transceiver)
;
; CAN Configuration:
;   - 500 Kbps nominal bit rate (standard for e-bike sensors)
;   - CAN 2.0 mode (not CAN FD)
;   - FIFO1 configured as RX FIFO (8 byte payload)
;   - Accept all standard ID messages (filter disabled)
;
; BAFANG PA261 Sensor:
;   - Torque range: 0-80 N.m mapped to 0.75-3.2V (analog)
;   - 32 speed pulses per revolution (separate digital signal)
;   - CAN interface pins: CAN_H (green), CAN_L (white)
;
; Provides:
;   CAN_Init           - Initialize CAN module (500 Kbps, RX FIFO)
;   CAN_IsRxReady      - Check if message received (Z flag clear = ready)
;   CAN_Receive        - Read received message into buffer
;
; Data access: after CAN_Receive, read data bytes directly:
;   movf  can_rx_buffer+0, w, c    ; data byte 0
;   movf  can_rx_buffer+N, w, c    ; data byte N (0-7)
;
; Usage: #include "can_torque.inc" from your main source file
;        (main must already have #include <xc.inc>)
;        Requires: pinconfig.inc (pin I/O + PPS done externally)
;
; Note: The BAFANG sensor CAN message format is not publicly documented.
;       You may need to use a CAN bus analyzer to discover the message IDs
;       and data format used by your specific sensor.
;======================================================================

; ---- Configuration Constants ----
; PPS codes for CAN module
CAN_PPS_CANRX_RB3   equ 0x0B        ; CANRXPPS = RB1
CAN_PPS_CANTX       equ 0x46        ; RBxPPS output function for CANTX

; CAN Operation Modes (for C1CONT REQOP field)
CAN_MODE_NORMAL_2_0 equ 0x06        ; CAN 2.0 Normal Operation
CAN_MODE_CONFIG     equ 0x04        ; Configuration Mode
CAN_MODE_DISABLE    equ 0x01        ; Disable Mode

; FIFO Message Object RAM base address (0x3800 in device RAM)
CAN_FIFO_BASE_L     equ 0x00        ; Low byte of 0x3800
CAN_FIFO_BASE_H     equ 0x38        ; High byte of 0x3800

; Bit Rate Configuration for 500 Kbps at 64 MHz Fosc
; Formula: BitRate = FCAN / ((BRP+1) * (Sync + TSEG1+1 + TSEG2+1))
;   FCAN = 64 MHz, BRP = 1 -> prescaler = 2 -> Tq = 31.25 ns
;   Sync=1, TSEG1=38 (39 Tq), TSEG2=23 (24 Tq) -> total 64 Tq
;   64 MHz / (2 * 64) = 500 Kbps  |  Sample point = 40/64 = 62.5%
;   (Values derived from Microchip MCC for PIC18-Q83 CAN 2.0 example)
;
; Register byte layout (C1NBTCFG 32-bit register):
;   C1NBTCFGL = SJW    |  C1NBTCFGH = TSEG2
;   C1NBTCFGU = TSEG1  |  C1NBTCFGT = BRP
CAN_NBTCFGL_VAL     equ 0x17        ; SJW = 23 (actual 24 Tq)
CAN_NBTCFGH_VAL     equ 0x17        ; TSEG2 = 23 (actual 24 Tq)
CAN_NBTCFGU_VAL     equ 0x26        ; TSEG1 = 38 (actual 39 Tq)
CAN_NBTCFGT_VAL     equ 0x01        ; BRP = 1 (prescaler = 2)

; ---- Variables (Access Bank - auto allocated by linker) ----
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

;----------------------------------------------------------------------
; CAN_Init
; Initialize CAN module for 500 Kbps CAN 2.0 operation.
; Configures RX FIFO1 to accept all standard-ID messages.
;
; Pin I/O and PPS routing for CAN TX/RX must be configured in
; pinconfig.inc BEFORE calling this routine (uncomment the CAN PPS
; block and adjust TRIS/WPU/LAT for RB0 output + RB1 input).
;----------------------------------------------------------------------
 
CAN_Init:
    ; === Enable CAN Module ===
    BANKSEL C1CONH
    bsf     BANKMASK(C1CONH), 7, 1      ; C1CONHbits.ON = 1

    ; === Request Configuration Mode ===
    ; C1CONT = 0x04 (ABAT=0, TXBWS=0)
    BANKSEL C1CONT
    movlw   CAN_MODE_CONFIG
    movwf   BANKMASK(C1CONT), 1         ; REQOP = Configuration Mode

    ; Wait for Configuration Mode
_can_init_wait_config:
    BANKSEL C1CONU
    movf    BANKMASK(C1CONU), 0, 1     ; Read OPMOD into W
    andlw   0xE0                        ; Mask OPMOD bits (5-7)
    xorlw   0x80                        ; Compare with Config mode (0x04 << 5)
    bnz     _can_init_wait_config

    ; === Configure FIFO Base Address ===
    ; Point to dedicated CAN RAM area at 0x3800
    BANKSEL C1FIFOBAL
    movlw   CAN_FIFO_BASE_L
    movwf   BANKMASK(C1FIFOBAL), 1
    movlw   CAN_FIFO_BASE_H
    movwf   BANKMASK(C1FIFOBAH), 1

    ; === Configure Control Registers ===
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

    ; === Configure Bit Rate (500 Kbps) ===
    BANKSEL C1NBTCFGL
    movlw   CAN_NBTCFGL_VAL             ; SJW = 23
    movwf   BANKMASK(C1NBTCFGL), 1
    movlw   CAN_NBTCFGH_VAL             ; TSEG2 = 23
    movwf   BANKMASK(C1NBTCFGH), 1
    movlw   CAN_NBTCFGU_VAL             ; TSEG1 = 38
    movwf   BANKMASK(C1NBTCFGU), 1
    movlw   CAN_NBTCFGT_VAL             ; BRP = 1
    movwf   BANKMASK(C1NBTCFGT), 1

    ; === Configure TX Queue (for future use) ===
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

    ; === Configure RX FIFO1 ===
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

    ; === Configure Filter 0 to Accept All Standard Messages ===
    ; Route to FIFO1
    BANKSEL C1FLTCON0L
    movlw   0x01                        ; F0BP = 1 (FIFO1)
    movwf   BANKMASK(C1FLTCON0L), 1

    ; Filter Object 0: Accept any SID (set to 0x000)
    BANKSEL C1FLTOBJ0L
    clrf    BANKMASK(C1FLTOBJ0L), 1     ; SID[7:0] = 0
    clrf    BANKMASK(C1FLTOBJ0H), 1     ; SID[10:8] = 0
    clrf    BANKMASK(C1FLTOBJ0U), 1     ; EID = 0
    clrf    BANKMASK(C1FLTOBJ0T), 1     ; EXIDE=0 (standard ID only)

    ; Mask 0: Don't care about any SID bits (accept all)
    BANKSEL C1MASK0L
    clrf    BANKMASK(C1MASK0L), 1       ; MSID[7:0] = 0 (don't care)
    clrf    BANKMASK(C1MASK0H), 1       ; MSID[10:8] = 0 (don't care)
    clrf    BANKMASK(C1MASK0U), 1       ; MEID = 0
    clrf    BANKMASK(C1MASK0T), 1       ; accept both standard and extended IDs for debug

    ; Enable Filter 0
    BANKSEL C1FLTCON0L
    bsf     BANKMASK(C1FLTCON0L), 7, 1  ; FLTEN0 = 1

    ; === Switch to Normal CAN 2.0 Mode ===
    BANKSEL C1CONT
    movlw   CAN_MODE_NORMAL_2_0
    movwf   BANKMASK(C1CONT), 1         ; REQOP = Normal 2.0 Mode

    ; Wait for Normal Mode
_can_init_wait_normal:
    BANKSEL C1CONU
    movf    BANKMASK(C1CONU), 0, 1     ; Read OPMOD into W
    andlw   0xE0                        ; Mask OPMOD bits
    xorlw   0xC0                        ; Compare with Normal 2.0 (0x06 << 5)
    bnz     _can_init_wait_normal

    return

;----------------------------------------------------------------------
; CAN_IsRxReady
; Check if a message has been received in FIFO1.
; Output: Z flag clear (NZ) if message ready, Z flag set if empty
; Clobbers: W
;----------------------------------------------------------------------
CAN_IsRxReady:
    BANKSEL C1FIFOSTA1L
    movf    BANKMASK(C1FIFOSTA1L), 0, 1 ; read FIFO status byte
    andlw   0x01                        ; keep TFNRFNIF only, sets Z correctly
    return

;----------------------------------------------------------------------
; CAN_Receive
; Read received CAN message from FIFO1 into buffer.
; Extracts message ID and DLC for easy access.
; Call CAN_IsRxReady first to ensure message is available.
;----------------------------------------------------------------------
CAN_Receive:
    ; Get FIFO1 User Address (pointer to current message in CAN RAM).
    ; On this device C1FIFOUA1 already points to the active object,
    ; so do not add the FIFO base address again.
    BANKSEL C1FIFOUA1L
    movf    BANKMASK(C1FIFOUA1L), 0, 1
    movwf   can_fifo_ptr_l, c
    movf    BANKMASK(C1FIFOUA1H), 0, 1
    movwf   can_fifo_ptr_h, c

    ; Setup indirect addressing using FSR0
    movf    can_fifo_ptr_l, w, c
    movwf   FSR0L, c
    movf    can_fifo_ptr_h, w, c
    movwf   FSR0H, c

    ; Read message header
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

    ; Clamp DLC to 8 bytes max so the software copy loop never runs
    ; past can_rx_buffer, even if a malformed value is seen.
    movlw   8
    cpfsgt  can_dlc, c
    bra     _can_dlc_ok
    movlw   8
    movwf   can_dlc, c
_can_dlc_ok:

    movf    POSTINC0, w, c              ; skip byte 5
    movf    POSTINC0, w, c              ; skip byte 6
    movf    POSTINC0, w, c              ; skip byte 7

    ; Clear software buffer first so short frames do not leave stale bytes
    clrf    can_rx_buffer+0, c
    clrf    can_rx_buffer+1, c
    clrf    can_rx_buffer+2, c
    clrf    can_rx_buffer+3, c
    clrf    can_rx_buffer+4, c
    clrf    can_rx_buffer+5, c
    clrf    can_rx_buffer+6, c
    clrf    can_rx_buffer+7, c

    ; Copy only DLC bytes into can_rx_buffer
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
    ; Acknowledge FIFO read
    BANKSEL C1FIFOCON1H
    bsf     BANKMASK(C1FIFOCON1H), 0, 1 ; UINC = 1

    return


;----------------------------------------------------------------------
; Added torque processing helpers
;----------------------------------------------------------------------
PSECT udata_acs
    torque_nm:           DS 1            ; filtered/displayed torque in whole Nm
    torque_sample:       DS 1            ; latest instantaneous torque sample in whole Nm
    torque_calc_l:       DS 1            ; generic torque scratch low byte
    torque_calc_h:       DS 1            ; generic torque scratch high byte
    torque_sum_l:        DS 1            ; low byte of the 128-sample running sum
    torque_sum_h:        DS 1            ; high byte of the 128-sample running sum
    torque_buf_idx:      DS 1            ; circular buffer write index (0..127)
    torque_oldest:       DS 1            ; sample being replaced in the circular buffer
    torque_clear_count:  DS 1            ; startup loop counter used to clear the buffer

PSECT udata_can_torque, class=UDATA
    ; Circular buffer that holds the latest 128 instantaneous torque samples.
    torque_history:      DS 128          ; last 128 torque samples (zero-prefilled)

PSECT code

;----------------------------------------------------------------------
; CAN_Torque_Init
; Clears all torque/cadence state used by the display path.
;----------------------------------------------------------------------
CAN_Torque_Init:
    ; Clear the latest decoded frame fields and the published filtered value.
    clrf    torque_l, c
    clrf    torque_h, c
    clrf    cadence, c
    clrf    torque_nm, c
    clrf    torque_sample, c

    ; Reset moving-average bookkeeping.
    clrf    torque_sum_l, c
    clrf    torque_sum_h, c
    clrf    torque_buf_idx, c
    clrf    torque_oldest, c

    ; Prefill the circular buffer with zeroes so the first published
    ; values ramp in smoothly from a known state.
    lfsr    0, torque_history
    movlw   128
    movwf   torque_clear_count, c

_can_torque_clear_history:
    clrf    POSTINC0, c
    decfsz  torque_clear_count, f, c
    bra     _can_torque_clear_history
    return

;----------------------------------------------------------------------
; CAN_ServiceTorqueSensor
; Scans received CAN frames until FIFO is empty or ID 0x67D is found.
; When ID 0x67D is found, extracts torque/cadence, converts torque to
; whole Nm, updates a 128-sample circular buffer, and publishes the
; moving average over the latest 128 valid sensor frames.
;----------------------------------------------------------------------
CAN_ServiceTorqueSensor:
_can_torque_scan:
    ; Exit immediately if FIFO1 is empty.
    call    CAN_IsRxReady
    bz      _can_torque_done

    ; Pull one raw frame out of FIFO1.
    call    CAN_Receive

    ; Software filter: keep scanning until the Bafang torque frame
    ; with standard ID 0x67D is found.
    movf    can_msg_id_h, w, c
    xorlw   0x06
    bnz     _can_torque_scan
    movf    can_msg_id_l, w, c
    xorlw   0x7D
    bnz     _can_torque_scan

    ; Bafang PA261 layout for ID 0x67D:
    ;   data[3] = torque low byte
    ;   data[4] = torque high byte
    ;   data[5] = cadence / RPM
    movf    can_rx_buffer+3, w, c
    movwf   torque_l, c
    movf    can_rx_buffer+4, w, c
    movwf   torque_h, c
    movf    can_rx_buffer+5, w, c
    movwf   cadence, c

    ; Convert the raw 16-bit sensor value into an instantaneous
    ; integer torque sample in Nm.
    call    _CAN_ComputeTorqueNm

    ; Circular moving average:
    ;   1. subtract the oldest buffered sample from the running sum
    ;   2. overwrite that buffer slot with the new sample
    ;   3. add the new sample to the running sum
    ;   4. advance the circular index
    ;   5. publish sum / 128
    lfsr    0, torque_history
    movf    torque_buf_idx, w, c
    addwf   FSR0L, f, c
    movlw   0x00
    addwfc  FSR0H, f, c

    movf    INDF0, w, c
    movwf   torque_oldest, c

    ; Remove the sample that is about to be overwritten.
    movf    torque_oldest, w, c
    subwf   torque_sum_l, f, c
    movlw   0x00
    subwfb  torque_sum_h, f, c

    ; Store the new sample in that same buffer slot.
    movf    torque_sample, w, c
    movwf   INDF0, c

    ; Add the new sample to the running sum.
    movf    torque_sample, w, c
    addwf   torque_sum_l, f, c
    movlw   0x00
    addwfc  torque_sum_h, f, c

    ; Advance the circular index and wrap back to 0 after slot 127.
    incf    torque_buf_idx, f, c
    movlw   128
    cpfseq  torque_buf_idx, c
    bra     _can_torque_average
    clrf    torque_buf_idx, c

_can_torque_average:
    ; Publish the moving average by dividing the running sum by 128.
    ; Since 128 = 2^7, this is just seven logical right shifts.
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

;----------------------------------------------------------------------
; _CAN_ComputeTorqueNm
; Converts raw little-endian torque bytes into whole Nm using:
;   torque = (raw - 750) / 30.625 = 8 * (raw - 750) / 245
; Result is truncated to an integer Nm and stored in torque_sample.
;----------------------------------------------------------------------
_CAN_ComputeTorqueNm:
    ; Copy the raw little-endian torque word into scratch registers so
    ; the original decoded bytes remain available if needed.
    movf    torque_l, w, c
    movwf   torque_calc_l, c
    movf    torque_h, w, c
    movwf   torque_calc_h, c

    ; Subtract 750 mV baseline (0x02EE). If below baseline, clamp to 0 Nm.
    movlw   0xEE
    subwf   torque_calc_l, f, c
    movlw   0x02
    subwfb  torque_calc_h, f, c
    bc      _can_torque_positive
    clrf    torque_sample, c
    return

_can_torque_positive:
    ; Multiply by 8 with three 16-bit left shifts.
    ; This implements the numerator of:
    ;   torque = 8 * (raw - 750) / 245
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
    ; Repeated subtraction by 245 (0x00F5).
    ; Each successful subtraction increments the integer Nm result by 1.
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
