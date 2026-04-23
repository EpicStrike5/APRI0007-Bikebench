;======================================================================
; can_torque.inc - CAN Bus Interface for BAFANG PA261 Torque Sensor
; Processor: PIC18F47Q84 @ 64MHz (16 MIPS)
;
; Hardware: CAN module uses hardware CAN FD peripheral in CAN 2.0 mode
;           RB0 = CANTX (output to CAN transceiver)
;           RB1 = CANRX (input from CAN transceiver)
;           Requires external CAN transceiver (e.g. ATA6563, MCP2551)
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
;   CAN_GetMsgID       - Get message ID low byte in W
;   CAN_GetDLC         - Get data length code (0-8) in W
;   CAN_Transmit       - Send a CAN message (W = DLC)
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
CAN_PPS_CANRX_RB1   equ 0x09        ; CANRXPPS = RB1
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
    movlw   0x40                        ; MIDE=1 (match IDE bit)
    movwf   BANKMASK(C1MASK0T), 1

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
    btfsc   BANKMASK(C1FIFOSTA1L), 0, 1 ; TFNRFNIF = RX FIFO not empty?
    retlw   1                           ; Return 1, Z=0 (not zero)
    retlw   0                           ; Return 0, Z=1 (zero)

;----------------------------------------------------------------------
; CAN_Receive
; Read received CAN message from FIFO1 into buffer.
; Extracts message ID and DLC for easy access.
; Call CAN_IsRxReady first to ensure message is available.
;----------------------------------------------------------------------
CAN_Receive:
    ; Get FIFO1 User Address (pointer to message in RAM)
    BANKSEL C1FIFOUA1L
    movf    BANKMASK(C1FIFOUA1L), 0, 1
    movwf   can_fifo_ptr_l, c
    movf    BANKMASK(C1FIFOUA1H), 0, 1
    movwf   can_fifo_ptr_h, c

    ; Add FIFO base address (0x3800) to get absolute address
    movlw   CAN_FIFO_BASE_H
    addwf   can_fifo_ptr_h, f, c

    ; Setup indirect addressing using FSR0
    movf    can_fifo_ptr_l, w, c
    movwf   FSR0L, c
    movf    can_fifo_ptr_h, w, c
    movwf   FSR0H, c

    ; Read message header (bytes 0-4) and data (bytes 8-15)
    ; CAN message object format:
    ;   Byte 0: SID[7:0]
    ;   Byte 1: SID[10:8] (bits 0-2), EID[4:0] (bits 3-7)
    ;   Byte 2: EID[12:5]
    ;   Byte 3: EID[17:13]
    ;   Byte 4: DLC[3:0], IDE, RTR, BRS, FDF
    ;   Bytes 5-7: reserved/timestamp
    ;   Bytes 8-15: Data[0-7]

    ; Extract SID (standard identifier, 11 bits)
    ; SID[7:0] from byte 0
    movf    POSTINC0, w, c              ; Read byte 0 (SID low)
    movwf   can_msg_id_l, c
    ; SID[10:8] from byte 1 (bits 0-2)
    movf    POSTINC0, w, c              ; Read byte 1
    andlw   0x07                        ; Mask SID[10:8]
    movwf   can_msg_id_h, c
    
    ; Skip bytes 2-3 (EID, not used for standard ID)
    movf    POSTINC0, w, c              ; Skip byte 2
    movf    POSTINC0, w, c              ; Skip byte 3

    ; Extract DLC from byte 4
    movf    POSTINC0, w, c              ; Read byte 4
    andlw   0x0F                        ; Mask DLC bits
    movwf   can_dlc, c

    ; Skip bytes 5-7 (reserved/timestamp)
    movf    POSTINC0, w, c              ; Skip byte 5
    movf    POSTINC0, w, c              ; Skip byte 6
    movf    POSTINC0, w, c              ; Skip byte 7

    ; Copy data bytes to buffer (up to 8 bytes)
    ; Always copy 8 bytes for simplicity
    movf    POSTINC0, w, c              ; Data byte 0
    movwf   can_rx_buffer+0, c
    movf    POSTINC0, w, c              ; Data byte 1
    movwf   can_rx_buffer+1, c
    movf    POSTINC0, w, c              ; Data byte 2
    movwf   can_rx_buffer+2, c
    movf    POSTINC0, w, c              ; Data byte 3
    movwf   can_rx_buffer+3, c
    movf    POSTINC0, w, c              ; Data byte 4
    movwf   can_rx_buffer+4, c
    movf    POSTINC0, w, c              ; Data byte 5
    movwf   can_rx_buffer+5, c
    movf    POSTINC0, w, c              ; Data byte 6
    movwf   can_rx_buffer+6, c
    movf    POSTINC0, w, c              ; Data byte 7
    movwf   can_rx_buffer+7, c

    ; Increment FIFO head pointer (acknowledge message read)
    BANKSEL C1FIFOCON1H
    bsf     BANKMASK(C1FIFOCON1H), 0, 1 ; UINC = 1

    return

;----------------------------------------------------------------------
; CAN_GetMsgID
; Get the 11-bit standard message ID from last received message.
; Output: can_msg_id_l (low 8 bits), can_msg_id_h (high 3 bits)
;         W = low byte of ID
;----------------------------------------------------------------------
CAN_GetMsgID:
    movf    can_msg_id_l, w, c
    return

;----------------------------------------------------------------------
; CAN_GetDLC
; Get the Data Length Code from last received message.
; Output: W = DLC (0-8)
;----------------------------------------------------------------------
CAN_GetDLC:
    movf    can_dlc, w, c
    return

; (Data byte access: read directly from buffer with known offsets)
;   movf    can_rx_buffer+0, w, c   ; Get data byte 0
;   movf    can_rx_buffer+1, w, c   ; Get data byte 1
;   ... etc up to can_rx_buffer+7

;----------------------------------------------------------------------
; CAN_Transmit
; Transmit a CAN message (for future expansion - basic implementation)
; Input: W = DLC (0-8)
;        can_msg_id_l, can_msg_id_h = 11-bit message ID
;        can_rx_buffer[0-7] = data to transmit
;----------------------------------------------------------------------
CAN_Transmit:
    movwf   can_dlc, c                  ; Save DLC

    ; Get TXQ User Address (pointer to next TX slot)
    BANKSEL C1TXQUAL
    movf    BANKMASK(C1TXQUAL), 0, 1
    movwf   can_fifo_ptr_l, c
    movf    BANKMASK(C1TXQUAH), 0, 1
    movwf   can_fifo_ptr_h, c

    ; Add FIFO base address
    movlw   CAN_FIFO_BASE_H
    addwf   can_fifo_ptr_h, f, c

    ; Setup indirect addressing
    movf    can_fifo_ptr_l, w, c
    movwf   FSR0L, c
    movf    can_fifo_ptr_h, w, c
    movwf   FSR0H, c

    ; Write message header
    ; Byte 0: SID[7:0]
    movf    can_msg_id_l, w, c
    movwf   POSTINC0, c
    ; Byte 1: SID[10:8]
    movf    can_msg_id_h, w, c
    andlw   0x07
    movwf   POSTINC0, c
    ; Bytes 2-3: EID (0 for standard frame)
    clrf    POSTINC0, c
    clrf    POSTINC0, c
    ; Byte 4: DLC, IDE=0, RTR=0, BRS=0, FDF=0
    movf    can_dlc, w, c
    andlw   0x0F
    movwf   POSTINC0, c
    ; Bytes 5-7: reserved (write 0)
    clrf    POSTINC0, c
    clrf    POSTINC0, c
    clrf    POSTINC0, c

    ; Write data bytes
    movf    can_rx_buffer+0, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+1, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+2, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+3, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+4, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+5, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+6, w, c
    movwf   POSTINC0, c
    movf    can_rx_buffer+7, w, c
    movwf   POSTINC0, c

    ; Request transmission: set UINC and TXREQ
    BANKSEL C1TXQCONH
    movlw   0x03                        ; UINC=1, TXREQ=1
    movwf   BANKMASK(C1TXQCONH), 1

    return
