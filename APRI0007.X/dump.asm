PROCESSOR 16F1789 ; WRONG, we need to update it to the PIC18F47Q84

; CONFIG1
  CONFIG  FOSC = INTOSC         ; Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable (WDT disabled)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable (PWRT disabled)
  CONFIG  MCLRE = ON            ; MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
  CONFIG  CP = OFF              ; Flash Program Memory Code Protection (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Memory Code Protection (Data memory code protection is disabled)
  CONFIG  BOREN = ON            ; Brown-out Reset Enable (Brown-out Reset enabled)
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
  CONFIG  IESO = ON             ; Internal/External Switchover (Internal/External Switchover mode is enabled)
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

; CONFIG2
  CONFIG  WRT = OFF             ; Flash Memory Self-Write Protection (Write protection off)
  CONFIG  VCAPEN = OFF          ; Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
  CONFIG  PLLEN = ON            ; PLL Enable (4x PLL enabled)
  CONFIG  STVREN = ON           ; Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
  CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
  CONFIG  LPBOR = OFF           ; Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
  CONFIG  DEBUG = OFF           ; In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
  CONFIG  LVP = ON             ; Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <xc.inc>
  
PSECT reset_vec, class = CODE, delta = 2
reset_vec:
        goto    start

PSECT isr_vec, class = CODE, delta = 2
        retfie

PSECT code

    retfie                      ;Consist in a return from interrupt
    
PSECT code                      ;Goes to the memory adress of the main loop
start:			;Main routine
   

    ;set a flag
    movlb 0x00
    clrf  0x20
    
    
    ;step 1
    banksel TRISC
    movlw 11111011B
    movwf TRISC


    banksel TRISA
    bsf     TRISA,7

    banksel OPTION_REG
    bcf     OPTION_REG,7

    banksel WPUA
    bsf     WPUA,7

    ;step2
    banksel PR2
    movlw 20
    movwf PR2
    
    ;step3
    banksel CCP1CON 
    movlw 00001100B
    movwf CCP1CON
    
    ;step 4
    
    banksel CCPR1L
    movlw 8; default = 25%
    movwf CCPR1L
    
    ;step 
    banksel PIR1
    bcf PIR1, 1
    
    banksel T2CON   ;prescaler = 1
    bsf T2CON, 1
    bcf T2CON, 0
    
    bsf T2CON, 2
    
    ;step 6 optional : not done
    
    
    
main_loop:                      ;Main loop

    goto    main_loop           ;Jump to the main loop
    
     end reset_vec
	


	




