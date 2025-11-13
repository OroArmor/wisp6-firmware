;/************************************************************************************************************************************/
;/**
; * @file		TxM2.asm
; * @brief		RFID Transmit in F2
; * @details
; *
; * @author		Eli Orona, UW Sensor Systems Lab
; * @created
; * @last rev	
; *
; *	@notes
; *	@todo
; *	@calling	extern void TxM2(volatile uint8_t *data,uint8_t numBytes,uint8_t numBits,uint8_t TRext)
; */
;/************************************************************************************************************************************/

;/INCLUDES----------------------------------------------------------------------------------------------------------------------------
    .cdecls C,LIST
    %{
       #include "../globals.h"
       #include "rfid.h"
    %}
    .include "../internals/NOPdefs.asm"; Definitions of NOPx MACROs...
    .global TxClock, RxClock

;/PRESERVED REGISTERS-----------------------------------------------------------------------------------------------------------------
R_currByte	.set  R6 
R_prevState .set  R7
R_scratch0  .set  R8        
R_scratch1  .set  R9
R_scratch2  .set  R10

R_sent .set R15


;/SCRATCH REGISTERS-------------------------------------------------------------------------------------------------------------------
R_dataPtr	.set    R12				; Entry: address of dataBuf start is in R_dataPtr
R_byteCt    .set	R13				; Entry: length of Tx'd Bytes is in R_byteCt
R_bitCt 	.set	R14				; Entry: length of Tx'd Bits is in R_bitCt
R_TRext     .set  	R15				; Entry: TRext? is in R_TRext

.data
M2_STATE:
    .byte 0x90, 0x90, 0x00, 0x00
    .byte 0x28, 0x28, 0x00, 0x00
    .byte 0x30, 0x88, 0x00, 0x00
    .byte 0x30, 0x88, 0x00, 0x00 

.text
;/Timing Notes------------------------------------------------------------------------------------------------------------------------
    ;*   Cycles Between Bits: 9 (for LF=320kHz @ 11.52MHz CPU)                                                                      */
    ;/** @todo Make sure the proper link frequency is listed here, or give a table of LF vs clock frequency							*/
    ;*   Cycles Before First Bit Toggle: 29 worst case                                                                              */

;/Begin ASM Code----------------------------------------------------------------------------------------------------------------------
	.def  TxM2

TxM2:
	CLR		&TA0CTL				;[] Disable TimerA when doing the TX related stuffs to allow the system to go to lpm4 for sleep.
	BIS.W #BIT7, &PTXDIR;
	BIS.W #BIT7, &PTXOUT;
	BIC.W #BIT7, &PTXOUT;

    ;/Push the Preserved Registers----------------------------------------------------------------------------------------------------
	PUSHM.A #5, R10					;[?] Push all preserved registers onto stack R6-R10/** @todo Find out how long this takes */

	CALLA #TxClock	;Switch to TxClock
  
;*************************************************************************************************************************************
;                                                                                                                                    *
;                                                                                                                                    *
;  Optimizations:   For Each Byte sent, we need to do the following tasks:                                                           *
;                    - decrement byte count (R_byteCt)                                                                               *
;                    - test for more bytes                                                                                           *
;************************************************************************************************************************************/
Send_Pilot_Tones:
    ;/Prep Some Registers to Send (optimized scratch0/2 down below though)------------------------------------------------------------
    MOV.B   #0xFF, R_scratch1		;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble

    MOV.B   #4,    R_scratch2		;[1] load up numTones=4
    ;/Test to see if we should send pilot tones---------------------------------------------------------------------------------------
    TST.B   R_TRext					;[1] TRext means that we should send the pilot tones
    JZ      Send_A_Pilot_Tone		;[2] skip 'em if (!TRext)
    MOV.B   #16,   R_scratch2       ;[1] We are sending the preamble so 16 tones

    ;/Send Pilot Tones if TRext-------------------------------------------------------------------------------------------------------
Send_A_Pilot_Tone:
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    ;*Timing Optimization Shoved Here(5 free cycles)*/
    MOV.B   #0x00, R_scratch0		;[1] setup R_scratch0 as LOW (note: if this is skipped, make sure to do it in preamble below too)
    NOPx4							;[4] 4 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOP								;[1] 1 timing cycles
    DEC     R_scratch2				;[1] decrement the tone count
    TST.B   R_scratch2				;[1] keep sending until the count is zero
    JNZ     Send_A_Pilot_Tone	    ;[2] ""
    
;/************************************************************************************************************************************
;/													SEND PREAMBLE                                 							         *
;/ operation: The preamble signals a tag transmission.                                                                               *
;/                                                                                                                                   *
;/ tx sequence: 000...[010111]                                                                                                       *
;/************************************************************************************************************************************    
Send_Preamble:
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX       /* 0: HLHL */
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles

    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX       /* 1: HLLH */
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles


    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX       /* 0: LHLH */
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles

    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX       /* 1: LHHL */
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles

    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX       /* 1: HLLH */
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    NOPx5							;[5] 5 timing cycles

    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX        /* 1: LHHL */
    NOPx2							;[2]
    INC     R_bitCt                 ;[1] We need to read one extra bit to send the last signal

    MOV.B   @R_dataPtr+, R_currByte ;[2] Read the first byte of data
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX

    MOV     #0, R_prevState         ;[1] Clear the previous state
    RLA.B   R_currByte              ;[1] Rotate the top bit (V) out of R_currByte and into the 
    RLC.B   R_prevState             ;[1] fourth bit of the state 
    RLA.B   R_prevState             ;[1]
    RLA.B   R_prevState             ;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4] HIGH on PTXOUT.PIN_TX
    RLA.B   R_prevState             ;[1]
    BIS     #1, R_prevState         ;[1] Set the bottom bit so that state=V001 (S3 -> S1/S2)

    RLA     R_byteCt                ;[1] Multiply the byte count by 8 and add to the bit count
    RLA     R_byteCt                ;[1]
    RLA     R_byteCt                ;[1]
    MOV.B   R_scratch0, &PTXOUT		;[4] LOW on PTXOUT.PIN_TX
    ADD     R_byteCt, R_bitCt       ;[1]

    MOV     #1, R_scratch0          ;[1] We read the first bit already


Send_Bit:
    ADD     #M2_STATE, R_prevState    ;[1]
    MOV.B   @R_prevState, R_scratch1  ;[2]
    MOV.B   R_scratch1, &PTXOUT		  ;[4]

    RLA.B   R_currByte              ;[1]
    RLC.B   R_scratch1              ;[1]
    RLC.B   R_scratch1              ;[1]
    INC     R_scratch0              ;[1]
    BIT     #7, R_scratch0          ;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]

    JNZ Skip_Read                   ;[2]
    MOV.B   @R_dataPtr+, R_currByte ;[2]
Return_From_Skip:
    RLA.B   R_scratch1              ;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]

    RLA.B   R_scratch1              ;[1]
    ADDC    #0, R_scratch1          ;[1]
    MOV.B   R_scratch1, R_prevState ;[1]
    AND.B   #15, R_prevState        ;[1]
    CMP     R_scratch0, R_bitCt     ;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]
    JNE     Send_Bit                ;[2]
  
Clean_Up:
    NOPx2
    INV.B   R_scratch1				;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]
    NOPx4
    INV.B   R_scratch1				;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]
    NOPx5
    MOV.B   R_scratch1, &PTXOUT		;[4]
    NOPx4
    INV.B   R_scratch1				;[1]
    MOV.B   R_scratch1, &PTXOUT		;[4]

   	BIC.B	#0x81, &PTXOUT			;[] Clear 1.0 & 1.7 (1.0 is for old 4.1 HW, 1.7 is for current hack...) eventually just 1.0

    POPM.A #5, R10					;[?] Restore preserved registers R6-R10 /** @todo Find out how long this takes *

    RETA
    
Skip_Read:
    JMP Return_From_Skip            ;[2]
    NOP

    .end ;* End of ASM */


