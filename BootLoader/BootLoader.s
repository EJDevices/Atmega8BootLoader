/*
 * BootLoader.s
 *
 *	Created: 7/05/2014 9:05:51 PM
 *  Author: EJ Devices
 *  Copyright EJ Devices 2014
*/ 

#include <avr/io.h>
#include "../../Common/standard_macros.h"
#include "motor_board_config.h"

.section .blvects,"ax",@progbits

;export our symbols for use in user code
.global __vector_reset
.global bootloader_run ; r0 bit1 must be clear before calling this function
.global bootloader_crc16
.global bootloader_comms_tx_byte
.global bootloader_comms_rx_wait_byte
.global bootloader_spm_wait


// number of times a garbled packet is received before giving up
#define BOOT_LOADER_PACKET_RETRIES 10

//EModem control commands
#define EMODEM_SOH         0x01
#define EMODEM_EOT         0x04
#define EMODEM_ACK         0x06
#define EMODEM_NACK         0x15

// debugging use only
#define EMODEM_NAK_PG      0x16
#define EMODEM_NAK_CRC     0x17
#define EMODEM_NAK_SOH         0x18
#define EMODEM_NAK_VERIFY  0x19
#define EMODEM_EOF         0x1A
#define EMODEM_NAK_STX	   0x1B
#define EMODEM_NAK_GIVEUP   0x1C




; location of the page buffer in memory only the high byte is used, low byte is zero
#define BOOT_LOADER_BUFFER_ADDR_H 0x03
; location of SP. Note that SPL is not defined, so stack can be initialised anywhere between 0x0200 and 0x02FF
; It can not be 0x01 as could conflict with registers, and can't be 0x03 as will conflict with buffer
#define BOOT_LOADER_STACK_ADDR_H 0x02

; page size
#define BOOT_LOADER_PAGE_SIZE SPM_PAGESIZE	

#define BOOT_LOADER_VERSION_NUMBER 0
#define BOOT_LOADER_VERSION (BOOT_LOADER_VERSION_NUMBER << 2 | (BOOT_LOADER_PAGE_SIZE >> 6))

; note because the buffer include crc bytes, max page size 256 - 2
; Thus the boot loader only supports devices with BOOT_LOADER_PAGE_SIZE of 128 or less

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Reset Entry
;; 
__vector_reset:
; If called from reset, clr T flag
; To call the boot loader from user code must have T flag set
	clt ; clear T flag ; just in case we didn't get here by a reset
bootloader_run: 
	; disable interrupts
	cli

	; set stack at BOOT_LOADER_STACK_ADDR_H, don't care about SPL
	ldi		r24, BOOT_LOADER_STACK_ADDR_H
	out		_SFR_IO_ADDR(SPH), r24

	; set wdt for 250MS
	; enable WDT change, note we user R18 as (1 << WDCE)|(1 << WDE) == (1 << RXEN)|(1 << TXEN)
	; we recycle the use r18 in bootloader_main, to save one instruction
	ldi		r18,  (1 << WDCE)|(1 << WDE)	
	out		_SFR_IO_ADDR(WDTCR), r18	
	ldi		r24, (1 << WDE)	|(1 << WDP2)		; set to 250MS
	out		_SFR_IO_ADDR(WDTCR), r24
	;wdr		; perform a watchdog reset to ensure correct timings
	
	
	; board initialisation
	
	; load 8MHz calibration byte, in case we are using internal oscillator for whatever reason
	; OSCCAL = 0x9D
	; ldi	r24, 0x9D
	; out	_SFR_IO_ADDR(OSCCAL), r24								

	; set button direction as input and rs485 tx as output
	ldi		r24, (1 << RS485_TX_ENABLE_PIN) ; | ~(REG1 << BUTTON_ON_PIN | 1 << BUTTON_OFF_PIN)
	out		_SFR_IO_ADDR(DDRREG(BUTTON_RS485_PORT)), r24
	; enable pullups on button inputs
	ldi		r24, (1 << BUTTON_ON_PIN | 1 << BUTTON_OFF_PIN)
	out		_SFR_IO_ADDR(PORTREG(BUTTON_PORT)), r24

	; DDRREG(LED_PORT) = 1 << LED_ORANGE_PIN | 1 << LED_GREEN_PIN | 1 << LED_YELLOW_PIN | 1 << LED_RED_PIN
	; leds are out r16
	ldi		r16, 1 << LED_ORANGE_PIN | 1 << LED_GREEN_PIN | 1 << LED_YELLOW_PIN | 1 << LED_RED_PIN
	out		_SFR_IO_ADDR(DDRREG(LED_PORT)), r16
		
	; run boot loader if if buttons are down (active low)	
	in		r24, _SFR_IO_ADDR(PINREG(BUTTON_PORT))
	andi	r24, (1 << BUTTON_ON_PIN | 1 << BUTTON_OFF_PIN)		
	breq	bootloader_main

	; if T flag set, run boot loader, otherwise jump to user code
	brts	bootloader_main	
	rjmp	homebase	

bootloader_main:
	; This implements a derivative of XMODEM-CRC
	; It transmits BOOT_LOADER_PAGE_SIZE and ~BOOT_LOADER_PAGE_SIZE to let host know what the page size is and version of the boot loader
	; It waits for XMODEM packets SOH, packet_number, ~packet_number, data[BOOT_LOADER_PAGE_SIZE], crc high, crc low, 
	; To terminate host sends an EOT
	; Note that packet_number is one based

	; r31:r30 (Z) contains the page pointer for the flash
	; r29:r28 (Y) contains pointer to the buffer
	; r25 retry counter
	; r18 == (1 << RXEN)|(1 << TXEN)
	; r26 the led port state
	; r16 the led port bits	
	; r27 is the packet counter
	; r20,21,22,24 are temps

	; initialise registers	
	clr		r31		; clear PGADDR (Z) for flash
	clr		r30
	movw	r26, r30 ; clear the page counter and led state	
	ldi		r29, BOOT_LOADER_BUFFER_ADDR_H ; setup buffer pointer in Y register
	;clr		r28 ; no need to clear as cleared before first use, below
	

	; initialise comms
	; UCSRA = 0
	; UCSRB = (1 << RXEN)|(1 << TXEN)
	; UCSRC = (1 << URSEL)|(1 << UCSZ1)|(1 << UCSZ0)	
	; UBRRH = 0
	;out		_SFR_IO_ADDR(UCSRB), r30		; flush RX buffers
    out		_SFR_IO_ADDR(UCSRA), r30		
    out		_SFR_IO_ADDR(UCSRB), r18 ; r18 contains (1 << RXEN)|(1 << TXEN)
	ldi		r24, (1 << URSEL)|(1 << UCSZ1)|(1 << UCSZ0)	
	out		_SFR_IO_ADDR(UCSRC), r24
	out		_SFR_IO_ADDR(UBRRH), r30	
	; set for 250Kbaud (1)
	ldi		r24, BAUD_UBRRL
	out		_SFR_IO_ADDR(UBRRL), r24		

	; transmit BOOT_LOADER_VERSION
	ldi		r24, BOOT_LOADER_VERSION 		
	rcall	bootloader_comms_tx_byte
	; transmit ~BOOT_LOADER_VERSION
	ldi		r24, ~BOOT_LOADER_VERSION 	
	rcall	bootloader_comms_tx_byte

	; success loop, reset retry count
bootloader_run_packet_loop_success:
	ldi		r25, BOOT_LOADER_PACKET_RETRIES	
	inc		r27							; increment packet counter
; fail loop, don't reset retries
bootloader_run_packet_loop_fail:
	; toggle leds	
	eor		r26, r16							; toggle leds
    out		_SFR_IO_ADDR(PORTREG(LED_PORT)), r26	

	; XMODEM-CRC rx loop
	; wait for either rx packet EMODEM_SOH, PACKET, ~PACKET, DATA, CRCH, CRCL
	; or if rx == EMODEM_EOT, send MODEM_ACK and exit		
	rcall	bootloader_comms_rx_wait_byte 	
	cpi		r24, EMODEM_SOH
	breq	bootloader_run_rx_packet	
	cpi		r24, EMODEM_EOT
	brne	bootloader_run_rx_retry
	rcall	bootloader_comms_tx_ack		; send final ACK
	rjmp	bootloader_exit

bootloader_run_rx_packet:

	; reset WDT
	wdr

	; receive packet counter and then ~packet_counter
	; if != to the packet counter then send EMODEM_NACK;			
	rcall	bootloader_comms_rx_wait_byte 
	cp		r24, r27	; compare of RX1 to packet counter (which is page counter)		
	brne	bootloader_run_rx_retry
	rcall	bootloader_comms_rx_wait_byte 
	and		r24, r27				; compare of RX1 + ~RX2 will give zero				
	brne	bootloader_run_rx_retry

	; load up buffer with received packet plus CRC
	; data is stored in Y (r29:r28) 
	clr		r28
bootloader_run_rx_packet_loop:
	rcall	bootloader_comms_rx_wait_byte 
	st		Y+, r24
	cpi		r28, BOOT_LOADER_PAGE_SIZE + 2 ; include CRC
	brne	bootloader_run_rx_packet_loop

	; calculate CRC on buffer and compare, send NACK if bad crc
	clr		r28			; r29:r28 contains the buffer
	ldi		r24, BOOT_LOADER_PAGE_SIZE + 2	; r24 contains the size of the buffer
	rcall   bootloader_crc16	; r21:r20 contains the crc result
	or		r20, r21		; crc is equal the result are 0x0000
	;ldi		r24, EMODEM_NAK_CRC
	brne	bootloader_run_rx_retry
	
	; perform flash page erase
    ldi		r24, (1<<PGERS) | (1 << SPMEN)					
	rcall   bootloader_spm_wait				
	; copy data from buffer in r28:r29 to temporary buffer	
	clr		r28
bootloader_page_write_loop:
	ld		r0, Y+						; copy buf_ptr to r1:r0
	ld		r1, Y+
	ldi		r24,(1 << SPMEN)			; spm copy byte (1 << SPMEN) == 1
	rcall	bootloader_spm_wait
	adiw	r30, 2					; increment Z pointer to next word
	cpi		r28, BOOT_LOADER_PAGE_SIZE
	brne	bootloader_page_write_loop	
	; write the flash
	subi	r30, BOOT_LOADER_PAGE_SIZE	; restore Z pointer
	sbci	r31, 0
	ldi		r24, (1<<PGWRT) | (1 << SPMEN)					
	rcall   bootloader_spm_wait		
	;clr		r0	; there is no need to clear r0 as we don't use it

	; reenable RWW section for reading
	ldi		r24, (1<<RWWSRE) | (1<<SPMEN)
	rcall   bootloader_spm_wait

	; verify flash
	; compare buffer at r29:r28 to flash page
	clr		r28				 
bootloader_page_verify_loop:
	lpm		r21, Z+
	ld		r22, Y+	
	cp		r22, r21
	brne	bootloader_run_rx_retry
	cpi		r28, BOOT_LOADER_PAGE_SIZE
	brne	bootloader_page_verify_loop	
	; send ack
	rcall	bootloader_comms_tx_ack
	rjmp	bootloader_run_packet_loop_success ; if we are sending ack we are successful
	; unsuccessful, send NACK, decrement retry counter and retry
bootloader_run_rx_retry:	
	ldi		r24, EMODEM_NACK
	rcall	bootloader_comms_tx_byte
	dec		r25		; decrement retry counter
	brne	bootloader_run_packet_loop_fail
bootloader_run_send_and_exit:
	rcall	bootloader_comms_tx_byte ; send final NACK
bootloader_exit:
	rjmp	bootloader_exit
	; end


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; r24 contains size (get's clobbered), r24 must be a multiple of 2, and has minimum value of 1 and maximum value of 255
; r29:r28 contains buf_ptr (r28 clobbered) 
; r21:r20 contain crc result 
; Calculate the CRC-XMODEM
; The buffer must be equal to the size plus 2 bytes for the crc
; To calculate a crc, these two byte must be zero and r21:r20 will contain the crc
; To check a crc, these two bytes must contain the crc and r21:r20 will be zero if correct
; The crc of zero size buffer is zero

/*int calcrc(ptr,	count)
char *ptr;
int count;
{
    int	crc, i;

    crc	= 0;
    while (--count >= 0) {
	   crc = crc ^ (int)*ptr++ << 8;
	   for (i = 0; i < 8; ++i)
	       if (crc & 0x8000)
		   crc = crc <<	1 ^ 0x1021;
	       else
		   crc = crc <<	1;
	   }
    return (crc	& 0xFFFF);
}*/

bootloader_crc16:		
	; r18 is the byte loop reg
	; r19 is the next byte loaded
	; r22, r23 are constants
	clr		r21			; clear up CRCH
	clr		r20			; clear up CRCL
	ldi		r23, 0x10   ; 0x1021
	ldi		r22, 0x21
bootloader_crc16_loop:
	ld		r19, Y+		; load to temp
	ldi		r18, 8		; 
bootloader_crc16_loop_byte:
	lsl		r19			; shift 24bit left
	rol		r20			
	rol		r21	
	brcc	bootloader_crc16_loop_byte_skip_eor ;if(crc & 0x8000)	
	eor		r21, r23
    eor		r20, r22	 ; XOR 0x1021 
bootloader_crc16_loop_byte_skip_eor:
	dec		r18
	brne	bootloader_crc16_loop_byte
	dec		r24
	brne	bootloader_crc16_loop 
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; r24 contains data to write
bootloader_comms_tx_ack:
	ldi		r24, EMODEM_ACK	
bootloader_comms_tx_byte:
	sbi		_SFR_IO_ADDR(PORTREG(RS485_PORT)), RS485_TX_ENABLE_PIN			;port_bit_set(PORTREG(RS485_PORT), RS485_TX_ENABLE_PIN)
    out		_SFR_IO_ADDR(UDR), r24		;UDR = dat;
bootloader_comms_tx_byte_wait_tx:
    sbis	_SFR_IO_ADDR(UCSRA), TXC		;while(!(UCSRA & (1<<TXC)));
    rjmp	bootloader_comms_tx_byte_wait_tx      	; 0x1e0a <bootloader_comms_tx_byte+0x4>
    sbi		_SFR_IO_ADDR(UCSRA), TXC			;UCSRA |= (1 << TXC); ; clear txc flag
    cbi		_SFR_IO_ADDR(PORTREG(RS485_PORT)), RS485_TX_ENABLE_PIN			;port_bit_clear(PORTREG(RS485_PORT), RS485_TX_ENABLE_PIN)
    ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; r24 contains data that was read
bootloader_comms_rx_wait_byte:
    sbis	_SFR_IO_ADDR(UCSRA), RXC	; 11
    rjmp	bootloader_comms_rx_wait_byte      	; 0x1e14 <bootloader_comms_rx_wait_byte>
    in		r24, _SFR_IO_ADDR(UDR)	; 12
    ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Execute SPM command and wait for it to complete
; r24 contains the SPM command
bootloader_spm_wait:
	out		_SFR_IO_ADDR(SPMCR), r24
	spm
bootloader_spm_wait_loop:
	in		r24, _SFR_IO_ADDR(SPMCR)	; invoke spm and wait for the instruction to complete
	sbrc	r24, SPMEN
	rjmp	bootloader_spm_wait_loop
	ret




