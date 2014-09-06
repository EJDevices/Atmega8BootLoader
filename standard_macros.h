/*
 * standard_macros.h
 *
 *	Created: 7/05/2014 9:05:51 PM
 *  Author: EJ Devices
 *
 */


#ifndef STANDARDMACROS_H_
#define STANDARDMACROS_H_

#define TRUE				1
#define FALSE				0

#define CONCAT(a, b)       a ## b
#define CONCAT3(a, b, c)   a ## b ## c


//register of PORT and bit define
#define PORTREG(No)        CONCAT(PORT, No)
#define PINREG(No)         CONCAT(PIN, No)
#define UDRREG(No)         CONCAT(UDR, No)
#define DDRREG(No)         CONCAT(DDR, No)
#define TXCBIT(No)         CONCAT(TXC, No)
#define RXCBIT(No)         CONCAT(RXC, No)
#define RXENBIT(No)        CONCAT(RXEN, No)
#define TXENBIT(No)        CONCAT(TXEN, No)
#define URSELBIT(No)       CONCAT(URSEL, No)

//comport register
#define UBRRHREG(No)       CONCAT3(UBRR, No, H)
#define UBRRLREG(No)       CONCAT3(UBRR, No, L)
#define UCSRAREG(No)       CONCAT3(UCSR, No, A)
#define UCSRBREG(No)       CONCAT3(UCSR, No, B)
#define UCSRCREG(No)       CONCAT3(UCSR, No, C)
#define UCSZBIT(No1, No2)  CONCAT3(UCSZ, No1, No2)

//calculate baudrate register
#define BAUDREG(BAUD)            ((unsigned int)((F_CPU * 10) / (16UL * BAUD) - 5) / 10)

//check baudrate register error
//mocro below maybe not same in different C compiler
#define FreqTemp(BAUD)           (16UL * BAUD * (((F_CPU * 10) / (16 * BAUD) + 5)/ 10))
#if ((FreqTemp * 50) > (51 * F_CPU)) || ((FreqTemp * 50) < (49 * F_CPU))
#error "BaudRate error > 2% ! Please check BaudRate and F_CPU value."
#endif


/*
typedef void (*PF_BOOTLOADER)();
static __inline__ void call_bootloader(void)
{ ((PF_BOOTLOADER) (0x1E02/2))(); }
	
typedef uint16_t (*PF_BOOTLOADER_CRC16)(uint8_t *buf, uint8_t buff_size);
static __inline__ uint16_t call_bootloader_crc16(uint8_t *buf, uint8_t buff_size)
{ return ((PF_BOOTLOADER_CRC16) (0x1E04/2))(buf, buff_size); }
*/
#endif /* STANDARDMACROS_H_ */