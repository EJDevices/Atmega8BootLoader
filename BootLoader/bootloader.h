/*
 * BootLoader.h
 *
 *	Created: 7/05/2014 9:05:51 PM
 *  Author: EJ Devices
 *  Copyright EJ Devices 2014
 */ 


#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_


typedef void (*PF_BOOTLOADER)();

// Run the boot loader from user code
// Set the T bit and jump to the correct offset
void bootloader_run()
{
	__asm__ __volatile__ (
		"set" "\n\t"
		::
	);
	((PF_BOOTLOADER) (0x1F02/2))();
}
 

#endif /* BOOTLOADER_H_ */