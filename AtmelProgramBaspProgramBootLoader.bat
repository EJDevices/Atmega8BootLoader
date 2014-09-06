cd ".\BootLoader\Release"
avrdude -p m8 -c usbasp -u -U flash:w:BootLoader.hex 
rem HFUSE : 11011110 0xDE BOOTRST BOOTSZ1 BOOTSZ0 (128 word boot loader, reset at word 0x0F80) 
rem LFUSE : 00011111 0x1F BODLEVEL BODEN SUT1 SUT0 CKSEL3,2,1 (8MHz ext oscillation with short startup and BOD)
rem LOCK :  00001111 0X0F BLB12(0) BLB11(0) (no SPM/LPM from boot loader)
avrdude -p m8 -c usbasp -u -U hfuse:w:0xDE:m -U lfuse:w:0x1F:m -U lock:w:0x0F:m
pause