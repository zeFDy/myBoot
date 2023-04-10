TARGET = myBoot.o

#CROSS_COMPILE = arm-linux-gnueabihf-
CROSS_COMPILE = "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.07/bin/arm-none-eabi-"
CC = $(CROSS_COMPILE)gcc.exe
AS = $(CROSS_COMPILE)as.exe
LD = $(CROSS_COMPILE)ld 
CP = $(CROSS_COMPILE)objcopy 
OD = $(CROSS_COMPILE)objdump.exe
ARCH = arm

LDSCRIPT := myBoot.lds
					
CCOPT = -mthumb -mthumb-interwork -mabi=aapcs-linux  -mno-unaligned-access -ffunction-sections -fdata-sections -fno-common -ffixed-r9  -msoft-float -march=armv7-a
LDSCRIPT := myBoot.lds

OPTIONS = -Os -Wall -Wstrict-prototypes -Wno-format-security -fno-builtin -ffreestanding -mthumb -mthumb-interwork -mabi=aapcs-linux -mword-relocations  -fno-pic  -mno-unaligned-access -mno-unaligned-access -ffunction-sections -fdata-sections -fno-common -ffixed-r9 -msoft-float -march=armv7-a -P -c 



start.o:			start.s
					$(CC) -x assembler-with-cpp start.S -o start.o -c
					$(OD) -d start.o >start.lst
					
lowlevel_init.o:	lowlevel_init.s
					$(AS) lowlevel_init.s -o lowlevel_init.o
					$(OD) -d lowlevel_init.o >lowlevel_init.lst
		
board.o:			board.c
					$(CC) $(OPTIONS) -c board.c -o board.o -nostartfiles -nodefaultlibs
					$(OD) -d board.o >board.lst
		
socfpga_spl.o:		socfpga_spl.c
					$(CC) $(OPTIONS) -c socfpga_spl.c -o socfpga_spl.o -nostartfiles -nodefaultlibs
					$(OD) -d socfpga_spl.o >socfpga_spl.lst

spl.o:				spl.c
					$(CC) $(OPTIONS) -c spl.c -o spl.o -nostartfiles -nodefaultlibs
					$(OD) -d spl.o >spl.lst

myBoot.o:			start.o lowlevel_init.o board.o socfpga_spl.o spl.o
					$(LD) -o myBoot.o start.o lowlevel_init.o board.o socfpga_spl.o spl.o -T myBoot.lds
					$(CP) -O binary myBoot.o myBoot.bin
					$(OD) -d myBoot.o >myBoot.lst