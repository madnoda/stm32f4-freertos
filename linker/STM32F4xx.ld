OUTPUT_FORMAT ("elf32-littlearm")

/* Memory Spaces Definitions */
MEMORY
{
	RAM1 (xrw)	: ORIGIN = 0x20000000, LENGTH = 112k	/* Main Embedded SRAM 					*/
	RAM2 (xrw)	: ORIGIN = 0x2001C000, LENGTH = 16k		/* Etheret & USB Specific Embedded SRAM */
	RAM3 (xrw)	: ORIGIN = 0x10000000, LENGTH = 64k		/* Core-Coupled SRAM 					*/
	RAM4 (xrw)	: ORIGIN = 0x40024000, LENGTH = 4K		/* Backuped SRAM 						*/
	RAM5 (xrw)	: ORIGIN = 0x68000000, LENGTH = 1024K	/* External SRAM (FSMC) 				*/
	ROM (rx)	: ORIGIN = 0x08000000, LENGTH = 1024K	/* Main Embedded FlashROM				*/
}

/* higher address of the stack bottom */
_estack = ORIGIN(RAM3)+LENGTH(RAM3);

/* higher address of the heap end */
_heap_end = ORIGIN(RAM1)+LENGTH(RAM1)-4;	/* due to 4byte alignments */


/* include the section management sub-script */
INCLUDE "STM32F4_FLASH.ld"
