ifeq ($(shell uname), Linux)
  TCHAIN  = arm-m4-eabi
else
  TCHAIN  = /usr/cross/bin/arm-m4-eabi
endif
TOOLDIR = /bin
REMOVAL = /bin/rm
WSHELL  = /bin/bash
MSGECHO = /bin/echo

# OPTIMIZE Definition
OPTIMIZE		= 2
#OPTIMIZE		= 0

# FPU Definition
USING_FPU		= -mfloat-abi=softfp  -mfpu=fpv4-sp-d16
#USING_FPU		= -mfloat-abi=soft

# Apprication Version
APP_VER = W.I.P

SUBMODEL		= STM32F407VGT6

MPU_DENSITY		= STM32F4xx
HSE_CLOCK 		= 8000000
PERIF_DRIVER    = USE_STDPERIPH_DRIVER

# Use FreeRTOS?
#OS_SUPPORT		= BARE_METAL
OS_SUPPORT		= USE_FREERTOS

FREERTOS_DIR = ../FreeRTOSV7.1.1
FREERTOS_COM = $(FREERTOS_DIR)/Demo/Common/include

# Synthesis makefile Defines
DEFZ = $(EVAL_BOARD)  $(MPU_DENSITY)  $(PERIF_DRIVER)    $(VECTOR_START) $(ROM_START)		\
	   $(USE_DISPLAY) $(USE_FONTSIZE) $(USE_TOUCH_SENCE) $(USE_LCD_SPI)	 $(USE_JPEG_LIB)	\
	   $(OS_SUPPORT)  $(USE_SPI_DMA)  $(USE_EXT_SRAM)    $(SUBMODEL)	 $(USE_KANJI)
SYNTHESIS_DEFS	= $(addprefix -D,$(DEFZ)) 							\
				 -DPACK_STRUCT_END=__attribute\(\(packed\)\) 		\
				 -DALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
				 -DMPU_SUBMODEL=\"$(SUBMODEL)\"						\
				 -DAPP_VERSION=\"$(APP_VER)\"						\
				 -DHSE_VALUE=$(HSE_CLOCK)UL 

# TARGET definition
TARGET 		= main
TARGET_ELF  = $(TARGET).elf
TARGET_SREC = $(TARGET).s19
TARGET_HEX  = $(TARGET).hex
TARGET_BIN  = $(TARGET).bin
TARGET_LSS  = $(TARGET).lss
TARGET_SYM  = $(TARGET).sym

FW  		= ../STM32F4xx_DSP_StdPeriph_Lib_V1.0.0/Libraries
FWLIB  		= $(FW)/STM32F4xx_StdPeriph_Driver
CM4LIB 		= $(FW)/CMSIS
CM4_DEVICE 	= $(CM4LIB)/Device/ST/STM32F4xx
CM4_CORE	= $(CM4LIB)/Include

# include PATH
INCPATHS	 = 	./						\
				./inc					\
				$(FWLIB)/inc  			\
				$(USBLIB)/inc			\
				$(CM4_DEVICE)/Include	\
				$(CM4_CORE)				\
				$(DISPLAY_INC)			\
				$(DISPLAY_DRV_INC)		\
				$(DISPLAY_MCU_INC)		\
				$(DISPLAY_FNT)			\
				$(DISPLAY_BMP)			\
				$(JPEGLIB)				\
				$(USBMSC)/inc			\
				$(FREERTOS_DIR)/Source/include	\
				$(FREERTOS_DIR)/Source/portable/GCC/ARM_CM4F \
				$(FREERTOS_COM)
INCLUDES     = $(addprefix -I ,$(INCPATHS))

# Set library PATH
LIBPATHS     = $(FWLIB) $(USBLIB) $(CM4LIB) $(DISPLAY_LIB)
LIBRARY_DIRS = $(addprefix -L,$(LIBPATHS))
# if you use math-library, put "-lm" 
MATH_LIB	 =	-lm

# LinkerScript PATH
LINKER_PATH =  ./linker
LINKER_DIRS = $(addprefix -L,$(LINKER_PATH)) 

# Object definition
OBJS 	 = $(CFILES:%.c=%.o) $(CPPFILES:%.cpp=%.o) 
LIBOBJS  = $(LIBCFILES:%.c=%.o) $(SFILES:%.s=%.o)

# C code PATH
SOURCE  = ./src
CFILES = \
 $(SOURCE)/$(TARGET).c				\
 $(SOURCE)/hw_config.c				\
 $(SOURCE)/rtc_support.c			\
 $(SOURCE)/uart_support.c			\
 $(SOURCE)/stm32f4xx_it.c			\
 $(SOURCE)/systick.c				\
 $(SOURCE)/syscalls.c

#/*----- STARTUP code PATH -----*/
STARTUP_DIR = $(CM4_DEVICE)/Source/Templates/gcc_ride7
ifeq ($(OS_SUPPORT),USE_FREERTOS)
SFILES += \
	$(SOURCE)/startup_stm32f4xx_rtos.s
else
SFILES += \
	$(STARTUP_DIR)/startup_stm32f4xx.s
endif

#/*----- STM32 library PATH -----*/
LIBCFILES = \
 $(FWLIB)/src/misc.c \
 $(FWLIB)/src/stm32f4xx_syscfg.c 	\
 $(FWLIB)/src/stm32f4xx_flash.c 	\
 $(FWLIB)/src/stm32f4xx_gpio.c 		\
 $(FWLIB)/src/stm32f4xx_fsmc.c 		\
 $(FWLIB)/src/stm32f4xx_rcc.c 		\
 $(FWLIB)/src/stm32f4xx_adc.c 		\
 $(FWLIB)/src/stm32f4xx_dma.c 		\
 $(FWLIB)/src/stm32f4xx_tim.c 		\
 $(FWLIB)/src/stm32f4xx_rtc.c 		\
 $(FWLIB)/src/stm32f4xx_sdio.c 		\
 $(FWLIB)/src/stm32f4xx_i2c.c 		\
 $(FWLIB)/src/stm32f4xx_spi.c 		\
 $(FWLIB)/src/stm32f4xx_usart.c 	\
 $(FWLIB)/src/stm32f4xx_pwr.c 		\
 $(SOURCE)/system_stm32f4xx.c		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/GenQTest.c 		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/BlockQ.c 			\
 $(FREERTOS_DIR)/Demo/Common/Minimal/blocktim.c 		\
 $(FREERTOS_DIR)/Demo/Common/Minimal/QPeek.c 			\
 $(FREERTOS_DIR)/Demo/Common/Minimal/PollQ.c 			\
 $(FREERTOS_DIR)/Source/tasks.c 						\
 $(FREERTOS_DIR)/Source/list.c 						\
 $(FREERTOS_DIR)/Source/queue.c 						\
 $(FREERTOS_DIR)/Source/portable/GCC/ARM_CM4F/port.c	\
 $(FREERTOS_DIR)/Source/portable/MemMang/heap_2.c


# TOOLCHAIN SETTING
CC 			= $(TCHAIN)-gcc
CPP 		= $(TCHAIN)-g++
OBJCOPY 	= $(TCHAIN)-objcopy
OBJDUMP 	= $(TCHAIN)-objdump
SIZE 		= $(TCHAIN)-size
AR 			= $(TCHAIN)-ar
LD 			= $(TCHAIN)-gcc
NM 			= $(TCHAIN)-nm
REMOVE		= $(REMOVAL) -f
REMOVEDIR 	= $(REMOVAL) -rf

# C and ASM FLAGS
CFLAGS  = -MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS)
CFLAGS += -mapcs-frame -mno-sched-prolog $(USING_FPU)
CFLAGS += -std=gnu99
CFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO)
CFLAGS += -fno-strict-aliasing -fsigned-char
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-schedule-insns2
CFLAGS += --param max-inline-insns-single=1000
CFLAGS += -fno-common -fno-hosted
CFLAGS += -Wall
#CFLAGS += -Os
#CFLAGS += -Wdouble-promotion
#CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CFLAGS += $(SYNTHESIS_DEFS)  

CXXFLAGS  = -MD -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
CXXFLAGS += -mthumb -mlittle-endian $(ALIGNED_ACCESS)
CXXFLAGS += -mapcs-frame -mno-sched-prolog $(USING_FPU)
#CXXFLAGS += -std=gnu99
CXXFLAGS += -gdwarf-2 -O$(OPTIMIZE) $(USE_LTO)
CXXFLAGS += -fno-strict-aliasing -fsigned-char
CXXFLAGS += -ffunction-sections -fdata-sections
CXXFLAGS += -fno-schedule-insns2
CXXFLAGS += --param max-inline-insns-single=1000
CXXFLAGS += -fno-common
CXXFLAGS += -Wall
#CXXFLAGS += -Os
#CXXFLAGS += -Wdouble-promotion
#CXXFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CXXFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<) 
CXXFLAGS += $(SYNTHESIS_DEFS)  

# Linker FLAGS -mfloat-abi=softfp -msoft-float
LDFLAGS  = -mcpu=cortex-m4 -march=armv7e-m -mthumb
LDFLAGS += -u g_pfnVectors -Wl,-static -Wl,--gc-sections -nostartfiles
LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += $(LIBRARY_DIRS) $(LINKER_DIRS) $(MATH_LIB)

ifeq ($(USE_EXT_SRAM),DATA_IN_ExtSRAM)
LDFLAGS +=-T$(LINKER_PATH)/$(MPU_DENSITY)_EXTRAM.ld
else
LDFLAGS +=-T$(LINKER_PATH)/$(MPU_DENSITY).ld
endif

# Object Copy and dfu generation FLAGS
OBJCPFLAGS = -O
OBJDUMPFLAGS = -h -S -C
DFU	  = hex2dfu
DFLAGS = -w
 
all: gccversion build sizeafter

# Object Size Infomations
ELFSIZE = $(SIZE) -A -x $(TARGET).elf
sizeafter:
	@$(MSGECHO) 
	@$(MSGECHO) Size After:
	$(SIZE) $(TARGET).elf
	@$(SIZE) -A -x $(TARGET).elf
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
	@$(MSGECHO) "BUILD_TYPE = "$(OS_SUPPORT)
	@$(MSGECHO) "USING_DISPLAY = "$(USE_DISPLAY)
	@$(MSGECHO) 

# Build Object
build: $(TARGET_ELF) $(TARGET_LSS) $(TARGET_SYM) $(TARGET_HEX) $(TARGET_SREC) $(TARGET_BIN)

.SUFFIXES: .o .c .s .cpp  

$(TARGET_LSS): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Disassemble: $@
	$(OBJDUMP) $(OBJDUMPFLAGS) $< > $@ 
$(TARGET_SYM): $(TARGET_ELF)
	@$(MSGECHO)
	@$(MSGECHO) Symbol: $@
	$(NM) -n $< > $@
$(TARGET).hex: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) ihex $^ $@    
$(TARGET).s19: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) srec $^ $@ 
$(TARGET).bin: $(TARGET).elf
	@$(MSGECHO)
	@$(MSGECHO) Objcopy: $@
	$(OBJCOPY) $(OBJCPFLAGS) binary $< $@ 
$(TARGET).dfu: $(TARGET).hex
	@$(MSGECHO)
	@$(MSGECHO) Make STM32 dfu: $@
	$(DFU) $(DFLAGS) $< $@
	@$(MSGECHO)
$(TARGET).elf: $(OBJS) stm32.a
	@$(MSGECHO) Link: $@
	$(LD) $(CFLAGS) $(LDFLAGS) $^ -o $@
	@$(MSGECHO)

stm32.a: $(LIBOBJS)
	@$(MSGECHO) Archive: $@
	$(AR) cr $@ $(LIBOBJS)    
	@$(MSGECHO)
.c.o:
	@$(MSGECHO) Compile: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.cpp.o:
	@$(MSGECHO) Compile: $<
	$(CPP) -c $(CXXFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)
.s.o:
	@$(MSGECHO) Assemble: $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $@
	@$(MSGECHO)

# Flash and Debug Program
debug :
	$(WSHELL) /c start /B $(INSIGHT) $(TARGET).elf
	$(OCD) $(OCD_CMD) -c "soft_reset_halt"
program :
	$(OCD) $(OCD_CMD) -c "mt_flash $(TARGET).elf"


# Drop files into dust-shoot
.PHONY clean:
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).obj
	$(REMOVE) stm32.a
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).s19
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).dfu
	$(REMOVE) $(wildcard *.stackdump)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(AOBJ)
	$(REMOVE) $(LIBOBJS)
	$(REMOVE) $(LST)
	$(REMOVE) $(CFILES:.c=.lst)
	$(REMOVE) $(CFILES:.c=.d)
	$(REMOVE) $(CPPFILES:.cpp=.lst)
	$(REMOVE) $(CPPFILES:.cpp=.d)
	$(REMOVE) $(LIBCFILES:.c=.lst)
	$(REMOVE) $(LIBCFILES:.c=.d)
	$(REMOVE) $(SFILES:.s=.lst)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.d)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.lst)
	$(REMOVE) $(wildcard $(DISPLAY_DRV_SRC)/*.o)
	$(REMOVE) $(wildcard $(CM4_DEVICE)/*.d)
	$(REMOVE) $(wildcard $(CM4_DEVICE)/*.lst)
	$(REMOVEDIR) .dep
	@$(MSGECHO)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
