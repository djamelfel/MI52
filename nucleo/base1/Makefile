#Makefile for the ST32F401xE, include stm32f4xx.h for type definition
#a conf file must be include (stm32f4xx_hal_conf.h) for constant definition

PROJECT = base
FLOAT_ABI = hard
DEBUG = 1

#----------------------------------------------------------------------
# sources files
#----------------------------------------------------------------------
# main file
CSRCS = src/main.c 
# add SystemInit() 
CSRCS += src/system_stm32f4xx.c
#add interrupt handler
CSRCS += src/stm32f4xx_it.c
# add startup file to build
ASRCS = startup/startup_stm32f401xe.s 

#To use HAL --> cf template
#modify stm32f4xx.h
#modify stm32f4xx_it.c to add HAL_IncTick

#Library driver source files for example (all file are compiled)
#CSRCS_LIB = $(wildcard $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/*.c)

#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dcmi.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hash_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_irda.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nand.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_nor.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pccard.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sai.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_smartcard.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
#CSRCS_LIB += $(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c


#----------------------------------------------------------------------
# Include and library
#----------------------------------------------------------------------

STM_LIBRARY = ./../STM32Cube_FW_F4_V1.4.0

INCLUDE_PATHS = -I`pwd` -I`pwd`/inc 
#CMSI library 
INCLUDE_PATHS += -I$(STM_LIBRARY)/Drivers/CMSIS/Include -I$(STM_LIBRARY)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
#STM HAL Library :
INCLUDE_PATHS += -I$(STM_LIBRARY)/Drivers/STM32F4xx_HAL_Driver/Inc
#link script
LINKER_SCRIPT = startup/STM32F401CE_FLASH.ld

#----------------------------------------------------------------------
# TOOL DEFINITIONS
#----------------------------------------------------------------------
AS      = arm-none-eabi-as
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE 	= arm-none-eabi-size


#----------------------------------------------------------------------
# COMPILER AND ASSEMBLER OPTIONS
#----------------------------------------------------------------------

CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=$(FLOAT_ABI)
CC_FLAGS  = $(CPU) -c
AS_FLAGS =
#define the target version (needed in some include file) :
CC_SYMBOLS = -DSTM32F401xE -D__FPU_USED

LD_FLAGS = $(CPU) -Wl,--gc-sections -Wl,-Map=$(PROJECT).map,--cref 
LD_SYS_LIBS = -lm -lc -lgcc -lnosys
LD_SYS_LIBS += -lstdc++ -lsupc++ 

ifeq ($(HARDFP),1)
	FLOAT_ABI = hard
else
	FLOAT_ABI = softfp
endif

ifeq ($(DEBUG), 1)
  CC_FLAGS +=  -O0 -g
  AS_FLAGS += -gdwarf-2
  CC_SYMBOLS += -DDEBUG
else
  CC_FLAGS += -Os
  CC_SYMBOLS += -DNDEBUG
endif


#----------------------------------------------------------------------
# BUILD OBJECTS
#----------------------------------------------------------------------
# list all object files
#----------------------------------------------------------------------
# ?= execute OBJS= if OBJS doesn't exist
# $(SRCS:.c=.o) : substitute all file.c by file.o
OBJECTS = $(CSRCS:.c=.o) $(ASRCS:.s=.o) $(CPPSRCS:.cpp=.o)
OBJECTS_LIB = $(CSRCS_LIB:.c=.o) 
#compile all object files
%.o:%.c 
	$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<
%.o:%.s
	$(AS) $(CPU) $(AS_FLAGS) -o $@ $<
.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<


all: $(PROJECT).bin $(PROJECT).hex

clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJECTS) $(DEPS) $(OBJECTS_LIB)


#----------------------------------------------------------------------
# BUILD PROJECT
#----------------------------------------------------------------------

$(PROJECT).elf: $(OBJECTS) $(OBJECTS_LIB)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LD_SYS_LIBS) $(LIBRARY_PATHS) -o $@ $^
	$(SIZE) $@

$(PROJECT).bin: $(PROJECT).elf
	@$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size:
	$(SIZE) $(PROJECT).elf

DEPS = $(OBJECTS:.o=.d) $(OBJECTS_LIB:.o=.d)
-include $(DEPS)


