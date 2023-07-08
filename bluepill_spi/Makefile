######################################
# target
######################################
TARGET =$(BUILD_DIR)/bluepill_spi


######################################
# building variables
######################################
# debug build?
DEBUG = 1

#######################################
# paths
#######################################
# Build path
BUILD_DIR :=build
TARGET_PATH =$(CURDIR)

LIB_PATH =$(LIBEMB_PATH)
DRIVERS_PATH = $(LIB_PATH)/drv

ifeq ($(STM32REPO), )
$(error Missing STM32REPO environment variable)
endif

REPOSITORY :=$(CURDIR)

$(info $(REPOSITORY))

######################################
# source
######################################

# Includes
C_INCLUDES =  \
$(TARGET_PATH)/Inc \
$(REPOSITORY)/Drivers/STM32F1xx_HAL_Driver/Inc \
$(REPOSITORY)/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
$(REPOSITORY)/Drivers/CMSIS/Include \
$(REPOSITORY)/Drivers/CMSIS/Core/Include \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \

# AS includes
AS_INCLUDES =

# C sources
C_SOURCES =  \
$(TARGET_PATH)/USBD/usbd_desc.c \
$(TARGET_PATH)/USBD/usbd_conf.c \
$(TARGET_PATH)/USBD/usb_device.c \
$(TARGET_PATH)/USBD/usbd_cdc_if.c \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
$(REPOSITORY)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(REPOSITORY)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
$(REPOSITORY)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
$(REPOSITORY)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
$(REPOSITORY)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
$(TARGET_PATH)/Src/startup_stm32f103.c \
$(TARGET_PATH)/Src/spi_stm32f1xx.c \
$(TARGET_PATH)/Src/dma_stm32f1xx.c \
$(TARGET_PATH)/Src/gpio_stm32f1xx.c \
$(TARGET_PATH)/Src/bluepill_spi.c \
$(TARGET_PATH)/Src/fifo.c \
$(TARGET_PATH)/Src/stimer.c \


CPP_SOURCES = \

# ASM sources
ASM_SOURCES =  \


# link script
LDSCRIPT =$(TARGET_PATH)/Src/STM32F103C8Tx_FLASH.ld

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef _GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S


ifeq ($(shell uname -s), Linux)
JLINK =JLinkExe
else
JLINK ="C:\Tools\JLink_V500\jlink"
endif


#######################################
# CFLAGS
#######################################
# cpu
CPU =-mcpu=cortex-m3 -mthumb
DEVICE =STM32F103xB

# fpu
#FPU =

# float-abi
#FLOAT-ABI =

# mcu
MCU =$(CPU) $(FPU) $(FLOAT-ABI) 

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
USE_HAL_DRIVER \
$(DEVICE) \
BOARD_BLUEPILL \

# CPP defines
CPP_DEFS = \

# optimization
OPT = -Og -gdwarf-2

# compile gcc flags
ASFLAGS = $(MCU) $(OPT) $(AS_DEFS) $(addprefix -I, $(AS_INCLUDES)) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -Wall -fdata-sections -ffunction-sections

CPPFLAGS = $(MCU) $(OPT) $(addprefix -D, $(CPP_DEFS)) $(addprefix -I, $(C_INCLUDES)) -Wall -fdata-sections -ffunction-sections -fno-exceptions -fmessage-length=0 -fno-rtti


# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
# Assembly options
#CFLAGS += -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))


#######################################
# LDFLAGS
#######################################
# libraries
SPECS =-specs=nosys.specs  -specs=nano.specs
LIBS =#-lstdc++
LIBDIR = 
LDFLAGS = $(MCU) $(SPECS) -T$(LDSCRIPT) $(LIBDIR) -Wl,--gc-sections,-Map=$(TARGET).map,--cref

# default action: build all
all: $(TARGET).elf
	@echo
	@$(SZ) -B $(TARGET).elf

 $(TARGET).cfg:
	@echo "Creating opencod configuration file"
	echo "adapter driver jlink" >> $@
	echo "transport select swd" >> $@
	echo "source [find target/stm32f1x.cfg]" >> $@
	echo "adapter speed 4000" >> $@

 $(TARGET).jlink: $(TARGET).hex
	@echo "Creating Jlink configuration file..."
	@echo "erase" > $@
	@echo "loadfile  $(TARGET).hex" >> $@
	@echo "r" >> $@
	@echo "q" >> $@

program: $(TARGET).jlink
	@echo "Programing device"
#	openocd -f  $(BUILD_DIR)/$(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
	$(VERBOSE)$(JLINK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(TARGET).jlink


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "[CC] " $<
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "[AS] " $<
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	@echo "[CC] " $<
	@$(CPP) -c $(CPPFLAGS) $< -o $@

$(TARGET).elf: $(OBJECTS) Makefile
	@echo "Linking " $@
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@ $(LIBS)

$(TARGET).hex: $(TARGET).elf | $(BUILD_DIR)
	@echo "Building hex file:" $@
	@$(HEX) $< $@
	
$(TARGET).bin: $(TARGET).elf | $(BUILD_DIR)
	@echo "Building bin file:" $@
	@$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
#-include $(wildcard $(BUILD_DIR)/*.d)
ifeq ($(GCC_COLORS),)
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:locus=01:quote=01'
unexport GCC_COLORS
endif

# *** EOF ***