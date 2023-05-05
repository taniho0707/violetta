######################################
# Default Target
######################################
# TARGET = Project1

######################################
# building variables
######################################
DEBUG = 1
# optimization
OPT = -Og

#######################################
# Build tool
#######################################
TOOLSET = gcc
# TOOLSET = clang

######################################
# Executable Files
######################################
DEBUG_TARGETS_LIB = debug_lib
TEST_TARGETS_LIB  = test_lib
DEBUG_TARGETS_TANIHO = debug_taniho
TEST_TARGETS_TANIHO  = test_taniho
DEBUG_TARGETS_REKO = debug_reko
TEST_TARGETS_REKO  = test_reko

#######################################
# Path
#######################################
BUILD_DIR = build
DEBUG_OBJECT_STM32HAL_DIR = $(BUILD_DIR)/obj/stm32hal/debug
DEBUG_OBJECT_ASM_STM32HAL_DIR = $(BUILD_DIR)/obj/stm32hal/debug
DEBUG_OBJECT_LIB_DIR = $(BUILD_DIR)/obj/lib/debug
DEBUG_TARGET_LIB_DIR = $(BUILD_DIR)/bin/lib/debug
TEST_OBJECT_LIB_DIR  = $(BUILD_DIR)/obj/lib/test
TEST_TARGET_LIB_DIR  = $(BUILD_DIR)/bin/lib/test
DEBUG_OBJECT_TANIHO_DIR = $(BUILD_DIR)/obj/taniho/debug
DEBUG_TARGET_TANIHO_DIR = $(BUILD_DIR)/bin/taniho/debug
TEST_OBJECT_TANIHO_DIR  = $(BUILD_DIR)/obj/taniho/test
TEST_TARGET_TANIHO_DIR  = $(BUILD_DIR)/bin/taniho/test
DEBUG_OBJECT_REKO_DIR = $(BUILD_DIR)/obj/reko/debug
DEBUG_TARGET_REKO_DIR = $(BUILD_DIR)/bin/reko/debug
TEST_OBJECT_REKO_DIR  = $(BUILD_DIR)/obj/reko/test
TEST_TARGET_REKO_DIR  = $(BUILD_DIR)/bin/reko/test

######################################
# source
######################################
ASM_SOURCE_STM32HAL_LIST = Generate/startup_stm32l4p5xx.s

C_SOURCE_STM32HAL_LIST = \
Generate/Core/Src/gpio.c \
Generate/Core/Src/adc.c \
Generate/Core/Src/dma.c \
Generate/Core/Src/spi.c \
Generate/Core/Src/tim.c \
Generate/Core/Src/usart.c \
Generate/Core/Src/stm32l4xx_hal_msp.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_adc.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
Generate/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
Generate/Core/Src/system_stm32l4xx.c

C_SOURCE_STM32HAL_DIR = Generate
ASM_SOURCE_STM32HAL_DIR = Generate
C_SOURCE_LIB_DIR = Library/Src
C_SOURCE_TANIHO_DIR = Project/taniho/Src
C_SOURCE_REKO_DIR = Project/reko/Src

C_INCLUDES_LIB_DIR = -ILibrary/Inc
C_INCLUDES_TANIHO_DIR = -IProject/taniho/Inc
C_INCLUDES_REKO_DIR = -IProject/reko/Inc

TARGET_MAINSRC = 
IGNORE_STM32HAL_SRC = 

DEBUG_OBJECT_GITHASH = $(BUILD_DIR)/obj/githash.o
GITHASH_HEADER = $(BUILD_DIR)/inc/githash.h
GITHASH_SOURCE = Tool/Githash/Githash.cpp
GITHASH_HEADER_DIR = $(BUILD_DIR)/inc


#######################################
# binaries
#######################################
ifeq ($(TOOLSET),gcc)
CC = arm-none-eabi-g++
AS = arm-none-eabi-g++ -x assembler-with-cpp
OBC = arm-none-eabi-objcopy
OBD = arm-none-eabi-objdump
LD = arm-none-eabi-ld
SZ = arm-none-eabi-size
NM = arm-none-eabi-nm
HEX = $(OBC) -O ihex
BIN = $(OBC) -O binary -S
else
ifeq ($(TOOLSET),clang)
CC = clang
AS = clang -x assembler-with-cpp
OBC = llvm-objcopy
OBD = llvm-objdump
LD = ld
SZ = llvm-size
NM = llvm-nm
HEX = $(OBC) -O ihex
BIN = $(OBC) -O binary -S
else
CC  = 
AS  = 
OBC = 
OBD = 
LD  = 
SZ  = 
NM  = 
HEX = 
BIN = 
endif
endif


#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -march=armv7e-m

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32L4P5xx \
-DARM_MATH_CM4 \
-DMOUSE_VIOLETTA
#-D__FPU_PRESENT


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IGenerate/Core/Inc \
-IGenerate/Drivers/STM32L4xx_HAL_Driver/Inc \
-IGenerate/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-IGenerate/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-IGenerate/Drivers/CMSIS/Include \
-I$(GITHASH_HEADER_DIR)


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
-fmessage-length=0 -fexceptions -fno-rtti -funsigned-char -fpermissive -fno-use-cxa-atexit -std=c++17 -Wno-narrowing -D _USE_MATH_DEFINES

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

ifeq ($(TOOLCHAIN),clang)
	CFLAGS += --target=thumbv7em-unknown-none-eabihf
	CFLAGS += -Wno-keyword-macro
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Generate/STM32L4P5CETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys \
-lgcc -lrdimon -lstdc++
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
-u _printf_float -Wl,-lgcc,-lc,-lm,-lrdimon,--gc-sections --static


#######################################
# build the application
#######################################
# 1. サブディレクトリを含むディレクトリリストの作成
SRCDIR_STM32HAL_LIST := $(shell find $(C_SOURCE_STM32HAL_DIR) -type d)
SRCDIR_LIB_LIST := $(shell find $(C_SOURCE_LIB_DIR) -type d)
SRCDIR_TANIHO_LIST := $(shell find $(C_SOURCE_TANIHO_DIR) -type d)
SRCDIR_REKO_LIST := $(shell find $(C_SOURCE_REKO_DIR) -type d)

# 2. 全てのcppファイルのリストの作成
# SRC_STM32HAL_LIST = $(foreach srcdir, $(SRCDIR_STM32HAL_LIST), $(wildcard $(srcdir)/*.c))
# SRC_ASM_STM32HAL_LIST = $(foreach srcdir, $(SRCDIR_STM32HAL_LIST), $(wildcard $(srcdir)/*.s))
SRC_LIB_LIST = $(foreach srcdir, $(SRCDIR_LIB_LIST), $(wildcard $(srcdir)/*.cpp))
SRC_TANIHO_LIST = $(foreach srcdir, $(SRCDIR_TANIHO_LIST), $(wildcard $(srcdir)/*.cpp $(srcdir)/*.c))
SRC_REKO_LIST = $(foreach srcdir, $(SRCDIR_REKO_LIST), $(wildcard $(srcdir)/*.cpp $(srcdir)/*.c))

# 3. トリミング
CUT_SRC_STM32HAL_LIST = $(subst $(C_SOURCE_STM32HAL_DIR),.,$(C_SOURCE_STM32HAL_LIST))
CUT_SRC_ASM_STM32HAL_LIST = $(subst $(C_SOURCE_STM32HAL_DIR),.,$(ASM_SOURCE_STM32HAL_LIST))
CUT_SRC_LIB_LIST = $(subst $(C_SOURCE_LIB_DIR),.,$(SRC_LIB_LIST))
CUT_SRC_TANIHO_LIST = $(subst $(C_SOURCE_TANIHO_DIR),.,$(SRC_TANIHO_LIST))
CUT_SRC_REKO_LIST = $(subst $(C_SOURCE_REKO_DIR),.,$(SRC_REKO_LIST))

# 4. オブジェクトファイル名の決定
STM32HAL_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_STM32HAL_DIR)/, $(CUT_SRC_STM32HAL_LIST:.c=.o))
STM32HAL_OBJ_ASM_LIST = $(addprefix $(DEBUG_OBJECT_STM32HAL_DIR)/, $(CUT_SRC_ASM_STM32HAL_LIST:.s=.o))
LIB_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(CUT_SRC_LIB_LIST:.cpp=.o))
TANIHO_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_TANIHO_DIR)/, $(CUT_SRC_TANIHO_LIST:.cpp=.o))
REKO_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_REKO_DIR)/, $(CUT_SRC_REKO_LIST:.cpp=.o))

# 5. テスト用にmainを含むファイルの除外
# FIXME: LIB,TANIHO,REKO毎にTESTを回せるように修正する
TEMP_SRC_TEST_LIST = $(filter-out %$(TARGET_MAINSRC), $(CUT_SRC_LIB_LIST))
TEST_MODULE_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(TEMP_SRC_TEST_LIST:.cpp=.o))

# 6. ディレクトリ構造のリスト化
STM32HAL_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_STM32HAL_DIR)/, $(SRCDIR_STM32HAL_LIST))
LIB_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(SRCDIR_LIB_LIST))
TANIHO_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_TANIHO_DIR)/, $(SRCDIR_TANIHO_LIST))
REKO_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_REKO_DIR)/, $(SRCDIR_REKO_LIST))


# 7. 各種ビルドターゲット設定
.PHONY: tanihotest rekotest clean

all: clean tanihobuild rekobuild

tanihobuild: $(DEBUG_TARGETS_TANIHO).hex $(DEBUG_TARGETS_TANIHO).bin

rekobuild: $(DEBUG_TARGETS_REKO).hex $(DEBUG_TARGETS_REKO).bin

libtest:

tanihotest:

rekotest:

clean:
	rm -fR $(BUILD_DIR)


# 8. ターゲット実行ファイルの生成
$(DEBUG_TARGETS_TANIHO).hex: $(DEBUG_TARGETS_TANIHO).elf
	$(HEX) $(DEBUG_TARGET_TANIHO_DIR)/$< $(DEBUG_TARGET_TANIHO_DIR)/$@

$(DEBUG_TARGETS_TANIHO).bin: $(DEBUG_TARGETS_TANIHO).elf
	$(BIN) $(DEBUG_TARGET_TANIHO_DIR)/$< $(DEBUG_TARGET_TANIHO_DIR)/$@

$(DEBUG_TARGETS_TANIHO).elf: $(TANIHO_OBJ_LIST) $(LIB_OBJ_LIST) $(STM32HAL_OBJ_LIST) $(STM32HAL_OBJ_ASM_LIST)
	@echo "$^"
	@if [ ! -e $(DEBUG_TARGET_TANIHO_DIR) ]; then mkdir -p $(DEBUG_TARGET_TANIHO_DIR); fi
	$(CC) -o $(DEBUG_TARGET_TANIHO_DIR)/$@ $^ $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$@.map,--cref -Wa,-a,-ad,-alms=$(BUILD_DIR)/$@.lst $(LIBS)
	rm -f $(GITHASH_HEADER) $(DEBUG_OBJECT_GITHASH)
	$(SZ) $(DEBUG_TARGET_TANIHO_DIR)/$@

$(DEBUG_TARGETS_REKO).hex: $(DEBUG_TARGETS_REKO).elf
	$(HEX) $(DEBUG_TARGET_REKO_DIR)/$< $(DEBUG_TARGET_REKO_DIR)/$@

$(DEBUG_TARGETS_REKO).bin: $(DEBUG_TARGETS_REKO).elf
	$(BIN) $(DEBUG_TARGET_REKO_DIR)/$< $(DEBUG_TARGET_REKO_DIR)/$@

$(DEBUG_TARGETS_REKO).elf: $(REKO_OBJ_LIST) $(LIB_OBJ_LIST) $(STM32HAL_OBJ_LIST) $(STM32HAL_OBJ_ASM_LIST)
	@echo "$^"
	@if [ ! -e $(DEBUG_TARGET_REKO_DIR) ]; then mkdir -p $(DEBUG_TARGET_REKO_DIR); fi
	$(CC) -o $(DEBUG_TARGET_REKO_DIR)/$@ $^ $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$@.map,--cref -Wa,-a,-ad,-alms=$(BUILD_DIR)/$@.lst $(LIBS)
	rm -f $(GITHASH_HEADER) $(DEBUG_OBJECT_GITHASH)
	$(SZ) $(DEBUG_TARGET_REKO_DIR)/$@


# 9. 中間バイナリの生成
$(DEBUG_OBJECT_ASM_STM32HAL_DIR)/%.o: $(ASM_SOURCE_STM32HAL_DIR)/%.s
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(AS) $(CFLAGS) -o $@ -c $<

$(DEBUG_OBJECT_STM32HAL_DIR)/%.o: $(C_SOURCE_STM32HAL_DIR)/%.c
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) -o $@ -c $<

$(DEBUG_OBJECT_LIB_DIR)/%.o: $(C_SOURCE_LIB_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_LIB_DIR) -o $@ -c $<

$(DEBUG_OBJECT_TANIHO_DIR)/%.o: $(C_SOURCE_TANIHO_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_TANIHO_DIR) -o $@ -c $<

$(DEBUG_OBJECT_REKO_DIR)/%.o: $(C_SOURCE_REKO_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_REKO_DIR) -o $@ -c $<

# 10. バージョン情報の生成
$(DEBUG_OBJECT_GITHASH): $(GITHASH_HEADER) $(GITHASH_SOURCE)
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) -I$(GITHASH_HEADER_DIR) -o $@ -c $(GITHASH_SOURCE)

$(GITHASH_HEADER): FORCE
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	Script/githash.sh > $(GITHASH_HEADER)
