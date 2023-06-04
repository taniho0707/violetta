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

# Platform
# STM32: build .elf and .hex
# Linux: build Linux binary file
# Windows: Build .exe
PLATFORM = STM32

#######################################
# Build tool
#######################################
# Toolset
# gcc: arm-none-eabi-gcc/g++
# clang: clang
TOOLSET = gcc

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
DEBUG_OBJECT_STM32HAL_DIR = $(BUILD_DIR)/$(PLATFORM)/obj/stm32hal/debug
DEBUG_OBJECT_ASM_STM32HAL_DIR = $(BUILD_DIR)/$(PLATFORM)/obj/stm32hal/debug
DEBUG_OBJECT_LIB_DIR = $(BUILD_DIR)/$(PLATFORM)/obj/lib/debug
DEBUG_TARGET_LIB_DIR = $(BUILD_DIR)/$(PLATFORM)/bin/lib/debug
TEST_OBJECT_LIB_DIR  = $(BUILD_DIR)/$(PLATFORM)/obj/lib/test
TEST_TARGET_LIB_DIR  = $(BUILD_DIR)/$(PLATFORM)/bin/lib/test
DEBUG_OBJECT_TANIHO_DIR = $(BUILD_DIR)/$(PLATFORM)/obj/taniho/debug
DEBUG_TARGET_TANIHO_DIR = $(BUILD_DIR)/$(PLATFORM)/bin/taniho/debug
TEST_OBJECT_TANIHO_DIR  = $(BUILD_DIR)/$(PLATFORM)/obj/taniho/test
TEST_TARGET_TANIHO_DIR  = $(BUILD_DIR)/$(PLATFORM)/bin/taniho/test
DEBUG_OBJECT_REKO_DIR = $(BUILD_DIR)/$(PLATFORM)/obj/reko/debug
DEBUG_TARGET_REKO_DIR = $(BUILD_DIR)/$(PLATFORM)/bin/reko/debug
TEST_OBJECT_REKO_DIR  = $(BUILD_DIR)/$(PLATFORM)/obj/reko/test
TEST_TARGET_REKO_DIR  = $(BUILD_DIR)/$(PLATFORM)/bin/reko/test

######################################
# Import makefile for specific target
######################################
ifeq ($(PLATFORM),STM32)
include Makefile.stm32.mk
else
ifeq ($(PLATFORM),Linux)
include Makefile.linux.mk
endif
endif

# 10. バージョン情報の生成
$(DEBUG_OBJECT_GITHASH): $(GITHASH_HEADER) $(GITHASH_SOURCE)
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) -I$(GITHASH_HEADER_DIR) -o $@ -c $(GITHASH_SOURCE)

$(GITHASH_HEADER): FORCE
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	Script/githash.sh > $(GITHASH_HEADER)
