######################################
# source
######################################
C_SOURCE_LIB_DIR = Library/act/Src Library/hal/Src Library/misc/Src Library/mll/Src Library/mpl/Src Library/msg/Src
C_SOURCE_TANIHO_DIR = Project/taniho/Src
C_SOURCE_REKO_DIR = Project/reko/Src

C_INCLUDES_LIB_DIR = -ILibrary/act/Inc -ILibrary/hal/Inc -ILibrary/misc/Inc -ILibrary/mll/Inc -ILibrary/mpl/Inc -ILibrary/msg/Inc
C_INCLUDES_TANIHO_DIR = -IProject/taniho/Inc
C_INCLUDES_REKO_DIR = -IProject/reko/Inc

TARGET_MAINSRC = 

DEBUG_OBJECT_GITHASH = $(BUILD_DIR)/obj/githash.o
GITHASH_HEADER = $(BUILD_DIR)/inc/githash.h
GITHASH_SOURCE = Tool/Githash/Githash.cpp
GITHASH_HEADER_DIR = $(BUILD_DIR)/inc

#######################################
# binaries
#######################################
ifeq ($(TOOLSET),gcc)
CC = g++
AS = g++ -x assembler-with-cpp
OBC = objcopy
OBD = objdump
LD = ld
SZ = size
NM = nm
else
ifeq ($(TOOLSET),clang)
CC = clang
AS = clang -x assembler-with-cpp
OBC = llvm-objcopy
OBD = llvm-objdump
LD = ld
SZ = llvm-size
NM = llvm-nm
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
# C defines
C_DEFS =  \
-DLINUX \
-DMOUSE_VIOLETTA

# C includes
C_INCLUDES =  \
-I$(GITHASH_HEADER_DIR)

# compile gcc flags
CFLAGS += $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall
CFLAGS += -fmessage-length=0 -fexceptions
#-fno-rtti
CFLAGS += -funsigned-char -fpermissive
#-fno-use-cxa-atexit 
CFLAGS += -std=c++17 -Wno-narrowing -D _USE_MATH_DEFINES

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

ifeq ($(TOOLCHAIN),clang)
	CFLAGS += -Wno-keyword-macro
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# libraries
LDFLAGS = 

#######################################
# build the application
#######################################
# 1. サブディレクトリを含むディレクトリリストの作成
SRCDIR_LIB_LIST := $(shell find $(C_SOURCE_LIB_DIR) -type d)
SRCDIR_TANIHO_LIST := $(shell find $(C_SOURCE_TANIHO_DIR) -type d)
SRCDIR_REKO_LIST := $(shell find $(C_SOURCE_REKO_DIR) -type d)

# 2. 全てのcppファイルのリストの作成
SRC_LIB_LIST = $(foreach srcdir, $(SRCDIR_LIB_LIST), $(wildcard $(srcdir)/*.cpp))
SRC_TANIHO_LIST = $(foreach srcdir, $(SRCDIR_TANIHO_LIST), $(wildcard $(srcdir)/*.cpp $(srcdir)/*.c))
SRC_REKO_LIST = $(foreach srcdir, $(SRCDIR_REKO_LIST), $(wildcard $(srcdir)/*.cpp $(srcdir)/*.c))

# 3. トリミング
CUT_SRC_LIB_LIST = $(subst $(C_SOURCE_LIB_DIR),.,$(SRC_LIB_LIST))
CUT_SRC_TANIHO_LIST = $(subst $(C_SOURCE_TANIHO_DIR),.,$(SRC_TANIHO_LIST))
CUT_SRC_REKO_LIST = $(subst $(C_SOURCE_REKO_DIR),.,$(SRC_REKO_LIST))

# 4. オブジェクトファイル名の決定
LIB_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(CUT_SRC_LIB_LIST:.cpp=.o))
TANIHO_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_TANIHO_DIR)/, $(CUT_SRC_TANIHO_LIST:.cpp=.o))
REKO_OBJ_LIST = $(addprefix $(DEBUG_OBJECT_REKO_DIR)/, $(CUT_SRC_REKO_LIST:.cpp=.o))

# 5. テスト用にmainを含むファイルの除外
# FIXME: LIB,TANIHO,REKO毎にTESTを回せるように修正する
TEMP_SRC_TEST_LIST = $(filter-out %$(TARGET_MAINSRC), $(CUT_SRC_LIB_LIST))
TEST_MODULE_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(TEMP_SRC_TEST_LIST:.cpp=.o))

# 6. ディレクトリ構造のリスト化
LIB_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_LIB_DIR)/, $(SRCDIR_LIB_LIST))
TANIHO_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_TANIHO_DIR)/, $(SRCDIR_TANIHO_LIST))
REKO_OBJ_DIR_LIST = $(addprefix $(DEBUG_OBJECT_REKO_DIR)/, $(SRCDIR_REKO_LIST))


# 7. 各種ビルドターゲット設定
.PHONY: tanihotest rekotest clean

all: tanihobuild rekobuild

tanihobuild: $(DEBUG_TARGETS_TANIHO)

rekobuild: $(DEBUG_TARGETS_REKO)

libtest:

tanihotest:

rekotest:

clean:
	rm -fR $(BUILD_DIR)

# 8. ターゲット実行ファイルの生成
$(DEBUG_TARGETS_TANIHO): $(TANIHO_OBJ_LIST) $(LIB_OBJ_LIST)
	@echo "$^"
	@if [ ! -e $(DEBUG_TARGET_TANIHO_DIR) ]; then mkdir -p $(DEBUG_TARGET_TANIHO_DIR); fi
	$(CC) -o $(DEBUG_TARGET_TANIHO_DIR)/$@ $^ $(LDFLAGS) $(LIBS)
	rm -f $(GITHASH_HEADER) $(DEBUG_OBJECT_GITHASH)
	$(SZ) $(DEBUG_TARGET_TANIHO_DIR)/$@

$(DEBUG_TARGETS_REKO).elf: $(REKO_OBJ_LIST) $(LIB_OBJ_LIST)
	@echo "$^"
	@if [ ! -e $(DEBUG_TARGET_REKO_DIR) ]; then mkdir -p $(DEBUG_TARGET_REKO_DIR); fi
	$(CC) -o $(DEBUG_TARGET_REKO_DIR)/$@ $^ $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$@.map,--cref -Wa,-a,-ad,-alms=$(BUILD_DIR)/$@.lst $(LIBS)
	rm -f $(GITHASH_HEADER) $(DEBUG_OBJECT_GITHASH)
	$(SZ) $(DEBUG_TARGET_REKO_DIR)/$@

# 9. 中間バイナリの生成
$(DEBUG_OBJECT_LIB_DIR)/%.o: $(C_SOURCE_LIB_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_LIB_DIR) -o $@ -c $<

$(DEBUG_OBJECT_TANIHO_DIR)/%.o: $(C_SOURCE_TANIHO_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_TANIHO_DIR) $(C_INCLUDES_LIB_DIR) -o $@ -c $<

$(DEBUG_OBJECT_REKO_DIR)/%.o: $(C_SOURCE_REKO_DIR)/%.cpp
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_REKO_DIR) -o $@ -c $<

$(LIB_OBJ_LIST): $(SRC_LIB_LIST)
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CC) $(CFLAGS) $(C_INCLUDES_LIB_DIR) -o $@ -c $<
