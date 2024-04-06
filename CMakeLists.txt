cmake_minimum_required(VERSION 3.10)

project("violetta")

# プラットフォーム分岐
if(${PLATFORM} STREQUAL Linux)
    message("[BUILD PLATFORM] Linux")
    include_directories(Platform/Linux/Inc)
    add_definitions(-DLINUX)
elseif(${PLATFORM} STREQUAL STM32)
    message("[BUILD PLATFORM] STM32")
    include_directories(
        Platform/STM32/Inc
        arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/arm-none-eabi/include
        arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/arm-none-eabi/include/c++/13.2.1
        arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/arm-none-eabi/include/c++/13.2.1/arm-none-eabi
    )
    add_definitions(-DSTM32)
    # add_compile_options(
    #     -Icmsis_gcc.h
    #     -Imyint.h
    # )
    set(CMAKE_C_COMPILER "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-g++")
    set(CMAKE_ASM_COMPILER "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc")
    set(CMAKE_EXE_LINKER "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-g++")
    set(OBJCOPY "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-objcopy")
    set(SIZE "${CMAKE_CURRENT_SOURCE_DIR}/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-size")
else()
    message(FATAL_ERROR "[BUILD PLATFORM] Invalid or unspecified platform. Please specify it as -DPLATFORM=[Linux,STM32]")
endif()

# マウス分岐
if(${MOUSE} STREQUAL Violetta)
    message("[BUILD MOUSE] Violetta")
    add_definitions(-DMOUSE_VIOLETTA)
    add_definitions(-DSTM32L4P5xx)
    add_compile_options(-std=c++17)
elseif(${MOUSE} STREQUAL Zirconia2kai)
    message("[BUILD MOUSE] Zirconia2kai")
    add_definitions(
        -DMOUSE_ZIRCONIA2KAI
        -DSTM32F411xE
        -DUSE_FULL_LL_DRIVER
        -DPREFETCH_ENABLE=1
        -DINSTRUCTION_CACHE_ENABLE=1
        -DDATA_CACHE_ENABLE=1
        -DHSE_VALUE=25000000
        -DHSE_STARTUP_TIMEOUT=100
        -DLSE_STARTUP_TIMEOUT=5000
        -DLSE_VALUE=32768
        -DEXTERNAL_CLOCK_VALUE=12288000
        -DHSI_VALUE=16000000
        -DLSI_VALUE=32000
        -DVDD_VALUE=3000
    )
    include_directories(
        Generate/zirconia2kai/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Generate/zirconia2kai/Drivers/CMSIS/Include
        Generate/zirconia2kai/Drivers/STM32F4xx_HAL_Driver/Inc
    )
    add_compile_options(
        -ffunction-sections
        -fdata-sections
        -fstack-usage
        --specs=nano.specs
    )
    set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -std=gnu11")
    set(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -fno-use-cxa-atexit -std=c++17")
    set(CMAKE_AS_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -x assembler-with-cpp")
    set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -T${CMAKE_CURRENT_SOURCE_DIR}/Platform/STM32F411xE/STM32F411CEUx_FLASH.ld -Wl,-Map=${CMAKE_CURRENT_SOURCE_DIR}/build/${TARGET}.map,--cref -Wl,--gc-sections -static --specs=nano.specs --specs=nosys.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lnosys -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -Wl,--no-warn-rwx-segment -u _printf_float -u _scanf_float")
    # --specs=rdimon.specs
    enable_language(ASM)

    set(STARTUP_ASM_FILE Platform/STM32F411xE/startup_stm32f411xe.s)
else()
    message(FATAL_ERROR "[BUILD MOUSE] Invalid or unspecified micromouse name. Please specify it as -DMOUSE=[Violetta,Zirconia2kai]")
endif()

########################
### General Settings ###
########################

include_directories(.
    Project/taniho/Inc
    Library/act/Inc
    Library/hal/Inc
    Library/misc/Inc
    Library/mll/Inc
    Library/mpl/Inc
    Library/msg/Inc
)
add_definitions(
    -D_USE_MATH_DEFINES
)
add_compile_options(
    -Wall
    -fmessage-length=0
    # -fexceptions
    -fno-exceptions
    -funsigned-char
    # -fpermissive
    -Wno-narrowing
    -g
    -gdwarf-2
    # -fPIC # こいつがいるとクロック設定を変更できなかった
    # -O0
)

add_subdirectory(Library)
add_subdirectory(Platform)
add_subdirectory(Project)
add_subdirectory(Tool)
add_subdirectory(Generate)


########################
### Project Settings ###
########################

add_custom_target(debug_taniho)

# add_custom_target(test_taniho)
# add_dependencies(test_taniho test_taniho_binary)

# add_executable(test_taniho_binary
#     Test/mpl/test_led.cpp
# )

# add_test(
#     NAME example_test
#     COMMAND test_taniho_binary
# )

# プラットフォーム分岐
if(${PLATFORM} STREQUAL Linux)
    add_executable(debug_taniho_binary
        Project/taniho/taniho_linux.cpp
    )
    target_link_libraries(debug_taniho_binary
        Linux
    )
    add_dependencies(debug_taniho debug_taniho_binary)
    target_link_libraries(debug_taniho_binary
        Library
        taniho
        GitHash
    )
elseif(${PLATFORM} STREQUAL STM32)
    set(ELF_FILE debug_taniho_binary.elf)
    set(HEX_FILE debug_taniho_binary.hex)
    set(BIN_FILE debug_taniho_binary.bin)
    set(MAP_FILE debug_taniho_binary.map)
    add_executable(${ELF_FILE}
        Project/taniho/taniho.cpp
        ${STARTUP_ASM_FILE}
        $<TARGET_OBJECTS:STM32F411xE>
    )
    add_dependencies(debug_taniho ${ELF_FILE})
    target_link_libraries(${ELF_FILE}
        taniho
        GitHash
        Library
        # STM32F411xE
        STM32HAL
    )
    add_custom_command(TARGET ${ELF_FILE} POST_BUILD
        COMMAND ${OBJCOPY} -Obinary ${ELF_FILE} ${BIN_FILE}
        COMMAND ${OBJCOPY} -Oihex   ${ELF_FILE} ${HEX_FILE}
        COMMAND ${SIZE} --format=berkeley ${ELF_FILE} ${HEX_FILE}
        # COMMENT "Invoking: Cross ARM GNU Print Size"
    )
else()
    message(FATAL_ERROR "[BUILD PLATFORM] Invalid or unspecified platform. Please specify it as -DPLATFORM=[Linux,STM32]")
endif()

# # マウス分岐
# if(${MOUSE} STREQUAL Violetta)
# elseif(${MOUSE} STREQUAL Zirconia2kai)
# else()
#     message(FATAL_ERROR "[BUILD MOUSE] Invalid or unspecified micromouse name. Please specify it as -DMOUSE=[Violetta,Zirconia2kai]")
# endif()

# target_link_libraries(test_taniho_binary
#     gtest_main
# )


##################
### GoogleTest ###
##################

# if(${PLATFORM} STREQUAL Linux)
#     include(FetchContent)
#     FetchContent_Declare(
#         googletest
#         URL https://github.com/google/googletest/archive/06f44bc951046150f1348598854b211afdcf37fc.zip
#     )
#     set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#     FetchContent_MakeAvailable(googletest)

#     enable_testing()
# endif()

