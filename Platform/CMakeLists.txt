cmake_minimum_required(VERSION 3.0)

# プラットフォーム分岐
if(${PLATFORM} STREQUAL Linux)
    add_subdirectory(Linux)
elseif(${PLATFORM} STREQUAL STM32)
    # マウス分岐
    if(${MOUSE} STREQUAL Violetta)
        add_subdirectory(STM32)
    elseif(${MOUSE} STREQUAL Zirconia2kai)
        add_subdirectory(STM32F411xE)
    else()
        message(FATAL_ERROR "[BUILD MOUSE] Invalid or unspecified micromouse name. Please specify it as -DMOUSE=[Violetta,Zirconia2kai]")
    endif()
else()
    message(FATAL_ERROR "[BUILD PLATFORM] Invalid or unspecified platform. Please specify it as -DPLATFORM=[Linux,STM32]")
endif()
