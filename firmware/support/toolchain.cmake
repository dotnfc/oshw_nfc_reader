option(CROSS_COMPILE "Path to cross compiler" /usr/bin/arm-none-eabi-)

set(triple arm-none-eabi)
set(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if (CMAKE_HOST_WIN32)
    set (SUFFIX .exe)
else()
    set (SUFFIX "")
endif()

if (NOT CROSS_COMPILE)
    message(STATUS "ARM GCC Path in cmake-kits.json")
else()
    message(STATUS "ARM GCC Path in $(CROSS_COMPILE)")
    set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc${SUFFIX}")
    set(CMAKE_C_COMPILER_TARGET ${triple})
    set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++${SUFFIX}")
    set(CMAKE_CXX_COMPILER_TARGET ${triple})
    set(CMAKE_ASM_COMPILER "${CROSS_COMPILE}gcc${SUFFIX}")
    set(CMAKE_ASM_COMPILER_TARGET ${triple})

    set(CMAKE_AR "${CROSS_COMPILE}ar${SUFFIX}")
    set(CMAKE_OBJCOPY "${CROSS_COMPILE}objcopy${SUFFIX}")
    set(CMAKE_OBJDUMP "${CROSS_COMPILE}objdump${SUFFIX}")
    set(SIZE "${CROSS_COMPILE}size${SUFFIX}")
endif()

set(JLINK_EXECUTABLE "D:/Program/SEGGER/JLink_V784c/jlink.exe")

