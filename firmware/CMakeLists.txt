set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

include(support/toolchain.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(oshw_nfc_reader_app C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

add_compile_options(-O0 -g)

set(FREERTOS_SOURCE_DIR at32_lib/middlewares/freertos/source)
set(USBD_SOURCE_DIR at32_lib/middlewares/usb_drivers/src)

include_directories(
        include
        board
        source/pn532
        at32_lib/libraries/cmsis/cm4/core_support 
        at32_lib/libraries/cmsis/cm4/device_support
        at32_lib/libraries/drivers/inc
        at32_lib/middlewares/usb_drivers/inc
        ${FREERTOS_SOURCE_DIR}/include
        ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM3
        source/ccid
    )

add_definitions(-DDEBUG -DRTOS_ENABLED -DAT32F415KBU7_4)

file(GLOB_RECURSE SOURCES "source/*.*" "board/*.*"
        "at32_lib/libraries/cmsis/*.*" "at32_lib/libraries/drivers/*.*"
        "source/ccid/*.*" 
    )

file(GLOB_RECURSE FREERTOS_SOURCES
        ${FREERTOS_SOURCE_DIR}/croutine.c
        ${FREERTOS_SOURCE_DIR}/event_groups.c
        ${FREERTOS_SOURCE_DIR}/list.c
        ${FREERTOS_SOURCE_DIR}/queue.c
        ${FREERTOS_SOURCE_DIR}/stream_buffer.c
        ${FREERTOS_SOURCE_DIR}/tasks.c
        ${FREERTOS_SOURCE_DIR}/timers.c
        ${FREERTOS_SOURCE_DIR}/portable/memmang/heap_4.c
        ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM3/*.c
    )
file(GLOB_RECURSE USBD_SOURCES
        ${USBD_SOURCE_DIR}/usb_core.c
        ${USBD_SOURCE_DIR}/usbd_core.c
        ${USBD_SOURCE_DIR}/usbd_int.c
        ${USBD_SOURCE_DIR}/usbd_sdr.c
    )
list(FILTER SOURCES EXCLUDE REGEX "at32_lib/libraries/cmsis/dsp/.*")
list(FILTER SOURCES EXCLUDE REGEX "at32_lib/libraries/cmsis/cm4/device_support/startup/iar/.*")
list(FILTER SOURCES EXCLUDE REGEX "at32_lib/libraries/cmsis/cm4/device_support/startup/mdk/.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/at32_lib/libraries/cmsis/cm4/device_support/startup/gcc/linker/AT32F415xB_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
#add_link_options(-specs=nano.specs -lc -lgcc)
#add_link_options(-lc -lgcc)
add_link_options(-specs=nosys.specs)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${FREERTOS_SOURCES} ${USBD_SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

set(JFLASH_FILE ${PROJECT_BINARY_DIR}/upload.jlink)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND echo "h" > ${JFLASH_FILE}
    COMMAND echo "loadbin ${BIN_FILE}" >> ${JFLASH_FILE}
    COMMAND echo "r" >> ${JFLASH_FILE}
    COMMAND echo "g" >> ${JFLASH_FILE}
    COMMAND echo "q" >> ${JFLASH_FILE}
    COMMENT "Generating upload.jlink"
)

add_custom_target(upload_jlink
    COMMAND ${JLINK_EXECUTABLE} -device AT32F415KBU7-4 -speed 10000 -if swd -autoconnect 1 -NoGui 1 -CommanderScript ${JFLASH_FILE}
    DEPENDS ${PROJECT_NAME}.elf
)
