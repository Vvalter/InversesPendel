Project(HalloWelt)

cmake_minimum_required(VERSION 2.8)

# use the eCos support coming with CMake
# without this line CMake doesn't know how to build eCos applications
# all functions/macros coming from this file start with "ECOS_"

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
INCLUDE(EZS)
INCLUDE(EZSconvenience)

INCLUDE(ezs_ecos_stm32)

# include_directories(Inc)
# include_directories(Drivers/CMSIS/Include)
# include_directories(Drivers/CMSIS/Device/ST/STM32F4xx/Include)
# include_directories(Drivers/STM32F4xx_HAL_Driver/Inc)
# include_directories(Drivers/STM32F4xx_HAL_Driver/Legacy)



set(EZS_ARCH stm32f4)

include_directories(${PROJECT_SOURCE_DIR}/libEZS/include)
include_directories(${OPENCM3_PATH}/include)
include_directories(${PROJECT_SOURCE_DIR}/../include)

add_definitions("-D EZS_DUMB_SERIAL")
add_definitions("-D USE_HAL_DRIVER")
add_definitions("-D STM32F411xE")

#AUX_SOURCE_DIRECTORY(Drivers/STM32F4xx_HAL_Driver/Src CMSIS_SOURCES)
#UX_SOURCE_DIRECTORY(Src CMSIS_SOURCES)

set(LIBEZS_SOURCES
#	libEZS/src/ezs_tracer.cpp
#	libEZS/src/ezs_impure_ptr.c
	libEZS/drivers/${EZS_ARCH}/ezs_gpio.c
	libEZS/drivers/${EZS_ARCH}/ezs_dac.cpp
	libEZS/drivers/${EZS_ARCH}/ezs_serial.cpp
	libEZS/drivers/${EZS_ARCH}/ezs_counter.cpp
	libEZS/src/ezs_delay.c
	)

ECOS_EZS_ADDLIB("${OPENCM3_PATH}/lib/libopencm3_stm32f4.a")

SET(SRC_LIST
        ../src/main.c
        ../src/state_machine.c
	../src/lib.c
	../src/aufschwingen.c
	../src/messung.c
	../src/pid.c
	${LIBEZS_SOURCES}
	${CMSIS_SOURCES}
	)

set(TGT "app")

ECOS_ADD_EXECUTABLE(${TGT} ${SRC_LIST})
