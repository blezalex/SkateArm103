
include ../nanopb/extra/nanopb.mk

BUILD_DIR := build

.DEFAULT_GOAL := ${BUILD_DIR}/BalancingController.elf

STM32_KIT=$(wildcard stm_lib/src/*.c) $(wildcard syscalls/*.c) $(wildcard cmsis_boot/*.c) $(wildcard cmsis_boot/*/*.c)

HDRS := $(wildcard *.h) $(wildcard *.hpp) $(wildcard */*.h) $(wildcard */*.hpp) $(wildcard */*/*.h) $(wildcard */*/*.hpp) $(wildcard */*/*/*.h) $(wildcard */*/*/*.hpp) drv/comms/protocol.pb.h
SRCS := $(wildcard *.cpp) $(wildcard io/*.cpp) $(wildcard imu/*.cpp) $(wildcard guards/*.cpp) $(wildcard drv/vesc/*.cpp) $(wildcard drv/settings/*.cpp) $(wildcard drv/mpu6050/*.cpp) $(wildcard drv/led/*.cpp) ${STM32_KIT} ${NANOPB_CORE} drv/comms/protocol.pb.c
INC:=drv cmsis_boot drv/vesc drv/comms stm_lib/inc cmsis . $(NANOPB_DIR)
INC_PARAMS=$(INC:%=-I%)

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-gcc

ARCH = -mcpu=cortex-m3 -mthumb
CFLAGS = ${ARCH} -Wall -ffunction-sections -g -O2 -flto -fno-builtin -c -DSTM32F103CB -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -DSUPPORT_CPLUSPLUS # -fstack-usage
CPPFLAGS = $(CFLAGS) -std=gnu++11

OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

# Build step for C source
${BUILD_DIR}/%.c.o : %.c ${HDRS}
	mkdir -p $(dir $@)
	${CC} $(CFLAGS) ${INC_PARAMS} -c $< -o $@

# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp ${HDRS}
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(INC_PARAMS) -c $< -o $@

# The final build step.
${BUILD_DIR}/BalancingController.elf: $(OBJS) link.ld
	$(CC) ${ARCH} -g -flto -Wl,-Map=${BUILD_DIR}/BalancingController.map -O2 -Wl,--gc-sections -Wl,--entry=main -Wl,-T./link.ld -g -o $@ $(OBJS) -lm -lgcc -lc -lstdc++

${BUILD_DIR}/BalancingController.bin: ${BUILD_DIR}/BalancingController.elf
	arm-none-eabi-objcopy -O binary ${BUILD_DIR}/BalancingController.elf $@

${BUILD_DIR}/BalancingController.hex: ${BUILD_DIR}/BalancingController.elf
	arm-none-eabi-objcopy -O ihex ${BUILD_DIR}/BalancingController.elf $@



# TODO1: Port generate bat.
# TODO2: Add program step
# TODO3: Figure out CooCox references.


.PHONY: clean program
clean:
	rm -r $(BUILD_DIR)

program: ${BUILD_DIR}/BalancingController.elf
	"C:/CooCox/CoIDE/bin\coflash.exe" program STM32F103CB $< --adapter-name=ST-Link --port=SWD --adapter-clk=2000000 --erase=affected --reset=SYSRESETREQ --driver="C:/CooCox/CoIDE/flash/STM32F10x_MD_128.elf" --verify=false 


descriptor.pb.bin: drv/comms/protocol.proto
	$(PROTOC) $< --descriptor_set_out=$@

