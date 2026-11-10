# AgriRover GH-1 base-controller firmware
#   make test    build and run the host unit tests (gcc)
#   make stm32   cross-compile for the STM32F411CE "Black Pill" (arm-none-eabi-gcc) -> build/agrirover_fw.elf/.bin
CORE = core/protocol.c core/control.c app/main.c
HOSTCC ?= gcc
CROSS ?= arm-none-eabi-
CFLAGS_COMMON = -std=c11 -Wall -Wextra -Werror -O2

build/test_core: $(CORE) tests/test_core.c core/*.h hal/hal.h
	@mkdir -p build
	$(HOSTCC) $(CFLAGS_COMMON) -DAR_HOST_BUILD -D_GNU_SOURCE -fsanitize=address,undefined -g $(CORE) tests/test_core.c -lm -o $@

test: build/test_core
	./build/test_core

STM = hal/stm32f411
STM_SRC = $(CORE) $(STM)/hal_stm32f411.c $(STM)/startup.c
STM_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -ffreestanding

stm32: build/agrirover_fw.elf
build/agrirover_fw.elf: $(STM_SRC) $(STM)/stm32f411.ld $(STM)/reg.h core/*.h
	@mkdir -p build
	$(CROSS)gcc $(CFLAGS_COMMON) $(STM_FLAGS) $(STM_SRC) -T $(STM)/stm32f411.ld -Wl,--gc-sections -Wl,-Map=build/agrirover_fw.map --specs=nano.specs --specs=nosys.specs -lm -o $@
	$(CROSS)objcopy -O binary $@ build/agrirover_fw.bin
	$(CROSS)size $@

flash: build/agrirover_fw.elf   # USB DFU: hold BOOT0, tap NRST
	dfu-util -a 0 -s 0x08000000:leave -D build/agrirover_fw.bin

clean:
	rm -rf build
.PHONY: test stm32 flash clean
