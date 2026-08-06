#*****************************************************************************
# Makefile for abi_monitor_h743 (STM32H743, GNU ARM toolchain + OpenOCD)
# Build:    make
# Flash:    make flash   (ST-Link V2 via OpenOCD)
# Clean:    make clean
#*****************************************************************************

GCC_PREFIX = D:/arm-official/arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi/bin/arm-none-eabi-
OPENOCD    = D:/openocd/xpack-openocd-0.12.0-7/bin/openocd.exe
OPENOCD_SCRIPTS = D:/openocd/xpack-openocd-0.12.0-7/openocd/scripts

CC      = $(GCC_PREFIX)gcc
AS      = $(GCC_PREFIX)gcc -x assembler-with-cpp
LD      = $(GCC_PREFIX)gcc
OBJCOPY = $(GCC_PREFIX)objcopy
SIZE    = $(GCC_PREFIX)size

TARGET   = abi_monitor_h743
BUILD    = build

CPU      = -mcpu=cortex-m7 -mthumb
FPU      = -mfpu=fpv5-d16 -mfloat-abi=hard
OPT      = -Og -g3
DEFS     = -DUSE_HAL_DRIVER -DSTM32H743xx
CFLAGS   = $(CPU) $(FPU) $(OPT) $(DEFS) -Wall -fdata-sections -ffunction-sections -MMD -MP
LDFLAGS  = $(CPU) $(FPU) -T gcc/stm32h743xx_flash.ld --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

INCLUDES = -IApp -ICore/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc \
           -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
           -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include \
           -IDrivers/User/Inc

C_SOURCES = \
  Core/Src/main.c \
  Core/Src/stm32h7xx_it.c \
  Core/Src/stm32h7xx_hal_msp.c \
  Core/Src/system_stm32h7xx.c \
  App/app_config.c \
  App/app_cli.c \
  App/app_pwm.c \
  App/abi.c \
  Drivers/User/Src/led.c \
  Drivers/User/Src/usart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c

ASM_SOURCES = gcc/startup_stm32h743xx.s

OBJS = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o)))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: Core/Src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: Core/Inc/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: App/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: Drivers/User/Src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: Drivers/STM32H7xx_HAL_Driver/Src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: gcc/%.s | $(BUILD)
	$(AS) $(CFLAGS) -c $< -o $@

$(BUILD):
	mkdir $(BUILD)

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS)

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(BUILD)/$(TARGET).bin
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f interface/stlink.cfg -f target/stm32h7x.cfg \
	  -c "transport select swd; adapter speed 4000; program $(BUILD)/$(TARGET).bin 0x08000000 verify reset exit"

clean:
	-rm -rf $(BUILD)

-include $(OBJS:.o=.d)

.PHONY: all clean flash
