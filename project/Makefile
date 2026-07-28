TARGET = firmware
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE  = arm-none-eabi-size
RM = del /q

STDPERIPH_SRC = Drivers/STM32F10x_StdPeriph_Driver/src
CMSIS_SRC = Drivers/CMSIS/CM3/CoreSupport

SRCS = \
	Core/Src/main.c \
	Core/Src/encoder.c \
	Core/Src/ring_buffer.c \
	Core/Src/recorder.c \
	Core/Src/sd_card.c \
	Core/Src/usart_cmd.c \
	Core/Src/stm32f10x_it.c \
	Core/Src/system_stm32f10x.c \
	$(CMSIS_SRC)/core_cm3.c \
	$(STDPERIPH_SRC)/misc.c \
	$(STDPERIPH_SRC)/stm32f10x_gpio.c \
	$(STDPERIPH_SRC)/stm32f10x_rcc.c \
	$(STDPERIPH_SRC)/stm32f10x_tim.c \
	$(STDPERIPH_SRC)/stm32f10x_exti.c \
	$(STDPERIPH_SRC)/stm32f10x_usart.c \
	$(STDPERIPH_SRC)/stm32f10x_spi.c \
	$(STDPERIPH_SRC)/stm32f10x_dma.c \
	$(STDPERIPH_SRC)/stm32f10x_flash.c \
	$(STDPERIPH_SRC)/stm32f10x_iwdg.c \
	FatFS/src/ff.c \
	FatFS/src/diskio.c

OBJS = $(SRCS:.c=.o)
OBJS += Drivers/CMSIS/startup/startup_stm32f10x_md.o

INCLUDES = \
	-ICore/Inc \
	-IDrivers/CMSIS/CM3/CoreSupport \
	-IDrivers/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
	-IDrivers/STM32F10x_StdPeriph_Driver/inc \
	-IFatFS/src

CFLAGS = -mcpu=cortex-m3 -mthumb -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER \
	-Os -Wall -fdata-sections -ffunction-sections -MMD -MP $(INCLUDES)

LDFLAGS = -Tlinker.ld --specs=nano.specs -Wl,--gc-sections -Wl,-Map=$(TARGET).map

.PHONY: all clean

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	$(SIZE) $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

%.o: %.s
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

-include $(SRCS:.c=.d)

clean:
	-$(RM) $(subst /,\,$(OBJS))
	-$(RM) $(TARGET).elf $(TARGET).hex $(TARGET).map