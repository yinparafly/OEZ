# STM32F103 ABZ Encoder Recorder — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement firmware for STM32F103C8T6 to read AS5047P ABZ encoder signals, monitor RPM, trigger on RPM>10+Z, and record 200 pre + 0.8s post data to SD card as CSV.

**Architecture:** Producer-consumer model with 4kHz TIM3 interrupt writing encoder counts to a 3584-point ring buffer, and main loop state machine checking trigger conditions and writing to SD card via SPI+FatFS.

**Tech Stack:** ARM GCC 13.2.1, STM32 Standard Peripheral Library V3.4.0, FatFS R0.08, STM32F103C8T6 (Cortex-M3)

**Spec:** `docs/superpowers/specs/2026-07-28-stm32f103-abz-encoder-recorder-design.md`

## Reference Projects (for comparison & code reference)

Downloaded to `references/`:

| Repo | Stars | Description | Key Learnings Applied |
|------|-------|-------------|----------------------|
| [eziya/STM32_SPI_SDCARD](https://github.com/eziya/STM32_SPI_SDCARD) | 257 | STM32F1/F4 SD+SPI+FatFS via CubeMX/HAL | CS mgmt in caller (not SendCmd), card type & sector addressing |
| [digiexchris/SDEC](https://github.com/digiexchris/SDEC) | - | STM32F103 dedicated encoder counter + Z signal | TIM2 encoder + index self-heal, RPM from Z period |
| [mohammedyosri2002/Quadrature-Encoder-RPM](https://github.com/mohammedyosri2002/STM32-Quadrature-Encoder-RPM-Counter) | - | TIM2 encoder + TIM3 RPM sampling | Identical timer architecture to ours |
| [NimaMX/STM32F103-SDCARD](https://github.com/NimaMX/STM32F103-SDCARD) | 27 | 4-layer SPI SD on F103 (driver/middleware/library/BSP) | Clean layering for portability |
| [jaiswalprabhakar/STM32F103C8-SD-CARD-SPI-data-logger](https://github.com/jaiswalprabhakar/STM32F103C8-SD-CARD-SPI-data-logger) | - | F103 Blue Pill datalogger - SD + SPI + CSV | Same MCU + SD + CSV pattern |

## Global Constraints

- MCU: STM32F103C8T6 (64KB Flash, 20KB RAM, 72MHz)
- Toolchain: ARM GCC 13.2.1 at `D:/arm-gcc/xpack-arm-none-eabi-gcc-13.2.1-1.1/bin`
- StdPeriph library V3.4.0 (existing at `D:\oezcon\stem32 F103\核心板测试程序(PC13闪烁)\Libraries`)
- FatFS V0.08A (existing at `D:\oezcon\stem32 F103\网络收集参考例程\FATFS V0.08A-SD Card\USER\FATFS_V0.08A\src`)
- Use GCC startup file: `startup/gcc_ride7/startup_stm32f10x_md.s`
- STM32F10X_MD defined (Medium Density = 64KB flash lineage)
- 8MHz HSE crystal, PLL x9 = 72MHz SYSCLK
- 4kHz sampling via TIM3 (APB1=36MHz, prescaler=0, ARR=8999)
- Ring buffer: 3584 points × 4 bytes = 14,336 bytes
- SPI1 for SD card: PA5(SCK), PA6(MISO), PA7(MOSI), PA4(CS)
- USART1: PA9(TX), PA10(RX), 115200 baud
- IWDG: ~1.28s timeout

---

## File Structure

```
Project/
├── Core/
│   ├── Src/
│   │   ├── main.c                # State machine + LED + IWDG feed
│   │   ├── encoder.c             # TIM2 encoder mode driver
│   │   ├── ring_buffer.c         # 3584-point ring buffer
│   │   ├── recorder.c            # Trigger state machine + RPM calc
│   │   ├── sd_card.c             # SPI SD driver + FatFS diskio
│   │   ├── usart_cmd.c           # USART1 command parsing
│   │   ├── stm32f10x_it.c        # Interrupt handlers
│   │   └── system_stm32f10x.c    # System clock init (72MHz)
│   └── Inc/
│       ├── main.h
│       ├── encoder.h
│       ├── ring_buffer.h
│       ├── recorder.h
│       ├── sd_card.h
│       ├── usart_cmd.h
│       └── stm32f10x_it.h
├── Drivers/
│   ├── CMSIS/
│   │   ├── CM3/
│   │   │   ├── CoreSupport/
│   │   │   │   ├── core_cm3.c
│   │   │   │   └── core_cm3.h
│   │   │   └── DeviceSupport/ST/STM32F10x/
│   │   │       ├── stm32f10x.h
│   │   │       └── system_stm32f10x.c (see Src/ above)
│   │   └── startup/
│   │       └── startup_stm32f10x_md.s  # GCC startup file
│   └── STM32F10x_StdPeriph_Driver/
│       ├── inc/           # All StdPeriph headers
│       └── src/           # All StdPeriph sources
├── FatFS/
│   ├── src/
│   │   ├── ff.h / ff.c
│   │   ├── ffconf.h
│   │   ├── diskio.h / diskio.c  # SPI SD adapter
│   │   └── integer.h
│   └── ...
├── linker.ld              # STM32F103C8T6 linker script
└── Makefile               # Build system
```

---

### Task 1: Project Scaffolding

**Files:**
- Create: `linker.ld`
- Create: `Makefile`
- Copy: `Drivers/CMSIS/CM3/CoreSupport/core_cm3.c`
- Copy: `Drivers/CMSIS/CM3/CoreSupport/core_cm3.h`
- Copy: `Drivers/CMSIS/CM3/DeviceSupport/ST/STM32F10x/stm32f10x.h`
- Copy: `Drivers/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c`
- Copy: `Drivers/startup/startup_stm32f10x_md.s`
- Copy: `Drivers/STM32F10x_StdPeriph_Driver/inc/*`
- Copy: `Drivers/STM32F10x_StdPeriph_Driver/src/*`
- Create: `Core/Inc/` and `Core/Src/` directories

**Interfaces:**
- Consumes: (none - this is the foundation)
- Produces: Working compile chain that can produce a minimal .hex

- [ ] **Step 1: Create project directory tree**

```bash
$project = "D:\oezcon\project"
New-Item -ItemType Directory -Path "$project\Core\Inc" -Force
New-Item -ItemType Directory -Path "$project\Core\Src" -Force
New-Item -ItemType Directory -Path "$project\Drivers\CMSIS\CM3\CoreSupport" -Force
New-Item -ItemType Directory -Path "$project\Drivers\CMSIS\CM3\DeviceSupport\ST\STM32F10x" -Force
New-Item -ItemType Directory -Path "$project\Drivers\CMSIS\startup" -Force
New-Item -ItemType Directory -Path "$project\Drivers\STM32F10x_StdPeriph_Driver\inc" -Force
New-Item -ItemType Directory -Path "$project\Drivers\STM32F10x_StdPeriph_Driver\src" -Force
New-Item -ItemType Directory -Path "$project\FatFS\src" -Force
```

- [ ] **Step 2: Copy CMSIS and StdPeriph files from existing project**

```bash
$src = "D:\oezcon\stem32 F103\核心板测试程序(PC13闪烁)\Libraries"
$dst = "D:\oezcon\project\Drivers"
Copy-Item "$src\CMSIS\CM3\CoreSupport\core_cm3.c" "$dst\CMSIS\CM3\CoreSupport\"
Copy-Item "$src\CMSIS\CM3\CoreSupport\core_cm3.h" "$dst\CMSIS\CM3\CoreSupport\"
Copy-Item "$src\CMSIS\CM3\DeviceSupport\ST\STM32F10x\stm32f10x.h" "$dst\CMSIS\CM3\DeviceSupport\ST\STM32F10x\"
Copy-Item "$src\CMSIS\CM3\DeviceSupport\ST\STM32F10x\system_stm32f10x.c" "$dst\CMSIS\CM3\DeviceSupport\ST\STM32F10x\"
Copy-Item "$src\CMSIS\CM3\DeviceSupport\ST\STM32F10x\startup\gcc_ride7\startup_stm32f10x_md.s" "$dst\CMSIS\startup\"
Copy-Item "$src\STM32F10x_StdPeriph_Driver\inc\*" "$dst\STM32F10x_StdPeriph_Driver\inc\"
Copy-Item "$src\STM32F10x_StdPeriph_Driver\src\*" "$dst\STM32F10x_StdPeriph_Driver\src\"
```

- [ ] **Step 3: Create linker.ld**

```ld
/* STM32F103C8T6 - 64KB Flash, 20KB RAM */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)); . = ALIGN(4); } > FLASH
    .text       : { . = ALIGN(4); *(.text*); *(.rodata*); . = ALIGN(4); } > FLASH
    .preinit_array : { . = ALIGN(4); KEEP(*(.preinit_array*)); . = ALIGN(4); } > FLASH
    .init_array   : { . = ALIGN(4); KEEP(*(SORT(.init_array*))); . = ALIGN(4); } > FLASH
    .fini_array   : { . = ALIGN(4); KEEP(*(SORT(.fini_array*))); . = ALIGN(4); } > FLASH
    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data*); _edata = .; . = ALIGN(4); } > RAM AT > FLASH
    .bss  : { . = ALIGN(4); _sbss = .; *(.bss*); *(COMMON); _ebss = .; . = ALIGN(4); } > RAM
    /DISCARD/ : { libc.a (*) ; libm.a (*) ; libgcc.a (*) ; }
}
```

- [ ] **Step 4: Create Makefile**

```makefile
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
	-Os -Wall -fdata-sections -ffunction-sections $(INCLUDES)

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

clean:
	-$(RM) $(subst /,\,$(OBJS))
	-$(RM) $(TARGET).elf $(TARGET).hex $(TARGET).map
```

---

### Task 2: System Clock + LED Blink Verification

**Files:**
- Copy: `Core/Src/system_stm32f10x.c` (from Drivers/CMSIS)
- Create: `Core/Inc/main.h`
- Create: `Core/Src/main.c` (minimal - LED blink only)
- Modify: `Makefile` (add system_stm32f10x.c to SRCS - already done in Task 1)

**Interfaces:**
- Consumes: Task 1 (linker.ld, Makefile, drivers)
- Produces: working hex with LED blink at 72MHz

- [ ] **Step 1: Create main.h**

```c
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#define LED_PIN       GPIO_Pin_13
#define LED_PORT      GPIOC
#define LED_ON()      GPIO_ResetBits(LED_PORT, LED_PIN)
#define LED_OFF()     GPIO_SetBits(LED_PORT, LED_PIN)
#define LED_TOGGLE()  (LED_PORT->ODR ^= LED_PIN)

void Delay(uint32_t ms);

#endif
```

- [ ] **Step 2: Create minimal main.c**

```c
#include "main.h"

static void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    LED_ON();
}

void Delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) { __NOP(); }
}

int main(void) {
    GPIO_Init();
    while (1) {
        LED_TOGGLE();
        Delay(500);
    }
}
```

- [ ] **Step 3: Build and verify**

```bash
cd D:\oezcon\project
& "D:\arm-gcc\xpack-arm-none-eabi-gcc-13.2.1-1.1\bin\arm-none-eabi-gcc.exe" --version
make clean; make all 2>&1
```

Expected: No errors. `firmware.hex` produced. `firmware.map` shows sizes.

- [ ] **Step 4: Copy system_stm32f10x.c from CMSIS to Core/Src**

```bash
Copy-Item "D:\oezcon\project\Drivers\CMSIS\CM3\DeviceSupport\ST\STM32F10x\system_stm32f10x.c" "D:\oezcon\project\Core\Src\system_stm32f10x.c"
```

Verify clock init uses SYSCLK_FREQ_72MHz (defined in system_stm32f10x.c line 83).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: project scaffolding with Makefile + LED blink test"
```

---

### Task 3: Encoder Driver (TIM2 Encoder Mode)

**Files:**
- Create: `Core/Inc/encoder.h`
- Create: `Core/Src/encoder.c`
- Modify: `Core/Src/main.c` (add encoder test code)
- Modify: `Makefile` (add encoder.c)

**Interfaces:**
- Consumes: stm32f10x.h, stm32f10x_tim.h, stm32f10x_gpio.h
- Produces: `Encoder_Init()`, `Encoder_GetCount() → uint32_t`, `Encoder_GetDirection() → int`

- [ ] **Step 1: Create encoder.h**

```c
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* AS5047P default: 1000 PPR, TIM2 encoder mode TI1+TI2 = 4000 counts/rev */
#define ENCODER_PPR         1000
#define ENCODER_STEPS_PER_REV  4000  /* 1000 * 4 (TI1+TI2) */

void Encoder_Init(void);
uint32_t Encoder_GetCount(void);
int Encoder_GetDirection(void); /* 1 = forward, -1 = reverse */

#endif
```

- [ ] **Step 2: Create encoder.c**

```c
#include "encoder.h"

void Encoder_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM2, ENABLE);
}

uint32_t Encoder_GetCount(void) {
    return TIM2->CNT;
}

int Encoder_GetDirection(void) {
    return (TIM2->CR1 & TIM_CR1_DIR) ? -1 : 1;
}
```

- [ ] **Step 3: Add encoder test to main.c (temporary)**

```c
#include "encoder.h"

// In main(), after GPIO_Init():
    Encoder_Init();
    USART_Init(115200); // from task 4

    while (1) {
        uint32_t cnt = Encoder_GetCount();
        printf("CNT: %lu DIR: %d\n", cnt, Encoder_GetDirection());
        Delay(100);
    }
```

- [ ] **Step 4: Build**

```bash
make clean; make all 2>&1
```

Expected: No errors.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: encoder driver with TIM2 encoder mode"
```

---

### Task 4: USART1 Command + TIM3 4kHz Sampling + Ring Buffer

**Files:**
- Create: `Core/Inc/usart_cmd.h`
- Create: `Core/Src/usart_cmd.c`
- Create: `Core/Inc/ring_buffer.h`
- Create: `Core/Src/ring_buffer.c`
- Modify: `Core/Src/main.c` (integrate sampling)
- Modify: `Core/Src/stm32f10x_it.c` (TIM3 + USART1 IRQ handlers)
- Modify: `Core/Inc/stm32f10x_it.h`
- Modify: `Makefile`

**Interfaces:**
- Consumes: Encoder_GetCount()
- Produces: `USART_Init(uint32_t baud)`, `Cmd_GetStartFlag()`, `RingBuffer*` functions

- [ ] **Step 1: Create usart_cmd.h**

```c
#ifndef __USART_CMD_H
#define __USART_CMD_H

#include "stm32f10x.h"
#include <stdbool.h>

#define CMD_BUF_SIZE    16
#define CMD_START_CHAR  'G'

void USART_Init(uint32_t baud);
bool Cmd_GetStartFlag(void);
void Cmd_ClearStartFlag(void);

#endif
```

- [ ] **Step 2: Create usart_cmd.c**

```c
#include "usart_cmd.h"

static volatile bool start_flag = false;

void USART_Init(uint32_t baud) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    /* Configure NVIC for USART1 */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

bool Cmd_GetStartFlag(void) {
    return start_flag;
}

void Cmd_ClearStartFlag(void) {
    start_flag = false;
}

/* Called from USART1_IRQHandler in stm32f10x_it.c */
void Cmd_ProcessChar(uint8_t c) {
    if (c == CMD_START_CHAR) {
        start_flag = true;
    }
}
```

- [ ] **Step 3: Create ring_buffer.h**

```c
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "stm32f10x.h"
#include <stdbool.h>

#define RING_BUFFER_SIZE    3584
#define RING_BUFFER_MASK    (RING_BUFFER_SIZE - 1)

typedef struct {
    uint32_t data[RING_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} RingBuffer_t;

extern RingBuffer_t g_buffer;

void Buffer_Init(void);
bool Buffer_Write(uint32_t value);
uint32_t Buffer_Read(uint32_t offset);
uint32_t Buffer_GetCount(void);
uint32_t Buffer_GetHead(void);

#endif
```

- [ ] **Step 4: Create ring_buffer.c**

```c
#include "ring_buffer.h"

RingBuffer_t g_buffer;

void Buffer_Init(void) {
    g_buffer.head = 0;
    g_buffer.tail = 0;
    g_buffer.count = 0;
}

bool Buffer_Write(uint32_t value) {
    uint32_t next = (g_buffer.head + 1) & RING_BUFFER_MASK;
    if (next == g_buffer.tail) {
        g_buffer.tail = (g_buffer.tail + 1) & RING_BUFFER_MASK;
        if (g_buffer.count > 0) g_buffer.count--;
    }
    g_buffer.data[g_buffer.head] = value;
    g_buffer.head = next;
    g_buffer.count++;
    return true;
}

uint32_t Buffer_Read(uint32_t offset) {
    uint32_t pos = (g_buffer.tail + offset) & RING_BUFFER_MASK;
    return g_buffer.data[pos];
}

uint32_t Buffer_GetCount(void) {
    return g_buffer.count;
}

uint32_t Buffer_GetHead(void) {
    return g_buffer.head;
}
```

- [ ] **Step 5: Create stm32f10x_it.h**

```c
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void TIM3_IRQHandler(void);
void EXTI0_IRQHandler(void);
void USART1_IRQHandler(void);

#endif
```

- [ ] **Step 6: Create stm32f10x_it.c with TIM3 and USART1 handlers**

```c
#include "stm32f10x_it.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "usart_cmd.h"
#include "recorder.h"  /* forward declare: will contain trigger check */

/* Global vars for RPM calculation in ISR */
volatile uint32_t g_last_count = 0;
volatile uint32_t g_last_time = 0;
volatile float g_current_rpm = 0.0f;
volatile bool g_z_signal_occurred = false;

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1); }
void MemManage_Handler(void) { while (1); }
void BusFault_Handler(void) { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        uint32_t count = Encoder_GetCount();
        Buffer_Write(count);
        /* Trigger check - in recorder.c */
        Recorder_ISR_Check(count);
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
        uint8_t c = USART_ReceiveData(USART1);
        Cmd_ProcessChar(c);
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0)) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        g_z_signal_occurred = true;
    }
}
```

- [ ] **Step 7: Add TIM3 init + IRQ handlers to main.c**

```c
#include "ring_buffer.h"
#include "usart_cmd.h"
#include "recorder.h"

static void TIM3_Sampling_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 8999;  /* 36MHz / 9000 = 4kHz */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

// In main():
    Buffer_Init();
    TIM3_Sampling_Init();
    USART_Init(115200);
    // main loop prints count every second
    while (1) {
        printf("BUF: %lu\n", Buffer_GetCount());
        Delay(1000);
    }
```

- [ ] **Step 8: Build**

```bash
make clean; make all 2>&1
```

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "feat: USART1, ring buffer, and TIM3 4kHz sampling"
```

---

### Task 5: Z Interrupt + Recorder State Machine + RPM Logic

**Files:**
- Create: `Core/Inc/recorder.h`
- Create: `Core/Src/recorder.c`
- Modify: `Core/Src/main.c` (state machine loop)
- Modify: `Core/Src/stm32f10x_it.c` (EXTI0 handler already added)
- Modify: `Makefile`

**Interfaces:**
- Consumes: `Buffer_Write()`, `Buffer_GetHead()`, `Encoder_GetCount()`
- Produces: `Recorder_Init()`, `Recorder_ISR_Check(count)`, `Recorder_MainLoop(uint32_t now_ms)`, `Recorder_IsTriggered()`

- [ ] **Step 1: Create recorder.h**

```c
#ifndef __RECORDER_H
#define __RECORDER_H

#include "stm32f10x.h"
#include <stdbool.h>

/* Trigger thresholds */
#define RPM_TRIGGER_HIGH    12
#define RPM_TRIGGER_LOW     8

/* Recording parameters */
#define PRE_TRIGGER_SAMPLES     200
#define POST_TRIGGER_MS         800    /* 0.8 seconds */
#define POST_TRIGGER_SAMPLES    (POST_TRIGGER_MS * 4)  /* 4kHz => 3200 */
#define TOTAL_RECORD_SAMPLES    (PRE_TRIGGER_SAMPLES + POST_TRIGGER_SAMPLES)

/* States */
typedef enum {
    RECORDER_IDLE,
    RECORDER_MONITOR,
    RECORDER_TRIGGERED,
    RECORDER_WRITING,
    RECORDER_DONE
} RecorderState_t;

/* LED modes */
typedef enum {
    LED_SLOW_BLINK = 0,
    LED_ON,
    LED_FAST_BLINK,
    LED_OFF,
    LED_BLINK_3
} LEDMode_t;

void Recorder_Init(void);
void Recorder_ISR_Check(uint32_t count);
void Recorder_MainLoop(uint32_t now_ms);
bool Recorder_IsTriggered(void);
RecorderState_t Recorder_GetState(void);
void Recorder_StartMonitor(void);
void Record_SaveToSD(void);

/* External vars from stm32f10x_it.c */
extern volatile bool g_z_signal_occurred;
extern volatile uint32_t g_last_count;
extern volatile uint32_t g_last_time;

#endif
```

- [ ] **Step 2: Create recorder.c**

```c
#include "recorder.h"
#include "ring_buffer.h"
#include "encoder.h"
#include "stm32f10x_it.h"

static RecorderState_t state = RECORDER_IDLE;
static bool rpm_ok = false;
static bool record_ready = false;
static uint32_t trigger_index = 0;
static uint32_t trigger_start_ms = 0;
static uint32_t last_rpm_check_count = 0;
static uint32_t last_rpm_check_time = 0;

/* RPM window calculation (called from main loop every ~100ms) */
static float CalcRPM(uint32_t now_ms) {
    uint32_t dt = now_ms - last_rpm_check_time;
    if (dt < 50) return 0.0f;
    uint32_t current_count = Encoder_GetCount();
    int32_t delta = (int32_t)(current_count - last_rpm_check_count);
    if (delta < 0) delta += 4096;  /* assume forward, handle wrap */
    float rpm = (float)delta / 4000.0f / ((float)dt / 60000.0f);
    last_rpm_check_count = current_count;
    last_rpm_check_time = now_ms;
    return rpm;
}

void Recorder_Init(void) {
    state = RECORDER_IDLE;
    rpm_ok = false;
    record_ready = false;
    trigger_index = 0;
    g_z_signal_occurred = false;
}

void Recorder_ISR_Check(uint32_t count) {
    if (state != RECORDER_MONITOR && state != RECORDER_TRIGGERED) return;
    if (state == RECORDER_MONITOR) {
        /* Trigger check: RPM high + Z pulse seen */
        if (rpm_ok && g_z_signal_occurred) {
            state = RECORDER_TRIGGERED;
            trigger_index = Buffer_GetHead();
            g_z_signal_occurred = false;
        }
    }
}

void Recorder_MainLoop(uint32_t now_ms) {
    switch (state) {
        case RECORDER_IDLE:
            /* Wait for start command (handled in main.c) */
            break;

        case RECORDER_MONITOR: {
            float rpm = CalcRPM(now_ms);
            if (rpm > RPM_TRIGGER_HIGH) rpm_ok = true;
            if (rpm < RPM_TRIGGER_LOW)  rpm_ok = false;
            break;
        }

        case RECORDER_TRIGGERED: {
            if (trigger_start_ms == 0) trigger_start_ms = now_ms;
            uint32_t elapsed = now_ms - trigger_start_ms;
            if (elapsed >= POST_TRIGGER_MS) {
                state = RECORDER_WRITING;
                record_ready = true;
            }
            break;
        }

        case RECORDER_WRITING:
            /* Handled in main.c - call Record_SaveToSD() */
            break;

        case RECORDER_DONE:
            record_ready = false;
            state = RECORDER_MONITOR;
            break;
    }
}

bool Recorder_IsTriggered(void) {
    return record_ready;
}

RecorderState_t Recorder_GetState(void) {
    return state;
}

/* Called from main.c to start monitoring */
void Recorder_StartMonitor(void) {
    state = RECORDER_MONITOR;
    rpm_ok = false;
    trigger_start_ms = 0;
    last_rpm_check_count = Encoder_GetCount();
    last_rpm_check_time = 0;
}
```

- [ ] **Step 3: Add EXTI0 Z signal init to main.c**

```c
static void EXTI_Z_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
```

- [ ] **Step 4: Update main() state machine**

```c
int main(void) {
    GPIO_Init();
    Encoder_Init();
    Buffer_Init();
    TIM3_Sampling_Init();
    USART_Init(115200);
    EXTI_Z_Init();
    Recorder_Init();

    while (1) {
        uint32_t now = 0; /* use TIM2 or SysTick for ms */
        Recorder_MainLoop(now);

        if (Recorder_GetState() == RECORDER_IDLE && Cmd_GetStartFlag()) {
            Cmd_ClearStartFlag();
            Recorder_StartMonitor();
        }

        if (Recorder_IsTriggered()) {
            /* will be filled in Task 6 */
        }

        IWDG_ReloadCounter(); /* feed watchdog - initialized in Task 7 */
        Delay(1);
    }
}
```

- [ ] **Step 5: Build**

```bash
make clean; make all 2>&1
```

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: recorder state machine with Z interrupt and RPM logic"
```

---

### Task 6: SPI SD Card + FatFS Integration

**Files:**
- Copy: `FatFS/src/ff.c`, `FatFS/src/ff.h`, `FatFS/src/ffconf.h`, `FatFS/src/integer.h`, `FatFS/src/diskio.h` (from reference examples)
- Create: `FatFS/src/diskio.c` (SPI adapter, replacing SDIO)
- Create: `Core/Inc/sd_card.h`
- Create: `Core/Src/sd_card.c`
- Modify: `Makefile`

**Interfaces:**
- Consumes: `SPI_ReadWriteByte(uint8_t) → uint8_t`
- Produces: `SD_Init() → bool`, `SD_ReadSector(uint32_t, uint8_t*) → bool`, `SD_WriteSector(uint32_t, const uint8_t*) → bool`

- [ ] **Step 1: Create sd_card.h**

```c
#ifndef __SD_CARD_H
#define __SD_CARD_H

#include "stm32f10x.h"
#include <stdbool.h>

#define SD_CS_PORT      GPIOA
#define SD_CS_PIN       GPIO_Pin_4
#define SD_CS_LOW()     GPIO_ResetBits(SD_CS_PORT, SD_CS_PIN)
#define SD_CS_HIGH()    GPIO_SetBits(SD_CS_PORT, SD_CS_PIN)

bool SD_Init(void);
bool SD_ReadSector(uint32_t sector, uint8_t* buffer);
bool SD_WriteSector(uint32_t sector, const uint8_t* buffer);
bool SD_GetStatus(void);

#endif
```

- [ ] **Step 2: Create sd_card.c with SPI driver**

```c
#include "sd_card.h"

/* SPI1: PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS */
static void SPI_Init(void) {
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SD_CS_PORT, &GPIO_InitStructure);
    SD_CS_HIGH();

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI_ReadWriteByte(uint8_t data) {
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1);
}

/* SPI SD card command */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    SD_CS_LOW();
    uint8_t frame[6] = { (uint8_t)(0x40 | cmd), (uint8_t)(arg >> 24),
                         (uint8_t)(arg >> 16), (uint8_t)(arg >> 8),
                         (uint8_t)(arg), crc };
    for (int i = 0; i < 6; i++) SPI_ReadWriteByte(frame[i]);
    uint8_t resp;
    for (int i = 0; i < 64; i++) {
        resp = SPI_ReadWriteByte(0xFF);
        if (!(resp & 0x80)) break;
    }
    SD_CS_HIGH();
    return resp;
}

bool SD_Init(void) {
    SPI_Init();
    /* Send 80 clocks */
    SD_CS_HIGH();
    for (int i = 0; i < 10; i++) SPI_ReadWriteByte(0xFF);

    /* CMD0: go idle */
    if (SD_SendCmd(0, 0, 0x95) != 0x01) return false;

    /* CMD8: check SDHC */
    uint8_t r1 = SD_SendCmd(8, 0x1AA, 0x87);
    if (r1 == 0x01) {
        /* SDHC - complete init */
        for (int i = 0; i < 100; i++) {
            if (SD_SendCmd(55, 0, 0x01) == 0x01 &&
                SD_SendCmd(41, 0x40000000, 0x01) == 0x00) break;
        }
    }
    /* CMD16: set block size */
    SD_SendCmd(16, 512, 0x01);
    return true;
}

bool SD_ReadSector(uint32_t sector, uint8_t* buffer) {
    /* CMD17: read single block */
    SD_CS_LOW();
    uint8_t frame[6] = { 0x51, (uint8_t)(sector >> 24), (uint8_t)(sector >> 16),
                         (uint8_t)(sector >> 8), (uint8_t)(sector), 0x01 };
    for (int i = 0; i < 6; i++) SPI_ReadWriteByte(frame[i]);
    uint8_t resp;
    for (int i = 0; i < 64; i++) {
        resp = SPI_ReadWriteByte(0xFF);
        if (resp == 0xFE) break;
    }
    if (resp != 0xFE) { SD_CS_HIGH(); return false; }
    for (int i = 0; i < 512; i++) buffer[i] = SPI_ReadWriteByte(0xFF);
    SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); /* CRC */
    SD_CS_HIGH();
    return true;
}

bool SD_WriteSector(uint32_t sector, const uint8_t* buffer) {
    SD_CS_LOW();
    uint8_t frame[6] = { 0x58, (uint8_t)(sector >> 24), (uint8_t)(sector >> 16),
                         (uint8_t)(sector >> 8), (uint8_t)(sector), 0x01 };
    for (int i = 0; i < 6; i++) SPI_ReadWriteByte(frame[i]);
    uint8_t resp;
    for (int i = 0; i < 64; i++) {
        resp = SPI_ReadWriteByte(0xFF);
        if (!(resp & 0x80)) break;
    }
    if (resp != 0x00) { SD_CS_HIGH(); return false; }
    SPI_ReadWriteByte(0xFE); /* start block */
    for (int i = 0; i < 512; i++) SPI_ReadWriteByte(buffer[i]);
    SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); /* CRC */
    /* Wait for write complete */
    for (int i = 0; i < 65535; i++) {
        if (SPI_ReadWriteByte(0xFF) == 0xFF) break;
    }
    SD_CS_HIGH();
    return true;
}

bool SD_GetStatus(void) {
    return true;
}
```

- [ ] **Step 3: Copy FatFS source files from reference**

```bash
$fatfs_src = "D:\oezcon\stem32 F103\网络收集参考例程\FATFS V0.08A-SD Card\USER\FATFS_V0.08A\src"
$fatfs_dst = "D:\oezcon\project\FatFS\src"
Copy-Item "$fatfs_src\ff.h" "$fatfs_dst\"
Copy-Item "$fatfs_src\ff.c" "$fatfs_dst\"
Copy-Item "$fatfs_src\ffconf.h" "$fatfs_dst\"
Copy-Item "$fatfs_src\integer.h" "$fatfs_dst\"
Copy-Item "$fatfs_src\diskio.h" "$fatfs_dst\"
```

- [ ] **Step 4: Create diskio.c (SPI adapter for FatFS)**

```c
#include "ff.h"
#include "diskio.h"
#include "sd_card.h"

DSTATUS disk_initialize(BYTE drv) {
    if (drv) return STA_NOINIT;
    return SD_Init() ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE drv) {
    if (drv) return STA_NOINIT;
    return 0;
}

DRESULT disk_read(BYTE drv, BYTE* buf, DWORD sector, BYTE count) {
    if (drv) return RES_ERROR;
    for (BYTE i = 0; i < count; i++) {
        if (!SD_ReadSector(sector + i, buf + (i * 512)))
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE* buf, DWORD sector, BYTE count) {
    if (drv) return RES_ERROR;
    for (BYTE i = 0; i < count; i++) {
        if (!SD_WriteSector(sector + i, buf + (i * 512)))
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void* buff) {
    if (drv) return RES_ERROR;
    switch (ctrl) {
        case CTRL_SYNC: return RES_OK;
        case GET_SECTOR_COUNT: *(DWORD*)buff = 15630336; return RES_OK; /* 8GB */
        case GET_SECTOR_SIZE: *(WORD*)buff = 512; return RES_OK;
        case GET_BLOCK_SIZE: *(DWORD*)buff = 1; return RES_OK;
    }
    return RES_PARERR;
}

DWORD get_fattime(void) {
    return ((20UL << 25) | (7UL << 21) | (28UL << 16) | (12UL << 11));
}
```

- [ ] **Step 5: Build and verify**

```bash
make clean; make all 2>&1
```

Expected: No errors.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: SPI SD card driver and FatFS integration"
```

---

### Task 7: Complete Data Saving + File Counter + IWDG

**Files:**
- Modify: `Core/Src/recorder.c` (add Record_SaveToSD function)
- Modify: `Core/Src/sd_card.c` (add file counter support)
- Modify: `Core/Src/main.c` (full integration)
- Modify: `Core/Src/stm32f10x_it.c` (ensure Z and TIM3 are correct)

**Interfaces:**
- Consumes: `RingBuffer`, `SD_Init()`, `f_open/f_write/f_close`, `Buffer_Read()`
- Produces: CSV files on SD card

- [ ] **Step 1: Add save function to recorder.c**

```c
#include "ff.h"
#include "string.h"

#define CRC32_POLY 0xEDB88320

static uint32_t CalcCRC32(const uint32_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t val = data[i];
        for (int b = 0; b < 32; b++) {
            uint32_t bit = (val ^ crc) & 1;
            crc >>= 1;
            if (bit) crc ^= CRC32_POLY;
            val >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

static uint32_t ReadFileCounter(void) {
    FIL fil;
    uint32_t counter = 1;
    if (f_open(&fil, "0:INDEX.TXT", FA_READ) == FR_OK) {
        char buf[16];
        UINT br;
        f_read(&fil, buf, sizeof(buf) - 1, &br);
        buf[br] = 0;
        counter = atol(buf) + 1;
        f_close(&fil);
    }
    return counter;
}

static void WriteFileCounter(uint32_t counter) {
    FIL fil;
    if (f_open(&fil, "0:INDEX.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        char buf[16];
        sprintf(buf, "%lu", counter);
        UINT bw;
        f_write(&fil, buf, strlen(buf), &bw);
        f_close(&fil);
    }
}

void Record_SaveToSD(void) {
    /* Verify buffer has enough data */
    if (g_buffer.count < TOTAL_RECORD_SAMPLES) {
        state = RECORDER_MONITOR;
        return;
    }

    /* Disable TIM3 interrupt during data extraction */
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

    /* Read all samples into a local array */
    uint32_t samples[TOTAL_RECORD_SAMPLES];
    uint32_t tail_at_trigger = g_buffer.tail;
    uint32_t start = (trigger_index - PRE_TRIGGER_SAMPLES) & RING_BUFFER_MASK;
    for (uint32_t i = 0; i < TOTAL_RECORD_SAMPLES; i++) {
        uint32_t pos = (start + i) & RING_BUFFER_MASK;
        uint32_t idx = (pos - tail_at_trigger) & RING_BUFFER_MASK;
        samples[i] = g_buffer.data[pos];
    }

    /* Re-enable TIM3 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* Mount filesystem */
    FATFS fs;
    if (f_mount(0, &fs) != FR_OK) { state = RECORDER_MONITOR; return; }

    /* Get file counter */
    uint32_t counter = ReadFileCounter();

    /* Create CSV file */
    FIL fil;
    char filename[32];
    sprintf(filename, "0:CAT_%05lu.CSV", counter);
    if (f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE) != FR_OK) {
        state = RECORDER_MONITOR;
        return;
    }

    /* Write CSV header */
    f_printf(&fil, "index,count\n");

    /* Write data rows */
    for (uint32_t i = 0; i < TOTAL_RECORD_SAMPLES; i++) {
        f_printf(&fil, "%lu,%lu\n", i, samples[i]);
    }

    /* Write CRC32 checksum */
    uint32_t crc = CalcCRC32(samples, TOTAL_RECORD_SAMPLES);
    f_printf(&fil, "# CHECKSUM: %08lX\n", crc);

    f_close(&fil);
    WriteFileCounter(counter);

    record_ready = false;
    state = RECORDER_DONE;
}
```

- [ ] **Step 2: Update main.c full integration**

```c
#include "main.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "usart_cmd.h"
#include "recorder.h"
#include "sd_card.h"

/* Static function declarations */
static void GPIO_Init(void);
static void TIM3_Sampling_Init(void);
static void EXTI_Z_Init(void);
static void IWDG_Init(void);
static void LED_Update(RecorderState_t st, uint32_t now_ms);

static uint32_t g_tick_ms = 0;

void Delay(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) { __NOP(); }
}

/* SysTick interrupt every 1ms */
volatile uint32_t g_tick_ms = 0;
void SysTick_Handler(void) { g_tick_ms++; }

int main(void) {
    GPIO_Init();
    Encoder_Init();
    Buffer_Init();
    TIM3_Sampling_Init();
    USART_Init(115200);
    EXTI_Z_Init();
    Recorder_Init();
    SD_Init();
    IWDG_Init();
    SysTick_Config(SystemCoreClock / 1000);  /* 1ms tick */

    while (1) {
        uint32_t now = g_tick_ms;
        IWDG_ReloadCounter();

        Recorder_MainLoop(now);

        if (Recorder_GetState() == RECORDER_IDLE && Cmd_GetStartFlag()) {
            Cmd_ClearStartFlag();
            Recorder_StartMonitor();
        }

        if (Recorder_IsTriggered()) {
            Record_SaveToSD();
        }

        LED_Update(Recorder_GetState(), now);
    }
}

static void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    LED_ON();
}

static void TIM3_Sampling_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 8999;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, ENABLE);
}

static void EXTI_Z_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void IWDG_Init(void) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);  /* 40kHz / 256 = ~156Hz */
    IWDG_SetReload(200);                    /* 200 / 156 = ~1.28s */
    IWDG_Enable();
}

static void LED_Update(RecorderState_t st, uint32_t now_ms) {
    static uint32_t last_toggle = 0;
    uint32_t period = 0;
    switch (st) {
        case RECORDER_IDLE:    period = 500; break;
        case RECORDER_MONITOR: period = 0; break; /* on */
        case RECORDER_TRIGGERED: period = 50; break;
        case RECORDER_WRITING: period = 0xFFFFFFFF; break; /* off */
        case RECORDER_DONE:    period = 500; break;
    }
    if (period == 0) { LED_ON(); }
    else if (period == 0xFFFFFFFF) { LED_OFF(); }
    else if ((now_ms - last_toggle) >= period) {
        LED_TOGGLE();
        last_toggle = now_ms;
    }
}
```

- [ ] **Step 3: Update Makefile to include all source files**

Ensure `Makefile` SRCS includes:
- `Core/Src/recorder.c`
- `Core/Src/sd_card.c`
- `Core/Src/usart_cmd.c`
- `Core/Src/encoder.c`
- `Core/Src/ring_buffer.c`
- `FatFS/src/ff.c`
- `FatFS/src/diskio.c`

Also add linker flags: `-u _printf_float` if using `f_printf` (FatFS has its own printf, so likely not needed).

- [ ] **Step 4: Update Makefile to include new sources**

```makefile
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
```

- [ ] **Step 5: Build**

```bash
make clean; make all 2>&1
```

Expected: No errors. firmware.hex produced.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: full integration with SD save, file counter, IWDG"
```

---

### Task 8: Verify and Document

**Files:**
- Verify: Full build
- Verify: Map file for RAM usage

- [ ] **Step 1: Verify RAM usage from map file**

```bash
# Check firmware.map for total RAM usage
# Should show < 20KB
```

- [ ] **Step 2: Final build**

```bash
make clean; make all 2>&1
```

- [ ] **Step 3: Commit final**

```bash
git add -A
git commit -m "docs: final build verification"
```
