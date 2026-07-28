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