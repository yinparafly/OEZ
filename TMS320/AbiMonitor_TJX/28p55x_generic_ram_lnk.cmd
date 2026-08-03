MEMORY
{
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN            : origin = 0x000000, length = 0x000002
   BOOT_RSVD        : origin = 0x000002, length = 0x000126     /* Part of M0, BOOT rom will use this for stack */
   RAMM0            : origin = 0x000128, length = 0x0002D8
   RAMM1            : origin = 0x000400, length = 0x000400
   
   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
    RAMLS5           : origin = 0x00A800, length = 0x000800
    RAMLS6           : origin = 0x00B000, length = 0x000800
    RAMLS7           : origin = 0x00B800, length = 0x000800

    RAMGS0A          : origin = 0x00C000, length = 0x001000   /* .text 补区 */

    /* RAMGS0B-3 + RAMLS8/9 物理连续（0xD000-0x17FFF，无洞），合并为连续数据区
       .bss 是 blocked section（EABI 结构体页对齐优化），必须整段放入连续内存 */
    RAMDATA          : origin = 0x00D000, length = 0x00B000

   /* Flash Banks (128 sectors each) */
   FLASH_BANK0     : origin = 0x080000, length = 0x20000
   FLASH_BANK1     : origin = 0x0A0000, length = 0x20000
   FLASH_BANK2     : origin = 0x0C0000, length = 0x20000
   FLASH_BANK3     : origin = 0x0E0000, length = 0x20000
   FLASH_BANK4     : origin = 0x100000, length = 0x07FFF


   CLATOCPURAM      : origin = 0x001480,   length = 0x000080
   CPUTOCLARAM      : origin = 0x001500,   length = 0x000080
   CLATODMARAM      : origin = 0x001680,   length = 0x000080
   DMATOCLARAM      : origin = 0x001700,   length = 0x000080

   RESET            : origin = 0x3FFFC0, length = 0x000002
}


SECTIONS
{
   codestart        : > BEGIN
   .text            : >> RAMLS0 | RAMLS1 | RAMLS2 | RAMLS3 | RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMGS0A
   .cinit           : > RAMM0
   .switch          : > RAMM0
   .reset           : > RESET, TYPE = DSECT /* not used, */

   .stack           : > RAMM1
#if defined(__TI_EABI__)
    .bss             : > RAMDATA
    .bss:output      : > RAMLS3
    .init_array      : > RAMM0
    .const           : >> RAMDATA
    .data            : > RAMDATA
    .sysmem          : > RAMDATA
#else
   .pinit           : > RAMM0
   .ebss            : >> RAMLS5 | RAMLS6
   .econst          : > RAMLS5
   .esysmem         : > RAMLS5
#endif

    .TI.ramfunc : {} > RAMM0

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
