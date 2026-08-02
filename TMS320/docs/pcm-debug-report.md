# PCM (Position Compare Match) 调试求助

> 芯片：TMS320F28P550SJ9，CCS 20.1.1，C2000Ware 5.04，编译器 22.6.1.LTS
> 现象：eQEP1 正交编码器（1000PPR，4X），PCM 中断始终不触发（QFLG POS_COMP_MATCH=0）
> C2000Ware 中无 PCM 官方例程（TI 5 个 eqep 例程均用 UTO+capture，无 PCM）

## 硬件状态

- EQEP1 正交模式正常：QDC 方向变化中断触发，IEL 索引中断触发（每圈 1 次）
- QEINT=0x0500（PCM 和 IEL 中断使能已开）
- EQEP1 输入：GPIO50(A) / GPIO51(B) / GPIO53(I)，`EQEP_setInputPolarity(..., false, false, false, false)`
- SysConfig 配置：QPOSMAX=0xFFFFFFFF，`EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_1X_RESOLUTION`

## SysConfig 初始化的寄存器状态

```c
// board.c (SysConfig 生成)
EQEP_setCompareConfig(Module_EQEP_BASE,
    EQEP_COMPARE_NO_SYNC_OUT | EQEP_COMPARE_LOAD_ON_MATCH,  // config=0xC000
    0U,   // compareValue=0
    0U);  // cycles=0 ← 可能有问题！
EQEP_enableCompare(Module_EQEP_BASE);
```

### EQEP_setCompareConfig 的实现

```c
void EQEP_setCompareConfig(uint32_t base, uint16_t config,
                           uint32_t compareValue, uint16_t cycles)
{
    HWREG(base + EQEP_O_QPOSCMP) = compareValue;  // 写 QPOSCMP
    // config & (PCSHDW|PCLOAD) → 0xC000 & 0x0043 = 0x0000
    // cycles-1 = 0-1 = 0xFFFF (uint16 underflow!)
    uint16_t regValue = (config & 0x0043) | 0xFFFF;  // = 0xFFFF
    HWREGH(base + EQEP_O_QPOSCTL) = ~0xF043 & old | regValue;
    // 结果：QPOSCTL = 0xF000 (PCSPW=15, PCSHDW=0, PCLOAD=0)
}
```

**问题**：`cycles=0` 导致 `0-1=0xFFFF`，PCSPW 字段被写为最大值 15。这可能是合法的（sync 输出的 pulse width）。但更重要的是 `PCSHDW=0`（shadow 关闭）和 `PCLOAD=0`（load mode=零匹配）。

## 我们的 abi_init 尝试修复

```c
void abi_init(void) {
    EQEP_disableCompare(Module_EQEP_BASE);
    // 清零 PCSPW, PCSHDW, PCLOAD (期望 → 0x0000)
    HWREGH(base + QPOSCTL) &= ~(0xF000 | 0x0040 | 0x0003);  // = 0x0000
    // 设 PCLOAD=0x0003 (load on both zero and match)
    HWREGH(base + QPOSCTL) |= 0x0003;                        // = 0x0003
    // 写 CMP active register
    HWREG(base + QPOSCMP) = current_QPOSCNT + 1;             // = 10000 (测试)
    // 使能 shadow
    HWREGH(base + QPOSCTL) |= 0x0040;                        // = 0x0043
    EQEP_enableCompare(Module_EQEP_BASE);
}
```

**但 CLI 读取 QPOSCTL 始终为 0xF000**，说明写入完全无效！QPOSCMP 也停留在 1 不变。

`EQEP_disableCompare` → `EQEP_enableCompare` 是否重置了寄存器？查 `EQEP_enableCompare` 源码：

```c
static inline void EQEP_enableCompare(uint32_t base)
{
    EQEP_setCompareConfig(base, oldConfig, oldValue, oldCycles);
}
```

如果 `EQEP_enableCompare` 内部调用了 `EQEP_setCompareConfig` 读取 oldConfig（但是我们的 abi_init 没有保存 oldConfig，导致它读取的可能是未初始化变量？）。不对，EQEP_enableCompare 的实现是：

```c
// 实际上 enableCompare 实现是
// HWREGH(base + QPOSCTL) |= 某些控制位
// 对吗？
```

**待确认：EQEP_disableCompare / EQEP_enableCompare 是否会影响 QPOSCTL**

## 核心问题

1. **为何 QPOSCTL 写不进去？** `HWREGH(base + EQEP_O_QPOSCTL) &= ~0xF043` 后应该为 0x0000，为何读回 0xF000？
2. **EQEP_enableCompare/EQEP_disableCompare 做了什么？** 这些函数是否会重置 QPOSCTL？
3. **PCM 使能位已开 (QEINT=0x0100)，为何 QFLG PCM 位从不置位？** 即使 QPOSCNT 在变化（电机在转，IEL 每圈触发），比较应匹配（CMP=1, QPOSCNT 从 0→1 时应该匹配）。

## 已验证排除的因素

- QEINT=0x0500 → PCM+IEL 中断已使能 ✓
- QDC 中断 (bit 1) 能正常触发 ✓
- IEL 中断 (bit 10) 能正常触发 ✓
- 仅 PCM (bit 8) 不触发 ✗
- PCSHDW=0, PCLOAD=0 → 无 shadow, 仅零匹配（已知问题但修复后仍无效）
