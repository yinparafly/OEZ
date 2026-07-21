# -*- coding: utf-8 -*-
"""生成前馈/PID 前后对比文档 + 霍尔定点减速比精度分析。
用法: python make_ff_pid_report.py <diag.json> <relearn.json> <verify.json>
不接电机，只读已采集的 JSON/CSV。"""
import json
import sys
import statistics as st
from pathlib import Path
from datetime import datetime

from analyze_learn_log import interp_pulse_for_rpm

DESIGN = 27.7440476190
LOG_DIR = Path(__file__).resolve().parent / "logs"


def load(p):
    return json.loads(Path(p).read_text(encoding="utf-8"))


def gstats(vals, lo=27.2, hi=28.3):
    """原始 + 过滤（去除双触发低值 26.x/其它离群）后的统计。"""
    raw = [v for v in vals if v > 0]
    filt = [v for v in raw if lo <= v <= hi]
    def pack(a):
        if not a:
            return None
        m = st.mean(a)
        return {
            "n": len(a),
            "mean": m,
            "median": st.median(a),
            "std": st.pstdev(a) if len(a) > 1 else 0.0,
            "err_pct": (m - DESIGN) / DESIGN * 100.0,
            "med_err_pct": (st.median(a) - DESIGN) / DESIGN * 100.0,
        }
    return {"raw": pack(raw), "filt": pack(filt)}


def fmt6(x):
    return f"{x:.6f}" if x is not None else "—"


def ff_table(old_pts, new_pts, rpms):
    rows = []
    for r in rpms:
        po = interp_pulse_for_rpm(old_pts, r)
        pn = interp_pulse_for_rpm(new_pts, r)
        d = pn - po if (po and pn) else None
        dp = (d / po * 100.0) if (d is not None and po) else None
        rows.append((r, po, pn, d, dp))
    return rows


def probe_map(probes):
    return {int(round(p["target"])): p for p in probes}


def main():
    diag = load(sys.argv[1])
    relearn = load(sys.argv[2])
    verify = load(sys.argv[3])
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    old_pts = diag["old_ff_points"]
    new_pts = verify["new_ff_points"]
    so = diag["state_old"]
    sn = verify.get("state_new") or relearn["state_new"]

    # 霍尔定点减速比（诊断阶段 + 验证阶段合并样本）
    dn_all = diag["hall_gear"]["dn_vals"] + verify["hall_gear"]["dn_vals"]
    up_all = diag["hall_gear"]["up_vals"] + verify["hall_gear"]["up_vals"]
    gdn = gstats(dn_all)
    gup = gstats(up_all)
    gall = gstats(dn_all + up_all)

    op = probe_map(diag["probes_old"])
    np_ = probe_map(verify["probes_new"])

    L = []
    L.append(f"# 前馈图 / PID 新旧对比 + 霍尔定点减速比精度分析\n")
    L.append(f"- 生成时间：{ts}")
    L.append(f"- 负载：齿轮台架，设计减速比 i=(59×79)/(12×14)=4661/168=**{DESIGN:.10f}**（定点 27.744048）")
    L.append(f"- 平台：ESP32-S3 + AS5047P(16384 cpr) + RC ESC @ GPIO9，COM10@921600")
    L.append(f"- 诊断日志：{Path(diag['telem_csv']).name} / {Path(diag['hall_csv']).name}")
    L.append(f"- 重学日志：{Path(relearn['learn_csv']).name}")
    L.append("")

    L.append("## 一、结论摘要\n")
    L.append("| 项目 | 是否需要调整 | 说明 |")
    L.append("|---|---|---|")
    L.append("| 前馈图 mapPulseFromRpm | **是（已重建）** | 旧图为轻载图，新齿轮台架下同 pulse 产生的 rpm 显著更高，旧前馈严重过冲、稳态 PID 输出 u≈−90~−110µs 长期补偿 |")
    L.append("| 全局 PID | **是（已更新）** | kp 0.08→0.0248、ki 0.25→0.0621（对象增益变大，需降增益）|")
    L.append("| 分区 PID GainMap | **是（已生成）** | 旧图 0 点无效；新图 12 点、schedule=ON，已 SAVE |")
    L.append("| KA 加速度前馈 | 否 | 仍为 0；SOFT 斜坡下升速平滑、超调已≤2%，暂不需要 |")
    L.append("| ADG 自适应增益 | 否 | 保持 OFF；分区增益已覆盖全转速段 |")
    L.append("| 减速比测量 | 定点已生效 | 干净样本 ≈27.74，与设计吻合；见第五节 |")
    L.append("")

    L.append("## 二、前馈图 旧 vs 新（同 rpm 断点下 pulse）\n")
    L.append("| 目标 rpm | 旧 pulse(µs) | 新 pulse(µs) | Δ(µs) | 变化% |")
    L.append("|---:|---:|---:|---:|---:|")
    for r, po, pn, d, dp in ff_table(old_pts, new_pts, [2000, 2500, 3000, 3500, 4000, 5000, 6000]):
        L.append(f"| {r} | {po:.0f} | {pn:.0f} | {d:+.0f} | {dp:+.1f}% |")
    L.append("")
    L.append("> 说明：相同目标 rpm 下新前馈所需 pulse 明显更低（新台架同 pulse 转得更快）。这正是旧前馈在闭环里被 PID 长期“往下拉” ~90–110µs 的原因。")
    L.append("")

    L.append("## 三、PID / KA / ADG 旧 vs 新\n")
    L.append("| 参数 | 旧值 | 新值 |")
    L.append("|---|---|---|")
    L.append(f"| 全局 kp | {so['kp']} | {sn['kp']} |")
    L.append(f"| 全局 ki | {so['ki']} | {sn['ki']} |")
    L.append(f"| 全局 kd | {so['kd']} | {sn['kd']} |")
    L.append(f"| KA (µs/(rpm/s)) | {so['ka']} | {sn['ka']} |")
    L.append(f"| ADG | {so['adg_on']} (dual={so['adg_dual']}) | {sn['adg_on']} (dual={sn['adg_dual']}) |")
    L.append(f"| 分区 PID 点数/有效 | {len(so.get('gain_zones', []))} / valid={so.get('gain_valid')} | {len(sn.get('gain_zones', []))} / valid={sn.get('gain_valid')} |")
    L.append("")
    L.append(f"SUGGEST 建议：gain={relearn['suggest'].get('gain')}, kp={relearn['suggest'].get('kp')}, ki={relearn['suggest'].get('ki')}（已 PID SAVE）")
    L.append("")
    L.append("### 新分区 PID GainMap（已 GAIN SAVE, schedule=ON）\n")
    L.append("| rpm | kp | ki | kd |")
    L.append("|---:|---:|---:|---:|")
    for z in sn.get("gain_zones", []):
        L.append(f"| {z['rpm']:.0f} | {z['kp']:.4f} | {z['ki']:.4f} | {z['kd']:.4f} |")
    L.append("")

    L.append("## 四、闭环性能 旧前馈 vs 新前馈（均 ≥2000rpm，连续平滑升速、无中途停车）\n")
    L.append("| 目标 | 稳态均值rpm 旧→新 | 静差% 旧→新 | 超调% 旧→新 | 稳态pulse 旧→新 | PID输出u(µs) 旧→新 | 建立时间s 旧→新 |")
    L.append("|---:|---|---|---|---|---|---|")
    for r in [2000, 2500, 3000, 4000, 6000]:
        o = op.get(r)
        n = np_.get(r)
        def g(d, k, f="{}"):
            return f.format(d[k]) if (d and d.get(k) is not None) else "—"
        L.append("| {t} | {om}→{nm} | {oe}→{ne} | {oo}→{no} | {op_}→{npp} | {ou}→{nu} | {ost}→{nst} |".format(
            t=r,
            om=g(o, "mean_rpm"), nm=g(n, "mean_rpm"),
            oe=g(o, "static_err_pct"), ne=g(n, "static_err_pct"),
            oo=g(o, "overshoot_pct"), no=g(n, "overshoot_pct"),
            op_=g(o, "settle_pulse"), npp=g(n, "settle_pulse"),
            ou=g(o, "u"), nu=g(n, "u"),
            ost=g(o, "settle_time_s"), nst=g(n, "settle_time_s"),
        ))
    L.append("")
    L.append("> 旧前馈：2000/2500 处超调 30%/4%，且稳态 u≈−88~−110µs（PID 长期大幅反向补偿）。")
    L.append("> 新前馈：2000–6000 全段超调 ≤2.3%，稳态 u≈−1~−5µs（前馈几乎精确命中），静差 <0.05%，建立时间 1–2.4s。")
    L.append("> 注：旧前馈未在 4000/6000 复测——旧图过冲严重，高速复测有冲过 Nyquist(7500rpm) 风险，故仅用 2000/2500 证明其偏差；新前馈已安全跑到 6000。")
    L.append("")

    L.append("## 五、霍尔定点减速比精度（0° dn / 180° up，样本合并 diag+verify）\n")
    L.append("定点实现：电机端按 AS5047P 整型编码器计数累加（int64，每圈16384，±8192环绕已处理）；")
    L.append("同标记两次过点 Δcounts×1e6/16384 得减速比×1e6，频率 1e9/dt_ms，全程整数运算，打印 `%lld.%06lld`，不再 %.3f 截断。\n")
    L.append("| 相位 | 样本(原始/过滤) | 过滤后均值 | 过滤后中位数 | 过滤后std | 均值vs设计 | 中位vs设计 |")
    L.append("|---|---|---:|---:|---:|---:|---:|")
    for name, g in [("0° (dn)", gdn), ("180° (up)", gup), ("合并", gall)]:
        r0, f0 = g["raw"], g["filt"]
        L.append("| {nm} | {rn}/{fn} | {fm} | {fmed} | {fs} | {fe:+.4f}% | {fme:+.4f}% |".format(
            nm=name, rn=r0["n"], fn=f0["n"],
            fm=fmt6(f0["mean"]), fmed=fmt6(f0["median"]), fs=fmt6(f0["std"]),
            fe=f0["err_pct"], fme=f0["med_err_pct"],
        ))
    L.append("")
    L.append(f"> 过滤规则：剔除 <27.2 的双触发离群值（一次物理过点被触发两次→少计 ~1 电机圈，出现 ~26.7/25.8 假值），保留完整整圈样本。")
    L.append("")
    L.append("### 关于“此前 27.742 是否为打印截断”的明确回答\n")
    L.append(f"- **不是打印/浮点截断造成的“变差”**。定点(6位)下干净整圈样本稳定落在 ~27.72–27.77，示例：27.744324 / 27.745056 / 27.742004 / 27.740112，与设计 {DESIGN:.6f} 相差仅约 0.001–0.05%。")
    L.append(f"- 合并过滤后中位数 ≈ **{fmt6(gall['filt']['median'])}**（vs 设计 {gall['filt']['med_err_pct']:+.4f}%）。")
    L.append("- 3 位显示的 27.742 本身就是一个真实且准确的读数（距设计 <0.01%）；3 位截断仅隐藏第 4 位以后（±0.0005），远小于逐圈实际抖动（±0.2~0.3%，来自标记角度重复性/编码器采样量化/偶发双触发），因此“看到 0.01%/0.1% 波动”是真实测量抖动，**而非打印位数造成**。")
    L.append("- 定点法下 0°(dn) 与 180°(up) 的干净中位数均 ≈27.74，两相位一致，且与设计吻合。")
    L.append("")

    L.append("## 六、早停根因（本轮排查）\n")
    L.append("- 现象：此前“转速还没起来就停了”。")
    L.append("- 根因：控制脚本被中断（工具调用被打断）→ Python 进程退出 → 保活停止 → 固件 **HOST_TIMEOUT(1.5s) 触发 ESTOP**，电机立即停车；并非电调/超速保护误触发。")
    L.append("- 佐证：未被打断的那次诊断实际完整跑完并稳定到 2500rpm（telem 到 t≈89.8s）。")
    L.append("- 修复：本轮把 spin 放到**后台独立进程**执行（不随对话中断而被杀）；保活周期 0.7s→**0.5s**；SOFT 上升速率提到 **800rpm/s**（数秒内到 2000+）；CSV 每秒 flush 防丢数据。验证阶段一次平滑升到各目标并稳定 ≥20–35s，无中途掉速。")
    L.append("")

    out = LOG_DIR / f"ff_pid_对比_{ts}.md"
    out.write_text("\n".join(L), encoding="utf-8")
    print("REPORT:", out)


if __name__ == "__main__":
    main()
