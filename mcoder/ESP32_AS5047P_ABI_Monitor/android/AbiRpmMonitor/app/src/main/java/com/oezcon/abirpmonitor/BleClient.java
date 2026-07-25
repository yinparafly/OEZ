package com.oezcon.abirpmonitor;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;

/**
 * Nordic UART → OEZ-ABI。
 * Android：Dump 期间以 Notify 为主（勿照搬 PC 持续 READ）；
 * 仅听 {@code BLE PULL READY} 后 SNAP? → DUMP BIN BLE / SD；无 PING/无自动重连。
 */
public class BleClient {
    public static final UUID NUS_SERVICE =
            UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID NUS_RX =
            UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID NUS_TX =
            UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID CCCD =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    public interface Listener {
        void onStatus(String msg);

        void onDevices(List<DeviceItem> devices);

        void onConnected(boolean ok);

        void onLive(float rpm, int dir, boolean armed, int phase, int logN, int logDrop,
                    float measHz, float stageRevs, int remainMs, int segs,
                    float revsAbi, float revsAbs, long indexN, long indexSigned,
                    long tRelMs, long unixMs);

        void onDumpSample(int idx, long tMs, float rpm, int dir, int seg, long indexN,
                          long tRelMs, long unixMs);

        /** I 过零跳动事件：rpmI 由转间隔推算 */
        void onIndexEvent(int seq, long tMs, long indexN, float rpmAb, float rpmI, long dtMs,
                          int seg, long tRelMs, long unixMs);

        void onDumpEnd(int n);

        void onIndexEnd(int n);

        void onInfo(String line);

        /** RAM snap 二进制拉齐（DUMP BIN BLE），已 CRC 校验。 */
        void onSnapBin(int n, int hz, List<float[]> rows);

        /** DUMP BIN BLE 进度：已收字节 / 期望字节 / 百分比 0~100。 */
        default void onBinBleProgress(int gotBytes, int expectBytes, int pct) {}

        /** PC 同款拉取提示：0=即将/开始拉取；-1=取消。（不再用 15s 倒数） */
        default void onPullCountdown(int secLeft) {}
    }

    public static class DeviceItem {
        public final String address;
        public final String name;

        public DeviceItem(String address, String name) {
            this.address = address;
            this.name = name;
        }

        @Override
        public String toString() {
            return name + "  [" + address + "]";
        }
    }

    private enum OpType { WRITE, READ }

    private static final class Op {
        final OpType type;
        final byte[] data;

        Op(OpType type, byte[] data) {
            this.type = type;
            this.data = data;
        }
    }

    private final Context appCtx;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final Listener listener;
    private final Map<String, DeviceItem> found = new LinkedHashMap<>();
    private final ArrayDeque<Op> opQ = new ArrayDeque<>();
    private boolean busy;
    private boolean notifyReady;
    private boolean mtuDone;

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rxChar;
    private BluetoothGattCharacteristic txChar;
    private boolean connected;
    private StringBuilder rxBuf = new StringBuilder(65536);
    private boolean dumping;
    private int dumpAccepted;
    private final java.util.ArrayList<Object[]> dumpBatch = new java.util.ArrayList<>(256);
    private boolean binBleActive;
    private final java.io.ByteArrayOutputStream binBleBuf = new java.io.ByteArrayOutputStream(65536);
    private int binBleExpect;
    private int binBleChunks;
    private int binBleLastPct = -1;
    private long binBleLastUiMs;
    /** 下一包期望 seq；小于此值视为 Notify+READ 重复包 */
    private int binBleNextSeq;
    private String binBleSrc = "RAM";
    /** 与 PC ble_link last_val 相同：跳过特征值重复读 */
    private byte[] lastIngestPayload;
    private long lastTelemMs;
    private int telemCount;
    private long lastPollMs;
    private long lastStatusMs;
    /** 发给板子的 TIME 锚点（毫秒），用于 unix=0 时本地推算 */
    private long timeAnchorUnixMs;
    private String lastAddress;
    private boolean ackQueued;
    private int dumpRetryLeft;
    /** 最近一次 snap 记完时间；此期间禁止发旧 LOG READ */
    private long snapDoneAtMs;
    /** 对齐 PC `_awaiting_bindump`：避免重复自动拉 */
    private boolean awaitingBindump;
    private boolean pullScheduled;
    /** 正在等 # SNAP valid= 回复 */
    private boolean snapQueryPending;
    /**
     * 流分离：点了开始监控后停 GATT 轮询读，直到 STOP / 记完 / PULL。
     * 与固件「武装后停发 BLE 实时转速」对称。
     */
    private boolean monitorSession;
    /** 本进程是否已完成首次连接引导（TIME/RATE/ABI?） */
    private boolean sessionConfigured;
    /** 首次连接后允许自动发一次 REC MS */
    private boolean recMsPendingFirst = true;
    private final Runnable dumpRetryTask = new Runnable() {
        @Override
        public void run() {
            if (!connected || binBleActive || awaitingBindump) return;
            if (dumpRetryLeft <= 0) return;
            dumpRetryLeft--;
            status("延时重试拉取 RAM… 剩余 " + dumpRetryLeft);
            dumpBinBle();
        }
    };
    private final Runnable pcPullTask = new Runnable() {
        @Override
        public void run() {
            pullScheduled = false;
            if (!connected || binBleActive) return;
            if (awaitingBindump) return;
            status("PC同款 · SNAP? → DUMP BIN BLE");
            ui.post(() -> listener.onPullCountdown(0));
            dumpBinBle();
        }
    };

    /**
     * Android 策略（朋友建议）：空闲可慢速 READ；Dump/BIN 期间默认只用 Notify，
     * 仅当 500ms 无新包时才补一次 READ（救援）。勿照搬 PC 持续快读。
     */
    private long lastBinRxMs;
    private final Runnable tickTask = new Runnable() {
        @Override
        public void run() {
            if (!connected) return;
            long now = System.currentTimeMillis();
            // 监控会话中：不轮询读
            if (monitorSession && !(dumping || binBleActive)) {
                if (now - lastStatusMs >= 2000L) {
                    lastStatusMs = now;
                    status("监控中 · 流已分离（无实时转速，等事件）");
                }
                ui.postDelayed(this, 200L);
                return;
            }
            if (dumping || binBleActive) {
                // Dump：默认不 READ；500ms 无进度才救援读一次
                if (lastBinRxMs > 0 && (now - lastBinRxMs) >= 500L
                        && (now - lastPollMs) >= 500L) {
                    lastPollMs = now;
                    enqueueRead();
                    status("BIN 超时救援 READ（仅一次窗口）");
                }
                if (now - lastStatusMs >= 1500L) {
                    lastStatusMs = now;
                    status(String.format(Locale.US, "拉取中(Notify) · %d/%s B",
                            binBleBuf.size(),
                            binBleExpect > 0 ? String.valueOf(binBleExpect) : "?"));
                }
                ui.postDelayed(this, 100L);
                return;
            }
            // 空闲：慢速 READ 兜底遥测（≥200ms，朋友2建议；勿过密）
            if (now - lastPollMs >= 200L) {
                lastPollMs = now;
                enqueueRead();
            }
            if (now - lastStatusMs >= 2000L) {
                lastStatusMs = now;
                String st;
                if (lastTelemMs == 0) {
                    st = "已连接 · 等待遥测…";
                } else if (now - lastTelemMs > 3000) {
                    st = String.format(Locale.US, "已连接 · 遥测暂停(曾%d帧)", telemCount);
                } else {
                    st = String.format(Locale.US, "已连接 · 遥测正常 %d帧", telemCount);
                }
                status(st);
            }
            ui.postDelayed(this, 40L);
        }
    };

    private final Runnable busyWatchdog = new Runnable() {
        @Override
        public void run() {
            if (!connected) return;
            if (busy) {
                busy = false;
                pump();
            }
            ui.postDelayed(this, 1200);
        }
    };

    private final Runnable mtuFallback = new Runnable() {
        @Override
        public void run() {
            if (!mtuDone && gatt != null && connected) {
                status("MTU 超时，直接发现服务…");
                gatt.discoverServices();
            }
        }
    };

    public BleClient(Context ctx, Listener listener) {
        this.appCtx = ctx.getApplicationContext();
        this.listener = listener;
        BluetoothManager bm =
                (BluetoothManager) appCtx.getSystemService(Context.BLUETOOTH_SERVICE);
        if (bm != null) {
            adapter = bm.getAdapter();
            if (adapter != null) scanner = adapter.getBluetoothLeScanner();
        }
    }

    /** 武装中或正在拉数：UI 应禁用「开始监控」，防止二次武装清 RAM */
    public boolean isMonitorOrPullBusy() {
        return monitorSession || dumping || binBleActive || awaitingBindump || pullScheduled;
    }

    public boolean isMonitorSession() {
        return monitorSession;
    }

    public boolean hasAdapter() {
        return adapter != null && adapter.isEnabled();
    }

    public boolean isConnected() {
        return connected;
    }

    /** 是否已做过首次连接配置（供 UI 决定是否自动发 REC MS） */
    public boolean isSessionConfigured() {
        return sessionConfigured;
    }

    /** 首次连接前为 false；finishNotifySetup 里首次设 true 之后重连不再自动灌设定 */
    public boolean shouldAutoSendRecMs() {
        // finishNotifySetup 在 onConnected(true) 之前把 sessionConfigured 置 true，
        // 故用「尚未配置」判断会失败。改为：仅当 recMsPendingFirst 时自动发。
        return recMsPendingFirst;
    }

    public void clearRecMsPendingFirst() {
        recMsPendingFirst = false;
    }

    @SuppressLint("MissingPermission")
    public void startScan(long ms) {
        if (scanner == null) {
            status("无 BLE 适配器");
            return;
        }
        found.clear();
        status("扫描中…");
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();
        scanner.startScan(null, settings, scanCb);
        ui.postDelayed(this::stopScan, ms);
    }

    @SuppressLint("MissingPermission")
    public void stopScan() {
        if (scanner != null) {
            try {
                scanner.stopScan(scanCb);
            } catch (Exception ignored) {
            }
        }
        listener.onDevices(new ArrayList<>(found.values()));
        status("扫描结束 · " + found.size() + " 台");
    }

    private final ScanCallback scanCb = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice d = result.getDevice();
            String name = result.getScanRecord() != null
                    ? result.getScanRecord().getDeviceName() : null;
            if (name == null) name = d.getName();
            if (name == null) return;
            String up = name.toUpperCase(Locale.US);
            if (!(up.contains("OEZ-ABI") || up.startsWith("OEZ"))) return;
            found.put(d.getAddress(), new DeviceItem(d.getAddress(), name));
            ui.post(() -> listener.onDevices(new ArrayList<>(found.values())));
        }
    };

    @SuppressLint("MissingPermission")
    public void connect(String address) {
        disconnect();
        lastAddress = address;
        BluetoothDevice d = adapter.getRemoteDevice(address);
        status("连接 " + address + " …");
        lastTelemMs = 0;
        telemCount = 0;
        dumping = false;
        awaitingBindump = false;
        pullScheduled = false;
        rxBuf.setLength(0);
        notifyReady = false;
        mtuDone = false;
        // 对齐 PC：不设 CONNECTION_PRIORITY_HIGH、不自动重连
        gatt = d.connectGatt(appCtx, false, gattCb, BluetoothDevice.TRANSPORT_LE);
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        ui.removeCallbacks(tickTask);
        ui.removeCallbacks(busyWatchdog);
        ui.removeCallbacks(mtuFallback);
        ui.removeCallbacks(dumpRetryTask);
        ui.removeCallbacks(pcPullTask);
        cancelPendingPull();
        connected = false;
        notifyReady = false;
        dumping = false;
        binBleActive = false;
        awaitingBindump = false;
        rxChar = null;
        txChar = null;
        opQ.clear();
        busy = false;
        if (gatt != null) {
            try {
                gatt.disconnect();
                gatt.close();
            } catch (Exception ignored) {
            }
            gatt = null;
        }
        listener.onConnected(false);
    }

    private static String gattStatusText(int status) {
        switch (status) {
            case 0:
                return "OK";
            case 8:
                return "连接超时(8)——多见于回传过猛";
            case 19:
                return "对端断开(19)";
            case 22:
                return "本地断开(22)";
            case 133:
                return "GATT 133（可重连）";
            default:
                return "status=" + status;
        }
    }

    public void send(String cmd) {
        if (cmd == null) return;
        if (dumping) {
            String u = cmd.trim().toUpperCase(Locale.US);
            // 回传中仍允许监控/对时；禁止其它杂令
            if (!(u.startsWith("DUMP ACK") || u.equals("ACK") || u.startsWith("DUMP NEXT")
                    || u.startsWith("DUMP ABORT") || u.startsWith("MONITOR")
                    || u.startsWith("TIME") || u.startsWith("ABI")
                    || u.startsWith("LOG DUMP") || u.startsWith("LOG CLEAR")
                    || u.startsWith("DUMP BIN") || u.startsWith("BIN DUMP") || u.startsWith("BLE BIN")
                    || u.startsWith("SNAP") || u.equals("READ") || u.startsWith("LOG READ"))) {
                return;
            }
        }
        enqueueWrite(cmd);
    }

    public void monitorStart() {
        // 对齐 PC _on_monitor_start：先 TIME，40ms 后 MONITOR START；不发 DIAG
        awaitingBindump = false;
        pullScheduled = false;
        monitorSession = true;
        ui.removeCallbacks(pcPullTask);
        cancelPendingPull();
        timeAnchorUnixMs = System.currentTimeMillis();
        enqueueWrite("TIME " + timeAnchorUnixMs);
        ui.postDelayed(() -> {
            if (connected) enqueueWrite("MONITOR START");
        }, 40);
        status("已武装 · BLE 实时转速关闭（流分离）");
    }

    public void monitorStop() {
        monitorSession = false;
        send("MONITOR STOP");
        status("已停止 · 恢复实时转速");
    }

    public void logClear() {
        send("LOG CLEAR");
    }

    public void logDump() {
        send("LOG DUMP");
    }

    /** SNAP? → 看 valid；valid=0 则 DUMP BIN BLE SD，否则 DUMP BIN BLE */
    public void dumpBinBle() {
        if (!connected) return;
        awaitingBindump = true;
        snapQueryPending = true;
        dumpRetryLeft = 5;
        snapDoneAtMs = System.currentTimeMillis();
        enqueueWrite("SNAP?");
        ui.postDelayed(() -> {
            if (!connected || binBleActive || !snapQueryPending) return;
            snapQueryPending = false;
            status("SNAP? 超时 · 仍发 DUMP BIN BLE");
            enqueueWrite("DUMP BIN BLE");
        }, 800);
    }

    /** 强制从 SD 最新 snap_*.bin 拉（先读卡进板再 BLE） */
    public void dumpBinBleSd() {
        if (!connected) return;
        awaitingBindump = true;
        dumpRetryLeft = 3;
        enqueueWrite("DUMP BIN BLE SD");
    }

    private void cancelPendingPull() {
        pullScheduled = false;
        ui.removeCallbacks(pcPullTask);
        ui.removeCallbacks(dumpRetryTask);
        ui.post(() -> listener.onPullCountdown(-1));
    }

    /**
     * 对齐 PC：仅 {@code BLE PULL READY} 后 200ms 自动拉。
     * CD15 / RECORD done 只提示，不拉（等板子 SD SAVE 完成）。
     */
    private void schedulePcStylePull(String why) {
        if (!connected || binBleActive || awaitingBindump) return;
        if (pullScheduled) return;
        pullScheduled = true;
        monitorSession = false; // 记完：允许再 poll；固件也会恢复 L
        snapDoneAtMs = System.currentTimeMillis();
        status("BLE PULL READY · 200ms 后拉取（同 PC / " + why + "）");
        ui.post(() -> listener.onPullCountdown(0));
        ui.removeCallbacks(pcPullTask);
        ui.postDelayed(pcPullTask, 200L);
    }

    private void scheduleDumpRetry(long delayMs) {
        if (dumpRetryLeft <= 0) dumpRetryLeft = 4;
        awaitingBindump = false;
        ui.removeCallbacks(dumpRetryTask);
        ui.postDelayed(dumpRetryTask, Math.max(delayMs, 1200L));
    }

    private boolean inSnapSettleWindow() {
        return snapDoneAtMs > 0 && (System.currentTimeMillis() - snapDoneAtMs) < 20000L;
    }

    private void status(String msg) {
        ui.post(() -> listener.onStatus(msg));
    }

    private void postBinProgress(int got, int expect, int pct) {
        final int g = Math.max(0, got);
        final int e = Math.max(0, expect);
        final int p = Math.max(0, Math.min(100, pct));
        ui.post(() -> listener.onBinBleProgress(g, e, p));
    }

    private void enqueueWrite(String line) {
        String s = line.endsWith("\n") ? line : line + "\n";
        Op op = new Op(OpType.WRITE, s.getBytes(StandardCharsets.UTF_8));
        // 对齐 PC：DUMP ACK 插队，优先于其它命令
        if (s.startsWith("DUMP ACK")) {
            opQ.addFirst(op);
        } else {
            opQ.offer(op);
        }
        pump();
    }

    private void enqueueRead() {
        for (Op op : opQ) {
            if (op.type == OpType.READ) return;
        }
        opQ.offer(new Op(OpType.READ, null));
        pump();
    }

    @SuppressLint("MissingPermission")
    private void pump() {
        if (busy || !connected || gatt == null) return;
        Op op = opQ.poll();
        if (op == null) return;
        busy = true;
        try {
            if (op.type == OpType.WRITE) {
                if (rxChar == null) {
                    busy = false;
                    return;
                }
                boolean ok;
                if (Build.VERSION.SDK_INT >= 33) {
                    int r = gatt.writeCharacteristic(rxChar, op.data,
                            BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
                    ok = (r == BluetoothGatt.GATT_SUCCESS);
                } else {
                    rxChar.setValue(op.data);
                    rxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
                    ok = gatt.writeCharacteristic(rxChar);
                }
                // NO_RESPONSE：短延时放行；回传 ACK 尽快发
                final boolean isAck = op.type == OpType.WRITE && op.data != null
                        && new String(op.data, StandardCharsets.UTF_8).startsWith("DUMP ACK");
                ui.postDelayed(() -> {
                    busy = false;
                    if (isAck) ackQueued = false;
                    pump();
                }, ok ? (isAck ? 8 : 15) : 40);
            } else {
                if (txChar == null) {
                    busy = false;
                    return;
                }
                if (!gatt.readCharacteristic(txChar)) {
                    busy = false;
                    ui.postDelayed(this::pump, 40);
                }
            }
        } catch (SecurityException e) {
            busy = false;
        }
    }

    private void flushDumpBatch() {
        if (dumpBatch.isEmpty()) return;
        final java.util.ArrayList<Object[]> batch = new java.util.ArrayList<>(dumpBatch);
        dumpBatch.clear();
        ui.post(() -> {
            for (Object[] a : batch) {
                listener.onDumpSample(
                        (Integer) a[0], (Long) a[1], (Float) a[2], (Integer) a[3],
                        (Integer) a[4], (Long) a[5], (Long) a[6], (Long) a[7]);
            }
        });
    }

    private void queueDumpAck() {
        if (!dumping || !connected) return;
        if (ackQueued) return;
        ackQueued = true;
        enqueueWrite("DUMP ACK");
    }

    private void handleLine(String line) {
        // 实时遥测到来 = 退出回传静默（修复卡住后不显示转速）
        // 短帧(BLE): L,rpm,dir,armed,phase,remain,log_n
        // 长帧(USB): L,t,rpm,dir,armed,log_n,drop,hz,...,phase,...
        if (line.startsWith("L,")) {
            if (dumping && !binBleActive) {
                dumping = false;
                ackQueued = false;
                status("遥测已恢复");
            }
            lastTelemMs = System.currentTimeMillis();
            telemCount++;
            String[] p = line.split(",");
            if (p.length < 3) return;
            try {
                final float rpm;
                final int dir;
                final boolean armed;
                final int logN;
                final int drop;
                final float hz;
                final int phase;
                final float stageRevs;
                final int remain;
                final int segs;
                final float revsAbi;
                final float revsAbs;
                final long indexN;
                final long indexSigned;
                long tRel;
                long unix;
                // BLE 短帧：第 2 字段是带小数点的 rpm（无 t_ms）
                boolean shortBle = p.length <= 8 && p[1].indexOf('.') >= 0;
                if (shortBle) {
                    rpm = Float.parseFloat(p[1]);
                    dir = p.length > 2 ? (int) Float.parseFloat(p[2]) : 0;
                    armed = p.length > 3 && ((int) Float.parseFloat(p[3])) != 0;
                    phase = p.length > 4 ? (int) Float.parseFloat(p[4]) : 0;
                    remain = p.length > 5 ? (int) Float.parseFloat(p[5]) : 0;
                    logN = p.length > 6 ? (int) Float.parseFloat(p[6]) : 0;
                    drop = 0;
                    hz = 0f;
                    stageRevs = 0f;
                    segs = 0;
                    revsAbi = 0f;
                    revsAbs = 0f;
                    indexN = 0L;
                    indexSigned = 0L;
                    tRel = 0L;
                    unix = 0L;
                } else {
                    if (p.length < 8) return;
                    rpm = Float.parseFloat(p[2]);
                    dir = (int) Float.parseFloat(p[3]);
                    armed = ((int) Float.parseFloat(p[4])) != 0;
                    logN = (int) Float.parseFloat(p[5]);
                    drop = (int) Float.parseFloat(p[6]);
                    hz = Float.parseFloat(p[7]);
                    phase = p.length > 10 ? (int) Float.parseFloat(p[10]) : 0;
                    stageRevs = p.length > 12 ? Float.parseFloat(p[12]) : 0f;
                    remain = p.length > 13 ? (int) Float.parseFloat(p[13]) : 0;
                    segs = p.length > 14 ? (int) Float.parseFloat(p[14]) : 0;
                    revsAbi = p.length > 15 ? Float.parseFloat(p[15]) : 0f;
                    revsAbs = p.length > 16 ? Float.parseFloat(p[16]) : 0f;
                    indexN = p.length > 17 ? (long) Double.parseDouble(p[17]) : 0L;
                    indexSigned = p.length > 18 ? (long) Double.parseDouble(p[18]) : 0L;
                    tRel = p.length > 19 ? (long) Double.parseDouble(p[19]) : 0L;
                    unix = p.length > 20 ? (long) Double.parseDouble(p[20]) : 0L;
                }
                if (unix <= 0 && timeAnchorUnixMs > 0 && tRel >= 0) {
                    unix = timeAnchorUnixMs + tRel;
                }
                long finalUnix = unix;
                long finalTRel = tRel;
                // 遥测只刷新 UI；是否记录 / 何时记完一律由 ESP32 决定，手机不根据 phase 开倒计时
                ui.post(() -> listener.onLive(rpm, dir, armed, phase, logN, drop, hz,
                        stageRevs, remain, segs, revsAbi, revsAbs, indexN, indexSigned,
                        finalTRel, finalUnix));
            } catch (NumberFormatException ignored) {
            }
            return;
        }
        // 仅明确回传标记（不要匹配 # OK ble_ms / 普通 # 日志）
        if (line.startsWith("# DUMP BUSY")) {
            dumping = true;
            opQ.clear();
            ackQueued = false;
            ui.post(() -> listener.onInfo(line));
            status("板子准备回传…");
            return;
        }
        // —— DUMP BIN BLE（RAM snap hex）——
        if (line.startsWith("# SNAP ") && (line.contains("valid=") || line.contains("src="))) {
            // # SNAP src=RAM valid=1 n=… bytes=…
            snapQueryPending = false;
            ui.post(() -> listener.onInfo(line));
            boolean valid = line.contains("valid=1");
            int n = 0;
            for (String tok : line.replace("#", " ").trim().split("\\s+")) {
                if (tok.startsWith("n=")) {
                    try {
                        n = Integer.parseInt(tok.substring(2));
                    } catch (NumberFormatException ignored) {
                    }
                }
            }
            if (!connected) return;
            awaitingBindump = true;
            if (valid && n > 0) {
                status("SNAP valid · DUMP BIN BLE (RAM)");
                enqueueWrite("DUMP BIN BLE");
            } else {
                status("SNAP empty · DUMP BIN BLE SD");
                enqueueWrite("DUMP BIN BLE SD");
            }
            return;
        }
        if (line.startsWith("# MONITOR START count=")) {
            ui.post(() -> listener.onInfo(line));
            status(line.replace("# ", ""));
            return;
        }
        if (line.startsWith("# BIN BLE BEGIN")) {
            binBleActive = true;
            dumping = true;
            awaitingBindump = true;
            binBleBuf.reset();
            binBleExpect = 0;
            binBleChunks = 0;
            binBleNextSeq = 0;
            binBleLastPct = -1;
            binBleLastUiMs = 0L;
            lastBinRxMs = System.currentTimeMillis();
            binBleSrc = "RAM";
            ackQueued = false;
            for (String tok : line.replace("#", " ").trim().split("\\s+")) {
                if (tok.startsWith("bytes=")) {
                    try {
                        binBleExpect = Integer.parseInt(tok.substring(6));
                    } catch (NumberFormatException ignored) {
                    }
                } else if (tok.startsWith("src=")) {
                    binBleSrc = tok.substring(4);
                }
            }
            ui.post(() -> listener.onInfo(line));
            status("蓝牙拉 " + binBleSrc + "（Notify）… 0/"
                    + (binBleExpect > 0 ? binBleExpect : "?"));
            postBinProgress(0, binBleExpect, 0);
            queueDumpAck();
            return;
        }
        if (binBleActive && line.startsWith("B,")) {
            lastBinRxMs = System.currentTimeMillis();
            String[] parts = line.split(",", 3);
            if (parts.length >= 3) {
                int seq = -1;
                try {
                    seq = Integer.parseInt(parts[1].trim());
                } catch (NumberFormatException ignored) {
                }
                // Notify + GATT READ 常把同一 B,seq 送两次 → 字节翻倍 → CRC 失败
                if (seq >= 0 && seq < binBleNextSeq) {
                    queueDumpAck();
                    return;
                }
                if (seq >= 0) binBleNextSeq = seq + 1;
                boolean room = binBleExpect <= 0 || binBleBuf.size() < binBleExpect;
                if (room) {
                    try {
                        String hx = parts[2].trim();
                        int len = hx.length();
                        if ((len & 1) == 0 && len > 0) {
                            int need = len / 2;
                            if (binBleExpect > 0) {
                                int left = binBleExpect - binBleBuf.size();
                                if (need > left) need = left;
                            }
                            if (need > 0) {
                                byte[] chunk = new byte[need];
                                for (int i = 0; i < need; i++) {
                                    chunk[i] = (byte) Integer.parseInt(
                                            hx.substring(i * 2, i * 2 + 2), 16);
                                }
                                binBleBuf.write(chunk);
                                binBleChunks++;
                            }
                        }
                    } catch (Exception ignored) {
                    }
                }
                queueDumpAck();
                int got = binBleBuf.size();
                int pct;
                if (binBleExpect > 0) {
                    pct = Math.min(99, got * 100 / binBleExpect);
                } else {
                    pct = Math.min(99, Math.max(1, got / 512));
                }
                long now = System.currentTimeMillis();
                if (pct != binBleLastPct || (now - binBleLastUiMs) >= 120L || (binBleChunks % 10) == 0) {
                    binBleLastPct = pct;
                    binBleLastUiMs = now;
                    postBinProgress(got, binBleExpect, pct);
                    status("蓝牙 BIN(" + binBleSrc + ") " + got + "/"
                            + (binBleExpect > 0 ? binBleExpect : "?") + " (" + pct + "%)");
                }
            }
            return;
        }
        if (line.startsWith("# BIN BLE END")
                || (binBleActive && line.startsWith("# BIN BLE"))) {
            if (binBleActive) {
                byte[] raw = binBleBuf.toByteArray();
                binBleActive = false;
                dumping = false;
                ackQueued = false;
                queueDumpAck();
                // 若仍有重复残留，按 BEGIN 声明长度截断
                if (binBleExpect > 0 && raw.length > binBleExpect) {
                    final int before = raw.length;
                    final int after = binBleExpect;
                    ui.post(() -> listener.onInfo("# BIN trim " + before + "→" + after
                            + " (dedupe excess)"));
                    raw = java.util.Arrays.copyOf(raw, binBleExpect);
                }
                final byte[] parsed = raw;
                final int exp = binBleExpect > 0 ? binBleExpect : parsed.length;
                postBinProgress(parsed.length, exp, 100);
                final String src = binBleSrc;
                try {
                    SnapBinParser.Result r = SnapBinParser.parse(parsed);
                    awaitingBindump = false;
                    ui.post(() -> listener.onSnapBin(r.n, r.hz, r.rows));
                    status("BIN 完成 ✓ src=" + src + " n=" + r.n + " hz=" + r.hz);
                } catch (IllegalArgumentException e) {
                    awaitingBindump = false;
                    final String err = e.getMessage();
                    final int gotLen = parsed.length;
                    ui.post(() -> listener.onInfo("# BIN parse fail: " + err
                            + " got=" + gotLen + " exp=" + exp + " src=" + src));
                    status("BIN 校验失败（已收" + gotLen + "/" + exp + "）");
                }
            }
            ui.post(() -> listener.onInfo(line));
            return;
        }
        if (line.startsWith("# MONITOR armed") || line.startsWith("# MONITOR disarm")) {
            ui.post(() -> listener.onInfo(line));
            if (line.contains("armed")) {
                monitorSession = true;
                status("板子已武装 · 无实时转速（正常）");
            } else {
                monitorSession = false;
            }
            return;
        }
        if (line.startsWith("# CONFIRM") || line.startsWith("# QUIET")
                || line.startsWith("# STAGING")) {
            monitorSession = true;
            ui.post(() -> listener.onInfo(line));
            status(line.startsWith("# CONFIRM") ? "已触发记录（无实时转速=正常）" : "探测中（流分离）");
            return;
        }
        if (line.startsWith("# NOISE discard")) {
            ui.post(() -> listener.onInfo(line));
            return;
        }
        if (line.startsWith("# CD15") || line.startsWith("# RECORD done")
                || line.startsWith("# RECORD_DONE")) {
            snapDoneAtMs = System.currentTimeMillis();
            monitorSession = false;
            ui.post(() -> listener.onInfo(line));
            status("板子记完 · 等 SD SAVE → BLE PULL READY");
            return;
        }
        if (line.startsWith("# BLE PULL READY")) {
            ui.post(() -> listener.onInfo(line));
            schedulePcStylePull("PULL_READY");
            return;
        }
        if (line.startsWith("# AUTO DUMP READY") || line.startsWith("# DATA_START")) {
            // snap 记完后的 15s 内忽略旧 LOG 自动拉，避免 READ 冲 BLE
            if (inSnapSettleWindow() && line.startsWith("# AUTO DUMP READY")) {
                ui.post(() -> listener.onInfo("# skip AUTO DUMP (snap settle)"));
                status("跳过旧 LOG 自动拉（正在等 RAM）");
                return;
            }
            dumping = line.startsWith("# DATA_START") || dumping;
            ackQueued = false;
            if (line.startsWith("# DATA_START")) {
                dumping = true;
                dumpAccepted = 0;
                dumpBatch.clear();
                opQ.clear();
            }
            ui.post(() -> listener.onInfo(line));
            status(line.startsWith("# DATA_START") ? "正在慢速分包接收…" : "准备慢速拉取…");
            if (line.startsWith("# AUTO DUMP READY")) {
                ui.postDelayed(() -> {
                    if (connected && !inSnapSettleWindow()) send("READ");
                }, 350);
            }
            return;
        }
        if (line.startsWith("# DATA_END")) {
            flushDumpBatch();
            dumping = false;
            ackQueued = false;
            ui.post(() -> listener.onInfo(line + " accepted=" + dumpAccepted));
            status("回传完成 · 点 " + dumpAccepted);
            return;
        }
        if (line.startsWith("# AUTO DUMP BEGIN") || line.startsWith("# LOG DUMP")
                || line.startsWith("# BLE dump mode")) {
            dumping = true;
            dumpAccepted = 0;
            dumpBatch.clear();
            opQ.clear();
            ackQueued = false;
            ui.post(() -> listener.onInfo(line));
            status("正在慢速分包接收…");
            return;
        }
        if (line.startsWith("# AUTO DUMP END")) {
            flushDumpBatch();
            dumping = false;
            ackQueued = false;
            ui.post(() -> listener.onInfo(line + " accepted=" + dumpAccepted));
            status("回传完成 · 点 " + dumpAccepted);
            return;
        }
        if (line.startsWith("# DUMP PROG")) {
            ui.post(() -> listener.onInfo(line));
            queueDumpAck();
            return;
        }
        if (line.startsWith("# KEEPALIVE")) {
            return;
        }
        if (line.startsWith("D END")) {
            flushDumpBatch();
            int n = 0;
            String[] p = line.split("\\s+");
            if (p.length >= 3) {
                try {
                    n = Integer.parseInt(p[2]);
                } catch (NumberFormatException ignored) {
                }
            }
            int finalN = n;
            int accepted = dumpAccepted;
            ui.post(() -> {
                listener.onDumpEnd(finalN);
                listener.onInfo("# D END board=" + finalN + " phone_accepted=" + accepted);
            });
            return;
        }
        if (line.startsWith("I END")) {
            flushDumpBatch();
            dumping = false;
            ackQueued = false;
            int n = 0;
            String[] p = line.split("\\s+");
            if (p.length >= 3) {
                try {
                    n = Integer.parseInt(p[2]);
                } catch (NumberFormatException ignored) {
                }
            }
            int finalN = n;
            ui.post(() -> listener.onIndexEnd(finalN));
            return;
        }
        if (line.startsWith("I,") && !line.startsWith("I END")) {
            String[] p = line.split(",");
            // I,seq,t_ms,index_n,rpm_ab,rpm_i,dt_ms,seg,t_rel,unix
            if (p.length >= 8) {
                try {
                    int seq = Integer.parseInt(p[1]);
                    long t = (long) Double.parseDouble(p[2]);
                    long indexN = (long) Double.parseDouble(p[3]);
                    float rpmAb = Float.parseFloat(p[4]);
                    float rpmI = Float.parseFloat(p[5]);
                    long dt = (long) Double.parseDouble(p[6]);
                    int seg = (int) Float.parseFloat(p[7]);
                    long tRel = p.length >= 9 ? (long) Double.parseDouble(p[8]) : 0L;
                    long unix = p.length >= 10 ? (long) Double.parseDouble(p[9]) : 0L;
                    if (unix <= 0 && timeAnchorUnixMs > 0 && tRel >= 0) {
                        unix = timeAnchorUnixMs + tRel;
                    }
                    long finalUnix = unix;
                    ui.post(() -> listener.onIndexEvent(seq, t, indexN, rpmAb, rpmI, dt, seg, tRel,
                            finalUnix));
                    queueDumpAck();
                } catch (NumberFormatException ignored) {
                }
            }
            return;
        }
        if (line.startsWith("D,")) {
            String[] p = line.split(",");
            if (p.length >= 5) {
                try {
                    int idx = Integer.parseInt(p[1]);
                    long t = (long) Double.parseDouble(p[2]);
                    float rpm = Float.parseFloat(p[3]);
                    int dir = (int) Float.parseFloat(p[4]);
                    int seg = p.length >= 6 ? (int) Float.parseFloat(p[5]) : 1;
                    long indexN = p.length >= 7 ? (long) Double.parseDouble(p[6]) : 0L;
                    long tRel = p.length >= 8 ? (long) Double.parseDouble(p[7]) : 0L;
                    long unix = p.length >= 9 ? (long) Double.parseDouble(p[8]) : 0L;
                    if (unix <= 0 && timeAnchorUnixMs > 0 && tRel >= 0) {
                        unix = timeAnchorUnixMs + tRel;
                    }
                    dumpAccepted++;
                    dumpBatch.add(new Object[]{idx, t, rpm, dir, seg, indexN, tRel, unix});
                    if (dumpBatch.size() >= 64) {
                        flushDumpBatch();
                    }
                    queueDumpAck();
                } catch (NumberFormatException ignored) {
                }
            }
            return;
        }
        if (line.startsWith("# DUMP BIN BLE fail") || line.startsWith("# DUMP BIN BLE wait")
                || line.startsWith("# DUMP BIN BLE busy")
                || line.startsWith("# DUMP BIN BLE SD fail")) {
            binBleActive = false;
            dumping = false;
            awaitingBindump = false;
            ackQueued = false;
            ui.post(() -> listener.onInfo(line));
            status(line.replace("# ", ""));
            if (line.contains("wait") || line.contains("busy")) {
                scheduleDumpRetry(800);
            }
            return;
        }
        // 固件提示正在改拉 SD（随后会有 BIN BLE BEGIN src=SD）
        if (line.startsWith("# DUMP BIN BLE: empty") || line.startsWith("# DUMP BIN BLE SD file=")) {
            ui.post(() -> listener.onInfo(line));
            status(line.contains("file=") ? "正在从 SD 拉…" : "RAM 空 · 固件改拉 SD…");
            return;
        }
        if (line.startsWith("# SNAP DONE")) {
            snapDoneAtMs = System.currentTimeMillis();
            ui.post(() -> listener.onInfo(line));
            status("SNAP DONE · 等 ALIVE/SD/PULL READY（同 PC）");
            return;
        }
        if (line.startsWith("# SNAP DUMP READY")) {
            snapDoneAtMs = System.currentTimeMillis();
            ui.post(() -> listener.onInfo(line));
            status("RAM 就绪 · 板子自动 SD SAVE…");
            return;
        }
        if (line.startsWith("#")) {
            ui.post(() -> listener.onInfo(line));
        }
    }

    private void ingest(byte[] data) {
        if (data == null || data.length == 0) return;
        // 对齐 PC：GATT READ 常返回「上一次 Notify 的同一特征值」，必须丢弃，否则 B 包翻倍
        if (lastIngestPayload != null
                && lastIngestPayload.length == data.length
                && java.util.Arrays.equals(lastIngestPayload, data)) {
            return;
        }
        lastIngestPayload = java.util.Arrays.copyOf(data, data.length);
        // 先拼上、立刻拆完整行；禁止先截断（旧逻辑会丢掉尚未解析的 D 行）
        rxBuf.append(new String(data, StandardCharsets.UTF_8));
        int nl;
        while ((nl = indexOfNewline(rxBuf)) >= 0) {
            String line = rxBuf.substring(0, nl).replace("\r", "").trim();
            rxBuf.delete(0, nl + 1);
            if (!line.isEmpty()) handleLine(line);
        }
        // 仅丢弃异常超长的半行残片
        if (rxBuf.length() > 8192) {
            rxBuf.setLength(0);
        }
    }

    private static int indexOfNewline(StringBuilder sb) {
        for (int i = 0; i < sb.length(); i++) {
            if (sb.charAt(i) == '\n') return i;
        }
        return -1;
    }

    @SuppressLint("MissingPermission")
    private void finishNotifySetup() {
        if (notifyReady) return;
        notifyReady = true;
        telemCount = 0;
        lastTelemMs = 0;
        lastPollMs = 0;
        ui.post(() -> {
            listener.onConnected(true);
            status("已连接 · PC同款引导…");
        });
        // 对齐 PC ble_link：notify 后 sleep 250ms，再 TIME / BLE RATE / ABI?（间隔 80ms）
        timeAnchorUnixMs = System.currentTimeMillis();
        dumping = false;
        binBleActive = false;
        awaitingBindump = false;
        ackQueued = false;
        final long t = timeAnchorUnixMs;
        ui.postDelayed(() -> {
            if (!connected) return;
            enqueueWrite("TIME " + t);
        }, 250);
        ui.postDelayed(() -> {
            if (!connected) return;
            enqueueWrite("BLE RATE 10");
        }, 330);
        ui.postDelayed(() -> {
            if (!connected) return;
            enqueueWrite("ABI?");
            sessionConfigured = true;
        }, 410);
        // REC MS 仍由 MainActivity @600ms 发（PC @500ms）
        if (!sessionConfigured) {
            recMsPendingFirst = true;
        }
        ui.removeCallbacks(tickTask);
        ui.removeCallbacks(busyWatchdog);
        ui.postDelayed(tickTask, 250);
        ui.post(busyWatchdog);
    }

    private final BluetoothGattCallback gattCb = new BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                connected = true;
                dumping = false;
                status("已连接，协商 MTU…");
                mtuDone = false;
                // 对齐 PC：不请求 CONNECTION_PRIORITY_HIGH
                boolean req = g.requestMtu(247);
                ui.postDelayed(mtuFallback, 800);
                if (!req) {
                    mtuDone = true;
                    g.discoverServices();
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                boolean wasDumping = dumping || binBleActive;
                connected = false;
                notifyReady = false;
                dumping = false;
                binBleActive = false;
                awaitingBindump = false;
                pullScheduled = false;
                dumpRetryLeft = 0;
                ui.removeCallbacks(tickTask);
                ui.removeCallbacks(busyWatchdog);
                ui.removeCallbacks(mtuFallback);
                ui.removeCallbacks(dumpRetryTask);
                ui.removeCallbacks(pcPullTask);
                final String why = gattStatusText(status)
                        + (wasDumping ? " · 断在回传中" : " · 记录/空闲时段");
                ui.post(() -> {
                    listener.onConnected(false);
                    // 对齐 PC：不自动重连，需手动点连接
                    status("已断开（" + why + "）· 请手动重连（同 PC）");
                });
            }
        }

        @Override
        public void onMtuChanged(BluetoothGatt g, int mtu, int status) {
            mtuDone = true;
            ui.removeCallbacks(mtuFallback);
            status("MTU=" + mtu + "，发现服务…");
            g.discoverServices();
        }

        @SuppressLint("MissingPermission")
        @Override
        public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService svc = g.getService(NUS_SERVICE);
            if (svc == null) {
                status("未找到 NUS");
                disconnect();
                return;
            }
            rxChar = svc.getCharacteristic(NUS_RX);
            txChar = svc.getCharacteristic(NUS_TX);
            if (rxChar == null || txChar == null) {
                status("NUS 特征缺失");
                disconnect();
                return;
            }
            g.setCharacteristicNotification(txChar, true);
            BluetoothGattDescriptor cccd = txChar.getDescriptor(CCCD);
            if (cccd != null) {
                cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                if (!g.writeDescriptor(cccd)) {
                    finishNotifySetup();
                }
            } else {
                finishNotifySetup();
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            busy = false;
            finishNotifySetup();
            pump();
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic ch) {
            // 立刻拷贝：特征值缓冲会被后续 notify 覆盖（朋友2）
            byte[] v = ch.getValue();
            if (v != null) ingest(java.util.Arrays.copyOf(v, v.length));
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                            byte[] value) {
            if (value != null) ingest(java.util.Arrays.copyOf(value, value.length));
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                byte[] v = ch.getValue();
                if (v != null) ingest(java.util.Arrays.copyOf(v, v.length));
            }
            busy = false;
            pump();
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                         byte[] value, int status) {
            if (status == BluetoothGatt.GATT_SUCCESS && value != null) {
                ingest(java.util.Arrays.copyOf(value, value.length));
            }
            busy = false;
            pump();
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                          int status) {
            busy = false;
            pump();
        }
    };
}
