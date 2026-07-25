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
import java.util.Queue;
import java.util.UUID;

/**
 * Nordic UART → OEZ-ABI。连接后发 TIME 对时；少发 PING；MTU 协商失败也能发现服务。
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
    private final Queue<Op> opQ = new ArrayDeque<>();
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
    private long lastTelemMs;
    private int telemCount;
    private long lastPingMs;
    /** 发给板子的 TIME 锚点（毫秒），用于 unix=0 时本地推算 */
    private long timeAnchorUnixMs;
    private String lastAddress;
    private int autoReconnectLeft;
    private boolean ackQueued;

    private final Runnable tickTask = new Runnable() {
        @Override
        public void run() {
            if (!connected) return;
            if (dumping) {
                ui.postDelayed(this, 500);
                return;
            }
            long now = System.currentTimeMillis();
            if (now - lastTelemMs > 800) enqueueRead();
            // 很少发 PING，且固件 PONG 不再灌 BLE
            if (now - lastPingMs > 8000) {
                lastPingMs = now;
                enqueueWrite("PING");
            }
            String st;
            if (lastTelemMs == 0) {
                st = "已连接 · 等待转速遥测…";
            } else if (now - lastTelemMs > 3000) {
                st = String.format(Locale.US, "已连接 · 遥测中断(曾%d帧)，重试读…", telemCount);
            } else {
                st = String.format(Locale.US, "已连接 · 遥测正常 %d帧", telemCount);
            }
            status(st);
            ui.postDelayed(this, 350);
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

    public boolean hasAdapter() {
        return adapter != null && adapter.isEnabled();
    }

    public boolean isConnected() {
        return connected;
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
        autoReconnectLeft = 2;
        BluetoothDevice d = adapter.getRemoteDevice(address);
        status("连接 " + address + " …");
        lastTelemMs = 0;
        telemCount = 0;
        dumping = false;
        rxBuf.setLength(0);
        notifyReady = false;
        mtuDone = false;
        gatt = d.connectGatt(appCtx, false, gattCb, BluetoothDevice.TRANSPORT_LE);
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        autoReconnectLeft = 0;
        ui.removeCallbacks(tickTask);
        ui.removeCallbacks(busyWatchdog);
        ui.removeCallbacks(mtuFallback);
        connected = false;
        notifyReady = false;
        dumping = false;
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
                    || u.equals("READ") || u.startsWith("LOG READ"))) {
                return;
            }
        }
        enqueueWrite(cmd);
    }

    public void monitorStart() {
        send("MONITOR START");
    }

    public void monitorStop() {
        send("MONITOR STOP");
    }

    public void logClear() {
        send("LOG CLEAR");
    }

    public void logDump() {
        send("LOG DUMP");
    }

    private void status(String msg) {
        ui.post(() -> listener.onStatus(msg));
    }

    private void enqueueWrite(String line) {
        String s = line.endsWith("\n") ? line : line + "\n";
        opQ.offer(new Op(OpType.WRITE, s.getBytes(StandardCharsets.UTF_8)));
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
        if (line.startsWith("L,")) {
            if (dumping) {
                dumping = false;
                ackQueued = false;
                status("遥测已恢复");
            }
            lastTelemMs = System.currentTimeMillis();
            telemCount++;
            String[] p = line.split(",");
            if (p.length < 8) return;
            try {
                float rpm = Float.parseFloat(p[2]);
                int dir = (int) Float.parseFloat(p[3]);
                boolean armed = ((int) Float.parseFloat(p[4])) != 0;
                int logN = (int) Float.parseFloat(p[5]);
                int drop = (int) Float.parseFloat(p[6]);
                float hz = Float.parseFloat(p[7]);
                int phase = p.length > 10 ? (int) Float.parseFloat(p[10]) : 0;
                float stageRevs = p.length > 12 ? Float.parseFloat(p[12]) : 0f;
                int remain = p.length > 13 ? (int) Float.parseFloat(p[13]) : 0;
                int segs = p.length > 14 ? (int) Float.parseFloat(p[14]) : 0;
                float revsAbi = p.length > 15 ? Float.parseFloat(p[15]) : 0f;
                float revsAbs = p.length > 16 ? Float.parseFloat(p[16]) : 0f;
                long indexN = p.length > 17 ? (long) Double.parseDouble(p[17]) : 0L;
                long indexSigned = p.length > 18 ? (long) Double.parseDouble(p[18]) : 0L;
                long tRel = p.length > 19 ? (long) Double.parseDouble(p[19]) : 0L;
                long unix = p.length > 20 ? (long) Double.parseDouble(p[20]) : 0L;
                if (unix <= 0 && timeAnchorUnixMs > 0 && tRel >= 0) {
                    unix = timeAnchorUnixMs + tRel;
                }
                long finalUnix = unix;
                long finalTRel = tRel;
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
        if (line.startsWith("# RECORD done") || line.startsWith("# RECORD_DONE")) {
            ui.post(() -> listener.onInfo(line));
            status("记录完成…");
            // 对齐 00ACC：RECORD_DONE 后 300ms 发 READ
            ui.postDelayed(() -> {
                if (connected) send("READ");
            }, 300);
            return;
        }
        if (line.startsWith("# AUTO DUMP READY") || line.startsWith("# DATA_START")) {
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
                    if (connected) send("READ");
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
        if (line.startsWith("#")) {
            ui.post(() -> listener.onInfo(line));
        }
    }

    private void ingest(byte[] data) {
        if (data == null || data.length == 0) return;
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
        ui.post(() -> {
            listener.onConnected(true);
            status("已连接，发送对时…");
        });
        // 手机墙上时钟 → ESP32 会话锚点
        timeAnchorUnixMs = System.currentTimeMillis();
        dumping = false;
        ackQueued = false;
        enqueueWrite("TIME " + timeAnchorUnixMs);
        enqueueWrite("BLE RATE 10");
        enqueueWrite("ABI?");
        // REC MS 由 MainActivity 在连接成功后按 SeekBar 下发
        ui.removeCallbacks(tickTask);
        ui.removeCallbacks(busyWatchdog);
        ui.post(tickTask);
        ui.post(busyWatchdog);
    }

    private final BluetoothGattCallback gattCb = new BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                connected = true; // 允许后续 pump
                dumping = false;
                status("已连接，协商 MTU…");
                mtuDone = false;
                try {
                    g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH);
                } catch (Exception ignored) {
                }
                boolean req = g.requestMtu(247);
                ui.postDelayed(mtuFallback, 800);
                if (!req) {
                    mtuDone = true;
                    g.discoverServices();
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                boolean wasDumping = dumping;
                connected = false;
                notifyReady = false;
                dumping = false;
                ui.removeCallbacks(tickTask);
                ui.removeCallbacks(busyWatchdog);
                ui.removeCallbacks(mtuFallback);
                final String why = gattStatusText(status)
                        + (wasDumping ? " · 断在回传中" : " · 非回传时段");
                ui.post(() -> {
                    listener.onConnected(false);
                    status("已断开（" + why + "）");
                });
                // 回传冲垮时自动重连一两次
                if (autoReconnectLeft > 0 && lastAddress != null
                        && (wasDumping || status == 8 || status == 133)) {
                    autoReconnectLeft--;
                    final String addr = lastAddress;
                    ui.postDelayed(() -> {
                        status("自动重连剩余 " + autoReconnectLeft + " …");
                        BluetoothDevice d = adapter.getRemoteDevice(addr);
                        try {
                            if (gatt != null) {
                                try {
                                    gatt.close();
                                } catch (Exception ignored) {
                                }
                                gatt = null;
                            }
                            gatt = d.connectGatt(appCtx, false, gattCb, BluetoothDevice.TRANSPORT_LE);
                        } catch (Exception e) {
                            status("自动重连失败: " + e.getMessage());
                        }
                    }, 900);
                }
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
            ingest(ch.getValue());
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                            byte[] value) {
            ingest(value);
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) ingest(ch.getValue());
            busy = false;
            pump();
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g, BluetoothGattCharacteristic ch,
                                         byte[] value, int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) ingest(value);
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
