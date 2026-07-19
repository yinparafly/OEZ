package com.oezcon.motorblecontrol;

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
 * Nordic UART 客户端，对接 ESP32 OEZ-RPM（不改固件）。
 * 优先 Notify；若无推送则定时 READ TX（与 PC 端相同兜底）。
 */
public class BleUartClient {
    public static final UUID NUS_SERVICE =
            UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID NUS_RX =
            UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID NUS_TX =
            UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
    public static final UUID CCCD =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private enum OpType { WRITE, READ }

    private static final class Op {
        final OpType type;
        final byte[] data;

        Op(OpType type, byte[] data) {
            this.type = type;
            this.data = data;
        }
    }

    public interface Listener {
        void onStatus(String msg);

        void onDevices(List<DeviceItem> devices);

        void onConnected(boolean ok);

        void onRpm(float rpm, float target);
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

    private final Context appCtx;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final Listener listener;
    private final Map<String, DeviceItem> found = new LinkedHashMap<>();
    private final Queue<Op> opQ = new ArrayDeque<>();
    private boolean busy;
    private int rxLines;
    private int rpmUpdates;

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rxChar;
    private BluetoothGattCharacteristic txChar;
    private boolean connected;
    private String rxBuf = "";
    private long lastPushMs; // notify 或成功解析到转速

    private final Runnable tickTask = new Runnable() {
        @Override
        public void run() {
            if (!connected) return;
            long now = System.currentTimeMillis();
            // 保活
            enqueueWrite("PING");
            // 无推送时 100ms 级读 TX（固件 setValue 后可读）
            if (now - lastPushMs > 300) {
                enqueueRead();
            }
            status(String.format(Locale.US,
                    "已连接 · 收行%d · 转速更新%d%s",
                    rxLines, rpmUpdates,
                    (now - lastPushMs > 2000 ? " · 等待遥测…" : "")));
            ui.postDelayed(this, 400);
        }
    };

    private final Runnable busyWatchdog = new Runnable() {
        @Override
        public void run() {
            if (!connected) return;
            if (busy) {
                // 防止 write/read 回调丢失导致永久卡死
                busy = false;
                pump();
            }
            ui.postDelayed(this, 1500);
        }
    };

    public BleUartClient(Context ctx, Listener listener) {
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

    @SuppressLint("MissingPermission")
    public void startScan(long ms) {
        found.clear();
        emitDevices();
        if (scanner == null) {
            status("无蓝牙适配器");
            return;
        }
        status("扫描 OEZ-RPM …");
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();
        try {
            scanner.startScan(null, settings, scanCb);
        } catch (SecurityException e) {
            status("缺少蓝牙权限");
            return;
        }
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
        status(found.isEmpty() ? "未发现 OEZ-RPM" : ("发现 " + found.size() + " 台"));
    }

    private final ScanCallback scanCb = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice d = result.getDevice();
            String name = result.getScanRecord() != null
                    ? result.getScanRecord().getDeviceName() : null;
            if (name == null) {
                try {
                    name = d.getName();
                } catch (SecurityException e) {
                    name = null;
                }
            }
            if (name == null) return;
            if (!name.toUpperCase(Locale.US).contains("OEZ")) return;
            found.put(d.getAddress(), new DeviceItem(d.getAddress(), name));
            emitDevices();
        }
    };

    @SuppressLint("MissingPermission")
    public void connect(String address) {
        disconnect();
        if (adapter == null) {
            status("无蓝牙");
            return;
        }
        BluetoothDevice dev = adapter.getRemoteDevice(address);
        status("连接 " + address + " …");
        try {
            gatt = dev.connectGatt(appCtx, false, gattCb, BluetoothDevice.TRANSPORT_LE);
        } catch (SecurityException e) {
            status("缺少蓝牙连接权限");
        }
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        ui.removeCallbacks(tickTask);
        ui.removeCallbacks(busyWatchdog);
        connected = false;
        busy = false;
        opQ.clear();
        rxChar = null;
        txChar = null;
        if (gatt != null) {
            try {
                gatt.disconnect();
                gatt.close();
            } catch (Exception ignored) {
            }
            gatt = null;
        }
        ui.post(() -> listener.onConnected(false));
    }

    public boolean isConnected() {
        return connected;
    }

    public void send(String line) {
        enqueueWrite(line);
    }

    public void setRpm(float rpm) {
        if (rpm < 0) rpm = 0;
        if (rpm > 6000) rpm = 6000;
        send(String.format(Locale.US, "RPM %.0f", rpm));
    }

    public void startMotor() {
        send("START");
    }

    public void stopMotor() {
        send("STOP");
    }

    private void enqueueWrite(String line) {
        if (!connected && rxChar == null) {
            // 允许连接完成后队列里已有指令；未连接时丢掉
        }
        if (gatt == null || rxChar == null) return;
        byte[] data = (line.trim() + "\n").getBytes(StandardCharsets.UTF_8);
        opQ.offer(new Op(OpType.WRITE, data));
        pump();
    }

    private void enqueueRead() {
        if (gatt == null || txChar == null) return;
        // 避免读队列堆积
        for (Op op : opQ) {
            if (op.type == OpType.READ) return;
        }
        opQ.offer(new Op(OpType.READ, null));
        pump();
    }

    @SuppressLint("MissingPermission")
    private void pump() {
        if (busy || opQ.isEmpty() || gatt == null) return;
        Op op = opQ.poll();
        if (op == null) return;
        busy = true;
        try {
            boolean ok;
            if (op.type == OpType.WRITE) {
                if (rxChar == null) {
                    busy = false;
                    return;
                }
                if (Build.VERSION.SDK_INT >= 33) {
                    int r = gatt.writeCharacteristic(rxChar, op.data,
                            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
                    ok = (r == BluetoothGatt.GATT_SUCCESS);
                } else {
                    rxChar.setValue(op.data);
                    rxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
                    ok = gatt.writeCharacteristic(rxChar);
                }
            } else {
                if (txChar == null) {
                    busy = false;
                    return;
                }
                ok = gatt.readCharacteristic(txChar);
            }
            if (!ok) {
                busy = false;
                ui.postDelayed(this::pump, 30);
            }
        } catch (SecurityException e) {
            busy = false;
        }
    }

    private void opDone() {
        busy = false;
        pump();
    }

    private final BluetoothGattCallback gattCb = new BluetoothGattCallback() {
        @Override
        @SuppressLint("MissingPermission")
        public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                status("已连接，发现服务…");
                g.discoverServices();
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connected = false;
                busy = false;
                opQ.clear();
                ui.removeCallbacks(tickTask);
                ui.removeCallbacks(busyWatchdog);
                status("已断开");
                ui.post(() -> listener.onConnected(false));
            }
        }

        @Override
        @SuppressLint("MissingPermission")
        public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService svc = g.getService(NUS_SERVICE);
            if (svc == null) {
                status("无 NUS 服务（确认 ESP32 已烧 BLE 固件）");
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
                if (Build.VERSION.SDK_INT >= 33) {
                    g.writeDescriptor(cccd, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                } else {
                    cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    g.writeDescriptor(cccd);
                }
            } else {
                finishConnect();
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            finishConnect();
        }

        private void finishConnect() {
            connected = true;
            busy = false;
            rxLines = 0;
            rpmUpdates = 0;
            lastPushMs = 0;
            status("已连接，开始收转速…");
            ui.post(() -> listener.onConnected(true));
            ui.removeCallbacks(tickTask);
            ui.removeCallbacks(busyWatchdog);
            ui.post(tickTask);
            ui.post(busyWatchdog);
            ui.postDelayed(() -> enqueueWrite("BLE?"), 200);
            // 立刻读几次，尽快出转速
            ui.postDelayed(() -> enqueueRead(), 350);
            ui.postDelayed(() -> enqueueRead(), 600);
            ui.postDelayed(() -> enqueueRead(), 900);
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt g,
                                          BluetoothGattCharacteristic characteristic,
                                          int status) {
            opDone();
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g,
                                            BluetoothGattCharacteristic characteristic) {
            handleTxBytes(characteristic.getValue());
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt g,
                                            BluetoothGattCharacteristic characteristic,
                                            byte[] value) {
            handleTxBytes(value);
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                handleTxBytes(characteristic.getValue());
            }
            opDone();
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt g,
                                         BluetoothGattCharacteristic characteristic,
                                         byte[] value,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                handleTxBytes(value);
            }
            opDone();
        }
    };

    private void handleTxBytes(byte[] value) {
        if (value == null || value.length == 0) return;
        String chunk = new String(value, StandardCharsets.UTF_8);
        // READ 时常是一整行（可能无 \\n）
        if (!chunk.contains("\n") && !chunk.contains("\r")) {
            String t = chunk.trim();
            if (!t.isEmpty()) {
                rxLines++;
                parseLine(t);
            }
            return;
        }
        rxBuf += chunk;
        int idx;
        while ((idx = indexOfNl(rxBuf)) >= 0) {
            String line = rxBuf.substring(0, idx).replace("\r", "").trim();
            rxBuf = rxBuf.substring(idx + 1);
            if (!line.isEmpty()) {
                rxLines++;
                parseLine(line);
            }
        }
    }

    private static int indexOfNl(String s) {
        int a = s.indexOf('\n');
        int b = s.indexOf('\r');
        if (a < 0) return b;
        if (b < 0) return a;
        return Math.min(a, b);
    }

    private void parseLine(String line) {
        if (line.startsWith("#")) return;
        try {
            String[] p = line.split(",");
            float rpm;
            float target = 0f;
            if (p.length >= 3 && p[0].trim().equalsIgnoreCase("B")) {
                // B,t_ms,rpm,pulse,target,mode,run
                rpm = Float.parseFloat(p[2].trim());
                if (p.length >= 5) target = Float.parseFloat(p[4].trim());
            } else if (p.length >= 5) {
                rpm = Float.parseFloat(p[4].trim());
                if (p.length >= 11) target = Float.parseFloat(p[10].trim());
            } else {
                return;
            }
            final float r = Math.abs(rpm);
            final float t = target;
            lastPushMs = System.currentTimeMillis();
            rpmUpdates++;
            ui.post(() -> listener.onRpm(r, t));
        } catch (Exception ignored) {
        }
    }

    private void status(String msg) {
        ui.post(() -> listener.onStatus(msg));
    }

    private void emitDevices() {
        List<DeviceItem> list = new ArrayList<>(found.values());
        ui.post(() -> listener.onDevices(list));
    }
}
