package com.oezcon.abirpmonitor;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.ArrayAdapter;
import android.widget.SeekBar;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.oezcon.abirpmonitor.databinding.ActivityMainBinding;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class MainActivity extends AppCompatActivity implements BleClient.Listener {
    private static final int REQ_BT = 1001;
    /** SeekBar: progress 0..25 → 500..3000 ms step 100 */
    private int recMs = 1000;

    /** 一段已接收数据 */
    static final class SegItem {
        final String label;
        final int boardSeg;
        final List<float[]> samples; // tMs,rpm,dir,seg,indexN,tRel,unix
        final List<float[]> indexEv;
        final long recvAt;

        SegItem(String label, int boardSeg, List<float[]> samples, List<float[]> indexEv) {
            this.label = label;
            this.boardSeg = boardSeg;
            this.samples = samples;
            this.indexEv = indexEv;
            this.recvAt = System.currentTimeMillis();
        }

        @Override
        public String toString() {
            return label;
        }
    }

    private ActivityMainBinding b;
    private BleClient ble;
    private final List<BleClient.DeviceItem> devices = new ArrayList<>();
    private ArrayAdapter<BleClient.DeviceItem> adapter;

    private final List<float[]> playback = new ArrayList<>();
    private final List<float[]> indexEvents = new ArrayList<>();
    private final List<SegItem> archive = new ArrayList<>();
    private ArrayAdapter<SegItem> segAdapter;
    private final Handler playHandler = new Handler(Looper.getMainLooper());
    private int playI;
    private boolean playing;
    private List<float[]> playRows = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        b = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(b.getRoot());

        ble = new BleClient(this, this);
        adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, devices);
        b.deviceSpinner.setAdapter(adapter);

        segAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, archive);
        b.segSpinner.setAdapter(segAdapter);

        // SeekBar 0..25 → 500..3000ms；默认 progress=5 → 1000ms
        b.seekRecMs.setMax(25);
        b.seekRecMs.setProgress(5);
        updateRecLabel(5);
        b.seekRecMs.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                updateRecLabel(progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                applyRecMsToBoard();
            }
        });

        b.btnScan.setOnClickListener(v -> {
            if (!ensureBtPerm()) return;
            if (!ble.hasAdapter()) {
                toast("请打开蓝牙");
                return;
            }
            ble.startScan(6000);
        });
        b.btnConnect.setOnClickListener(v -> {
            if (!ensureBtPerm()) return;
            if (ble.isConnected()) {
                ble.disconnect();
                b.btnConnect.setText("连接");
                return;
            }
            Object sel = b.deviceSpinner.getSelectedItem();
            if (!(sel instanceof BleClient.DeviceItem)) {
                toast("请先扫描并选择 OEZ-ABI");
                return;
            }
            ble.connect(((BleClient.DeviceItem) sel).address);
        });

        b.btnMonStart.setOnClickListener(v -> {
            if (!ble.isConnected()) {
                toast("请先连接");
                return;
            }
            ble.monitorStart();
        });
        b.btnMonStop.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.monitorStop();
        });
        b.btnClear.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.logClear();
        });
        b.btnRevsClear.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.send("REVS CLEAR");
        });
        b.btnDump.setOnClickListener(v -> {
            if (!ble.isConnected()) {
                toast("请先连接");
                return;
            }
            playback.clear();
            indexEvents.clear();
            b.playInfo.setText("拉取中…");
            ble.logDump();
        });
        b.btnViewSeg.setOnClickListener(v -> viewSelectedSegment());
        b.btnPlay.setOnClickListener(v -> startPlaybackSelected());
        b.btnLiveChart.setOnClickListener(v -> {
            playing = false;
            playHandler.removeCallbacksAndMessages(null);
            b.rpmChart.setLiveMode();
            b.playInfo.setText("已切回实时曲线");
        });
        b.btnApplyWin.setOnClickListener(v -> applyWindowFromEdit());
        b.btnWin03.setOnClickListener(v -> {
            b.editT0.setText("0.0");
            b.editT1.setText("0.3");
            if (!ensureSegmentLoaded()) return;
            b.rpmChart.setTimeWindow(0f, 0.3f);
            syncEditFromChart();
        });
        b.btnWinFull.setOnClickListener(v -> {
            if (!ensureSegmentLoaded()) return;
            float end = b.rpmChart.getDataEnd();
            b.rpmChart.setTimeWindow(0f, end);
            syncEditFromChart();
        });

        ensureBtPerm();
    }

    private void updateRecLabel(int progress) {
        recMs = 500 + progress * 100;
        b.recMsLabel.setText(String.format(Locale.US, "%.1f s", recMs / 1000f));
    }

    private void applyRecMsToBoard() {
        if (!ble.isConnected()) return;
        ble.send("REC MS " + recMs);
        toast("记录时长 " + String.format(Locale.US, "%.1f", recMs / 1000f) + " s");
    }

    private SegItem selectedSeg() {
        Object o = b.segSpinner.getSelectedItem();
        return (o instanceof SegItem) ? (SegItem) o : null;
    }

    private boolean ensureSegmentLoaded() {
        SegItem seg = selectedSeg();
        if (seg == null || seg.samples.isEmpty()) {
            toast("请先选择已接收数据段");
            return false;
        }
        if (!b.rpmChart.isSegmentMode()) {
            loadSegmentToChart(seg);
        }
        return true;
    }

    private void viewSelectedSegment() {
        SegItem seg = selectedSeg();
        if (seg == null || seg.samples.isEmpty()) {
            toast("暂无数据段");
            return;
        }
        playing = false;
        playHandler.removeCallbacksAndMessages(null);
        loadSegmentToChart(seg);
        applyWindowFromEdit();
        b.playInfo.setText(String.format(Locale.US, "查看 %s · %d 点 · 时长 %.3fs",
                seg.label, seg.samples.size(), b.rpmChart.getDataEnd()));
    }

    private void loadSegmentToChart(SegItem seg) {
        float t0 = seg.samples.get(0)[0];
        float[] ts = new float[seg.samples.size()];
        float[] rpm = new float[seg.samples.size()];
        for (int i = 0; i < seg.samples.size(); i++) {
            float[] s = seg.samples.get(i);
            ts[i] = (s[0] - t0) * 0.001f;
            rpm[i] = s[1];
        }
        b.rpmChart.setSegmentData(ts, rpm);
        syncEditFromChart();
    }

    private void applyWindowFromEdit() {
        if (!ensureSegmentLoaded()) return;
        try {
            float t0 = Float.parseFloat(b.editT0.getText().toString().trim());
            float t1 = Float.parseFloat(b.editT1.getText().toString().trim());
            b.rpmChart.setTimeWindow(t0, t1);
            syncEditFromChart();
        } catch (NumberFormatException e) {
            toast("时间格式错误，例如 0.0 与 0.3");
        }
    }

    private void syncEditFromChart() {
        b.editT0.setText(String.format(Locale.US, "%.3f", b.rpmChart.getWinStart()));
        b.editT1.setText(String.format(Locale.US, "%.3f", b.rpmChart.getWinEnd()));
    }

    private void archiveIncoming() {
        if (playback.isEmpty()) return;
        Map<Integer, List<float[]>> bySeg = new LinkedHashMap<>();
        for (float[] s : playback) {
            int seg = (int) s[3];
            List<float[]> list = bySeg.get(seg);
            if (list == null) {
                list = new ArrayList<>();
                bySeg.put(seg, list);
            }
            list.add(s);
        }
        Map<Integer, List<float[]>> byI = new LinkedHashMap<>();
        for (float[] ie : indexEvents) {
            int seg = (int) ie[5];
            List<float[]> list = byI.get(seg);
            if (list == null) {
                list = new ArrayList<>();
                byI.put(seg, list);
            }
            list.add(ie);
        }
        SimpleDateFormat fmt = new SimpleDateFormat("HH:mm:ss", Locale.getDefault());
        String stamp = fmt.format(new Date());
        for (Map.Entry<Integer, List<float[]>> e : bySeg.entrySet()) {
            List<float[]> rows = e.getValue();
            float dur = 0f;
            if (rows.size() >= 2) {
                dur = (rows.get(rows.size() - 1)[0] - rows.get(0)[0]) * 0.001f;
            }
            List<float[]> ie = byI.containsKey(e.getKey()) ? byI.get(e.getKey()) : new ArrayList<>();
            String label = String.format(Locale.US, "#%d 板seg%d · %d点 · %.2fs · I%d · %s",
                    archive.size() + 1, e.getKey(), rows.size(), dur, ie.size(), stamp);
            archive.add(0, new SegItem(label, e.getKey(), new ArrayList<>(rows), new ArrayList<>(ie)));
        }
        segAdapter.notifyDataSetChanged();
        if (!archive.isEmpty()) {
            b.segSpinner.setSelection(0);
        }
    }

    private void startPlaybackSelected() {
        SegItem seg = selectedSeg();
        if (seg == null || seg.samples.isEmpty()) {
            toast("请先选择数据段");
            return;
        }
        playRows = seg.samples;
        loadSegmentToChart(seg);
        b.rpmChart.setTimeWindow(0f, b.rpmChart.getDataEnd());
        syncEditFromChart();
        playing = true;
        playI = 0;
        playHandler.removeCallbacksAndMessages(null);
        tickPlay();
    }

    private void tickPlay() {
        if (!playing || playI >= playRows.size()) {
            playing = false;
            b.playInfo.setText(String.format(Locale.US, "回放结束 · 共 %d 点", playRows.size()));
            return;
        }
        float[] s = playRows.get(playI);
        playI++;
        float rpm = s[1];
        int dir = (int) s[2];
        long indexN = (long) s[4];
        long unix = s.length > 6 ? (long) s[6] : 0L;
        b.rpmValue.setText(String.format(Locale.US, "%.1f", rpm));
        b.dirText.setText(dir > 0 ? "方向 正转 +" : (dir < 0 ? "方向 反转 −" : "方向 静止"));
        b.revsText.setText(String.format(Locale.US, "I圈数 %d", indexN));
        if (unix > 0) {
            b.timeText.setText(formatUnix(unix));
        }
        float t0 = playRows.get(0)[0];
        float tRel = (s[0] - t0) * 0.001f;
        // 回放时窗口跟随当前点附近 0.3s
        float half = 0.15f;
        b.rpmChart.setTimeWindow(Math.max(0f, tRel - half), tRel + half);
        syncEditFromChart();
        b.playInfo.setText(String.format(Locale.US, "回放 %d/%d · t=%.3fs", playI, playRows.size(), tRel));
        long delay = 20;
        if (playI < playRows.size()) {
            long dt = (long) (playRows.get(playI)[0] - s[0]);
            if (dt > 5 && dt < 200) delay = dt;
        }
        playHandler.postDelayed(this::tickPlay, delay);
    }

    private boolean ensureBtPerm() {
        List<String> need = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= 31) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN)
                    != PackageManager.PERMISSION_GRANTED) {
                need.add(Manifest.permission.BLUETOOTH_SCAN);
            }
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                need.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
        } else {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                    != PackageManager.PERMISSION_GRANTED) {
                need.add(Manifest.permission.ACCESS_FINE_LOCATION);
            }
        }
        if (!need.isEmpty()) {
            ActivityCompat.requestPermissions(this, need.toArray(new String[0]), REQ_BT);
            return false;
        }
        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_BT) ensureBtPerm();
    }

    private void toast(String s) {
        Toast.makeText(this, s, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onStatus(String msg) {
        b.statusText.setText(msg);
    }

    @Override
    public void onDevices(List<BleClient.DeviceItem> list) {
        devices.clear();
        devices.addAll(list);
        adapter.notifyDataSetChanged();
    }

    @Override
    public void onConnected(boolean ok) {
        b.btnConnect.setText(ok ? "断开" : "连接");
        if (!ok) {
            b.statusText.setText("未连接 · 找 OEZ-ABI");
            b.dumpProgText.setText("");
        } else {
            // 连接后下发当前记录时长
            b.dumpProgText.setText("");
            new Handler(Looper.getMainLooper()).postDelayed(this::applyRecMsToBoard, 600);
        }
    }

    private String formatUnix(long unixMs) {
        if (unixMs <= 0) return "时间 未对时";
        SimpleDateFormat fmt = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());
        return "时间 " + fmt.format(new Date(unixMs));
    }

    @Override
    public void onLive(float rpm, int dir, boolean armed, int phase, int logN, int logDrop,
                       float measHz, float stageRevs, int remainMs, int segs,
                       float revsAbi, float revsAbs, long indexN, long indexSigned,
                       long tRelMs, long unixMs) {
        if (playing) return;
        b.rpmValue.setText(String.format(Locale.US, "%.1f", rpm));
        b.dirText.setText(dir > 0 ? "方向 正转 +" : (dir < 0 ? "方向 反转 −" : "方向 静止"));
        b.revsText.setText(String.format(Locale.US,
                "I圈数 %d（有符号 %d）\nA/B %.2f · 路程 %.2f",
                indexN, indexSigned, revsAbi, revsAbs));
        if (unixMs > 0) {
            b.timeText.setText(String.format(Locale.US, "%s · +%d ms",
                    formatUnix(unixMs), tRelMs));
        } else {
            b.timeText.setText(String.format(Locale.US, "相对 +%d ms（等待对时）", tRelMs));
        }
        if (!armed) {
            b.monText.setText("仅实时显示 · 未记录（点开始监控才写入板内）");
        } else if (phase == 1) {
            b.monText.setText(String.format(Locale.US, "记录中·临时池 · 已转 %.2f 周", stageRevs));
        } else if (phase == 2) {
            b.monText.setText(String.format(Locale.US, "记录中·已确认 · 剩余 %d ms", remainMs));
        } else {
            b.monText.setText("已武装 · 等待 |RPM|>20 开始记（转速仍实时）");
        }
        b.logText.setText(String.format(Locale.US, "log_n=%d drop=%d segs=%d meas≈%.0f",
                logN, logDrop, segs, measHz));
        if (!b.rpmChart.isSegmentMode()) {
            b.rpmChart.addSample(rpm);
        }
    }

    @Override
    public void onDumpSample(int idx, long tMs, float rpm, int dir, int seg, long indexN,
                             long tRelMs, long unixMs) {
        playback.add(new float[]{tMs, rpm, dir, seg, indexN, tRelMs, unixMs});
    }

    @Override
    public void onIndexEvent(int seq, long tMs, long indexN, float rpmAb, float rpmI, long dtMs,
                             int seg, long tRelMs, long unixMs) {
        indexEvents.add(new float[]{tMs, indexN, rpmAb, rpmI, dtMs, seg, tRelMs, unixMs});
    }

    @Override
    public void onDumpEnd(int n) {
        b.playInfo.setText(String.format(Locale.US, "已拉取 %d 点，等待 I 事件…", playback.size()));
    }

    @Override
    public void onIndexEnd(int n) {
        archiveIncoming();
        b.playInfo.setText(String.format(Locale.US,
                "已归档 · 共 %d 段可选 · 本次 I跳动 %d", archive.size(), indexEvents.size()));
        toast("已接收并归档 " + archive.size() + " 段");
        if (!archive.isEmpty()) {
            viewSelectedSegment();
        }
    }

    @Override
    public void onInfo(String line) {
        if (line.contains("TIME ok") || line.contains("SESSION")) {
            b.monText.setText(line.replace("# ", ""));
        } else if (line.contains("REC MS=")) {
            b.monText.setText(line.replace("# ", ""));
        } else if (line.contains("phone_accepted") || line.contains("expect_D")) {
            b.playInfo.setText(line.replace("# ", ""));
            if (line.contains("expect_D")) {
                b.dumpProgText.setText("预计接收… " + line.replace("# ", ""));
            }
        } else if (line.contains("DUMP BUSY")) {
            b.monText.setText("记录完成 · 准备回传，请保持连接");
            b.dumpProgText.setText("回传准备中…");
        } else if (line.contains("AUTO DUMP READY")) {
            b.monText.setText("记录完成 · 即将慢速拉取");
            b.dumpProgText.setText("准备拉取… " + line.replace("# ", ""));
            toast("记录完成，慢速拉取中…请保持连接");
        } else if (line.contains("DUMP PROG")) {
            // # DUMP PROG n=20 total=300 pct=6
            int n = -1, tot = -1, pct = -1;
            for (String tok : line.replace("#", " ").trim().split("\\s+")) {
                if (tok.startsWith("n=")) {
                    try { n = Integer.parseInt(tok.substring(2)); } catch (Exception ignored) {}
                } else if (tok.startsWith("total=")) {
                    try { tot = Integer.parseInt(tok.substring(6)); } catch (Exception ignored) {}
                } else if (tok.startsWith("pct=")) {
                    try { pct = Integer.parseInt(tok.substring(4)); } catch (Exception ignored) {}
                }
            }
            if (pct < 0 && n >= 0 && tot > 0) pct = n * 100 / tot;
            String msg = String.format(Locale.US, "回传进度 %s/%s  (%d%%)",
                    n >= 0 ? String.valueOf(n) : "?",
                    tot >= 0 ? String.valueOf(tot) : "?",
                    Math.max(pct, 0));
            b.dumpProgText.setText(msg);
            b.playInfo.setText(msg);
        } else if (line.contains("AUTO DUMP BEGIN")) {
            playback.clear();
            indexEvents.clear();
            playing = false;
            b.playInfo.setText("板子自动推送数据中…");
            b.dumpProgText.setText("回传 0%");
            toast("记录完成，慢速接收中…");
        } else if (line.contains("AUTO DUMP END")) {
            if (!playback.isEmpty() && (archive.isEmpty()
                    || archive.get(0).samples.size() != playback.size())) {
                archiveIncoming();
            }
            b.playInfo.setText(String.format(Locale.US, "自动接收完成 · 档案 %d 段", archive.size()));
            b.dumpProgText.setText("回传完成");
        } else if (line.contains("CONFIRM")) {
            b.monText.setText("已确认真实转动");
        } else if (line.contains("NOISE")) {
            b.monText.setText("噪声：临时池已丢弃");
        } else if (line.contains("RECORD done")) {
            b.monText.setText("本段已存盘 · 即将回传");
        } else if (line.contains("disarm") || line.contains("MONITOR disarm")) {
            b.monText.setText("已停记 · 转速仍实时刷新");
        } else if (line.contains("armed")) {
            b.monText.setText("已武装：等待 |RPM|>20（转速仍实时）");
        }
    }

    @Override
    protected void onDestroy() {
        playing = false;
        playHandler.removeCallbacksAndMessages(null);
        ble.disconnect();
        super.onDestroy();
    }
}
