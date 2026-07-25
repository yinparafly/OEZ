package com.oezcon.abirpmonitor;

import android.Manifest;
import android.content.ActivityNotFoundException;
import android.content.ClipData;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.provider.DocumentsContract;
import android.widget.ArrayAdapter;
import android.widget.SeekBar;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.FileProvider;

import com.oezcon.abirpmonitor.databinding.ActivityMainBinding;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
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
            if (ble.isMonitorOrPullBusy()) {
                toast("正在监控/拉数中，勿再次「开始监控」（会清 RAM）");
                return;
            }
            ble.monitorStart();
            b.btnMonStart.setEnabled(false);
            toast("已武装：实时转速已关，请加油");
            b.monText.setText("监控中 · 无实时转速（正常）· 等触发事件");
            b.rpmValue.setText("--");
        });
        b.btnMonStop.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.monitorStop();
            b.btnMonStart.setEnabled(true);
        });
        b.btnClear.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.logClear();
        });
        b.btnDiag.setOnClickListener(v -> {
            if (!ble.isConnected()) {
                toast("请先连接");
                return;
            }
            ble.send("DIAG");
            toast("已请求诊断");
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
            b.playInfo.setText("拉取 RAM，空则自动拉 SD…");
            b.dumpProgText.setText("准备下载… 0%");
            b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
            b.dumpProgressBar.setProgress(0);
            ble.dumpBinBle();
        });
        b.btnDump.setOnLongClickListener(v -> {
            if (!ble.isConnected()) {
                toast("请先连接");
                return true;
            }
            playback.clear();
            indexEvents.clear();
            b.playInfo.setText("强制从 SD 拉最新 snap…");
            b.dumpProgText.setText("SD→BLE… 0%");
            b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
            b.dumpProgressBar.setProgress(0);
            ble.dumpBinBleSd();
            toast("长按：强制拉 SD");
            return true;
        });
        b.btnShare.setOnClickListener(v -> shareCurrentData());
        b.btnOpenFolder.setOnClickListener(v -> openExportFolder());
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

    private File exportDir() {
        // 公共 Download/AbiRpmMonitor —— 文件管理器能看到
        File pub = new File(
                Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS),
                "AbiRpmMonitor");
        if (pub.exists() || pub.mkdirs()) {
            return pub;
        }
        File priv = new File(getExternalFilesDir(null), "exports");
        //noinspection ResultOfMethodCallIgnored
        priv.mkdirs();
        return priv;
    }

    private boolean ensureStoragePerm() {
        if (Build.VERSION.SDK_INT >= 29) {
            return true;
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                == PackageManager.PERMISSION_GRANTED) {
            return true;
        }
        ActivityCompat.requestPermissions(this,
                new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1002);
        toast("请允许存储权限后再试");
        return false;
    }

    private Uri uriForFile(File f) {
        return FileProvider.getUriForFile(this, getPackageName() + ".fileprovider", f);
    }

    /** 导出当前选中段（或最新 playback）为 CSV + 曲线 PNG */
    private List<File> exportCurrentFiles() {
        List<float[]> rows;
        String tag;
        SegItem seg = selectedSeg();
        if (seg != null && !seg.samples.isEmpty()) {
            rows = seg.samples;
            tag = "seg" + seg.boardSeg;
        } else if (!playback.isEmpty()) {
            rows = playback;
            tag = "snap";
        } else {
            return null;
        }
        SimpleDateFormat fmt = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
        String stamp = fmt.format(new Date());
        File dir = exportDir();
        if (!dir.exists() && !dir.mkdirs()) {
            toast("无法创建导出目录");
            return null;
        }
        List<File> out = new ArrayList<>();
        try {
            File csv = new File(dir, "abi_" + tag + "_" + stamp + ".csv");
            try (OutputStreamWriter w = new OutputStreamWriter(new FileOutputStream(csv),
                    StandardCharsets.UTF_8)) {
                w.write("t_ms,rpm,dir,seg,index_n,t_rel,unix_ms\n");
                for (float[] r : rows) {
                    long tRel = r.length > 5 ? (long) r[5] : 0L;
                    long unix = r.length > 6 ? (long) r[6] : 0L;
                    w.write(String.format(Locale.US, "%.3f,%.2f,%d,%d,%.0f,%d,%d\n",
                            r[0], r[1], (int) r[2], (int) r[3], r[4], tRel, unix));
                }
            }
            out.add(csv);

            int wpx = Math.max(b.rpmChart.getWidth(), 800);
            int hpx = Math.max(b.rpmChart.getHeight(), 400);
            try {
                Bitmap bmp = Bitmap.createBitmap(wpx, hpx, Bitmap.Config.ARGB_8888);
                Canvas canvas = new Canvas(bmp);
                if (b.rpmChart.getWidth() > 0 && b.rpmChart.getHeight() > 0) {
                    b.rpmChart.draw(canvas);
                } else {
                    canvas.drawColor(0xFF1A1A1A);
                }
                File png = new File(dir, "abi_" + tag + "_" + stamp + ".png");
                try (FileOutputStream fos = new FileOutputStream(png)) {
                    bmp.compress(Bitmap.CompressFormat.PNG, 95, fos);
                }
                bmp.recycle();
                out.add(png);
            } catch (Exception ignorePng) {
                // 仅 CSV 也可分享
            }
        } catch (Exception e) {
            toast("导出失败: " + e.getMessage());
            return null;
        }
        return out;
    }

    private void shareCurrentData() {
        if ((selectedSeg() == null || selectedSeg().samples.isEmpty()) && playback.isEmpty()) {
            toast("没有可发送的数据，请先拉取");
            return;
        }
        if (!ensureStoragePerm()) return;
        if (selectedSeg() != null) {
            try {
                loadSegmentToChart(selectedSeg());
            } catch (Exception ignored) {
            }
        }
        b.getRoot().post(() -> {
            try {
                List<File> files = exportCurrentFiles();
                if (files == null || files.isEmpty()) {
                    toast("导出失败");
                    return;
                }
                // 只发 CSV 数据包，不发 PNG
                File csv = null;
                for (File f : files) {
                    if (f.getName().endsWith(".csv")) {
                        csv = f;
                        break;
                    }
                }
                if (csv == null) {
                    toast("没有可发送的 CSV");
                    return;
                }
                Uri mainUri = uriForFile(csv);
                Intent intent = new Intent(Intent.ACTION_SEND);
                intent.setType("text/csv");
                intent.putExtra(Intent.EXTRA_STREAM, mainUri);
                intent.putExtra(Intent.EXTRA_SUBJECT, "ABI 转速数据");
                intent.putExtra(Intent.EXTRA_TEXT,
                        "ABI 数据包: " + csv.getName() + "\n已保存到「下载/AbiRpmMonitor」");
                intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
                intent.setClipData(ClipData.newRawUri("abi", mainUri));

                Intent chooser = Intent.createChooser(intent, "发送数据包");
                List<ResolveInfo> res = getPackageManager().queryIntentActivities(intent,
                        PackageManager.MATCH_DEFAULT_ONLY);
                for (ResolveInfo ri : res) {
                    grantUriPermission(ri.activityInfo.packageName, mainUri,
                            Intent.FLAG_GRANT_READ_URI_PERMISSION);
                }
                startActivity(chooser);
                toast("发送 CSV: " + csv.getName());
            } catch (Exception e) {
                toast("发送失败: " + e.getMessage());
            }
        });
    }

    private void openExportFolder() {
        if (!ensureStoragePerm()) return;
        File dir = exportDir();
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        File tip = new File(dir, "请在此查看导出的CSV和PNG.txt");
        try (OutputStreamWriter w = new OutputStreamWriter(new FileOutputStream(tip),
                StandardCharsets.UTF_8)) {
            w.write("目录: 手机存储 / Download / AbiRpmMonitor\n");
            w.write(dir.getAbsolutePath());
            w.write("\n");
        } catch (Exception ignored) {
        }

        // 优先：DocumentsUI 直接打开 Download/AbiRpmMonitor
        try {
            Uri doc = Uri.parse(
                    "content://com.android.externalstorage.documents/document/primary:Download%2FAbiRpmMonitor");
            Intent intent = new Intent(Intent.ACTION_VIEW);
            intent.addCategory(Intent.CATEGORY_DEFAULT);
            intent.setDataAndType(doc, DocumentsContract.Document.MIME_TYPE_DIR);
            intent.putExtra(DocumentsContract.EXTRA_INITIAL_URI, doc);
            intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(intent);
            toast("正在打开 下载/AbiRpmMonitor");
            return;
        } catch (Exception ignored) {
        }

        try {
            Intent dl = new Intent(android.app.DownloadManager.ACTION_VIEW_DOWNLOADS);
            dl.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(dl);
            toast("请进入文件夹 AbiRpmMonitor");
            return;
        } catch (Exception ignored) {
        }

        try {
            Uri uri = uriForFile(tip);
            Intent intent = new Intent(Intent.ACTION_VIEW);
            intent.setDataAndType(uri, "text/plain");
            intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            startActivity(Intent.createChooser(intent, "打开导出目录中的说明"));
            toast("文件在: 下载/AbiRpmMonitor");
        } catch (Exception e) {
            toast("请用文件管理打开: 下载/AbiRpmMonitor");
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
            b.dumpProgressBar.setVisibility(android.view.View.GONE);
            b.btnMonStart.setEnabled(true);
        } else {
            b.dumpProgText.setText("");
            b.dumpProgressBar.setVisibility(android.view.View.GONE);
            b.btnMonStart.setEnabled(!ble.isMonitorOrPullBusy());
            // 仅首次连接自动发 REC MS；重连不发，避免冲掉板内正在记/已记数据
            if (ble.shouldAutoSendRecMs()) {
                new Handler(Looper.getMainLooper()).postDelayed(() -> {
                    if (ble.isConnected() && ble.shouldAutoSendRecMs()) {
                        applyRecMsToBoard();
                        ble.clearRecMsPendingFirst();
                    }
                }, 600);
            } else {
                b.statusText.setText("重连 · 保留板内记录（未重发时长设定）");
            }
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
            b.monText.setText("已武装 · 等待 |RPM|>10 且 I 过1圈（转速仍实时）");
        }
        b.logText.setText(String.format(Locale.US, "RAM点数=%d drop=%d segs=%d meas≈%.0f",
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
    public void onSnapBin(int n, int hz, java.util.List<float[]> rows) {
        playback.clear();
        indexEvents.clear();
        playback.addAll(rows);
        archiveIncoming();
        b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
        b.dumpProgressBar.setProgress(100);
        b.dumpProgText.setText(String.format(Locale.US, "下载完成 ✓  %d 点 @ %dHz", n, hz));
        b.playInfo.setText(String.format(Locale.US, "已取回 · %d 点 · 档案 %d 段（RAM空则来自SD）",
                n, archive.size()));
        b.monText.setText("下载完成");
        toast("下载完成，共 " + n + " 个点");
        b.btnMonStart.setEnabled(true);
        // 自动落盘，方便随后「发送数据」
        b.rpmChart.post(() -> {
            List<File> files = exportCurrentFiles();
            if (files != null && !files.isEmpty()) {
                b.playInfo.setText(String.format(Locale.US,
                        "已取回 %d 点 · 已保存 %d 文件到 exports", n, files.size()));
            }
        });
        if (!archive.isEmpty()) {
            viewSelectedSegment();
        }
    }

    @Override
    public void onBinBleProgress(int gotBytes, int expectBytes, int pct) {
        b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
        b.dumpProgressBar.setProgress(Math.max(0, Math.min(100, pct)));
        String exp = expectBytes > 0 ? String.valueOf(expectBytes) : "?";
        String msg;
        if (pct >= 100) {
            msg = String.format(Locale.US, "下载完成  %d/%s 字节  100%%", gotBytes, exp);
        } else {
            msg = String.format(Locale.US, "下载中…  %d/%s 字节  %d%%", gotBytes, exp, pct);
        }
        b.dumpProgText.setText(msg);
        b.playInfo.setText(msg);
        if (pct > 0 && pct < 100) {
            b.monText.setText("蓝牙取数 " + pct + "%");
        }
    }

    @Override
    public void onPullCountdown(int secLeft) {
        if (secLeft < 0) {
            return;
        }
        // 对齐 PC：不再 15s 倒数；0 = 即将/开始 SNAP?→DUMP BIN BLE
        b.monText.setText("BLE PULL READY · 即将拉取 RAM（同 PC）");
        b.dumpProgText.setText("PC同款拉取… 0%");
        b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
        b.dumpProgressBar.setProgress(0);
        b.playInfo.setText("SNAP? → DUMP BIN BLE");
    }

    @Override
    public void onInfo(String line) {
        if (line.contains("TIME ok") || line.contains("SESSION")) {
            b.monText.setText(line.replace("# ", ""));
        } else if (line.contains("REC MS=")) {
            b.monText.setText(line.replace("# ", ""));
        } else if (line.contains("CD15") || line.contains("SNAP DONE")
                || (line.contains("RECORD done") && line.contains("SD SAVE"))) {
            toast("记完 · 等 BLE PULL READY（同 PC）");
        } else if (line.contains("BLE PULL READY")) {
            toast("即将拉取 RAM（同 PC）");
            b.monText.setText("SD 已存 · 即将蓝牙拉取 RAM（同 PC）");
            playback.clear();
            indexEvents.clear();
        } else if (line.startsWith("# MON ") || line.contains("# MON phase=")) {
            // 每秒诊断：phase / snap / ring
            b.logText.setText(line.replace("# ", "").trim());
            if (line.contains("phase=STAGING")) {
                b.monText.setText("探测中 · " + line.replace("# MON ", "").trim());
            } else if (line.contains("phase=RECORD") || line.contains("rec=1")) {
                b.monText.setText("记录中 · " + line.replace("# MON ", "").trim());
            } else if (line.contains("phase=IDLE") && line.contains("ring=")) {
                // ring 在涨说明武装环缓正常，等待触发
                b.monText.setText("武装环缓 · " + line.replace("# MON ", "").trim());
            }
        } else if (line.contains("# DIAG") || line.startsWith("# DIAG") || line.contains("DIAG snap=")
                || line.contains("DIAG2")) {
            b.dumpProgText.setText(line.replace("# ", "").trim());
            b.playInfo.setText(line.replace("# ", "").trim());
        } else if (line.contains("empty SNAP") || line.contains("trigger never")) {
            b.monText.setText("未触发正式记录（环缓有数据）");
            b.dumpProgText.setText(line.replace("# ", "").trim());
            toast("未触发：保持转速>10约1秒，或检查 I 信号");
        } else if (line.contains("SNAP DUMP READY")) {
            // no-op
        } else if (line.contains("ALIVE")) {
            if (b.dumpProgText.getText() == null
                    || !b.dumpProgText.getText().toString().contains("倒计时")) {
                b.monText.setText(line.replace("# ", "").trim());
            }
        } else if (line.contains("CONFIRM")) {
            b.monText.setText("开始记录 · 无实时转速（流分离，正常）");
            toast("开始记录，请保持转动");
        } else if (line.contains("MONITOR armed")) {
            b.monText.setText("已武装 · 实时转速已关，请加油");
            b.rpmValue.setText("--");
        } else if (line.contains("STAGING") || line.contains("QUIET")) {
            b.monText.setText("探测中 · 无实时转速（正常）");
            toast("探测到了");
        } else if (line.contains("SD SAVE OK")) {
            b.monText.setText("已存入存储卡");
            toast("已存入存储卡");
        } else if (line.contains("BIN BLE BEGIN")) {
            b.playInfo.setText("蓝牙传输中… 0%");
            b.dumpProgText.setText("下载中… 0%");
            b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
            b.dumpProgressBar.setProgress(0);
        } else if (line.contains("BIN parse fail")) {
            toast("蓝牙数据校验失败");
            b.dumpProgText.setText(line.replace("# ", ""));
            b.dumpProgressBar.setVisibility(android.view.View.GONE);
        } else if (line.contains("DUMP BIN BLE fail") || line.contains("DUMP BIN BLE wait")
                || line.contains("DUMP BIN BLE busy") || line.contains("empty RAM")) {
            String err = line.replace("# ", "").trim();
            b.dumpProgText.setText(err);
            b.playInfo.setText(err);
            b.monText.setText("拉取失败");
            b.dumpProgressBar.setProgress(0);
            toast(err.contains("empty") ? "RAM 为空：记录未写入（看 RAM点数 是否>0）"
                    : (err.contains("wait") ? "请稍后再拉" : "拉取失败"));
        } else if (line.contains("phone_accepted") || line.contains("expect_D")) {
            b.playInfo.setText(line.replace("# ", ""));
            if (line.contains("expect_D")) {
                b.dumpProgText.setText("预计接收… " + line.replace("# ", ""));
            }
        } else if (line.contains("DUMP BUSY")) {
            b.monText.setText("记录完成 · 准备回传，请保持连接");
            b.dumpProgText.setText("回传准备中…");
        } else if (line.contains("AUTO DUMP READY")) {
            // snap 路径忽略旧提示
        } else if (line.contains("DUMP PROG")) {
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
            b.dumpProgressBar.setVisibility(android.view.View.VISIBLE);
            b.dumpProgressBar.setProgress(Math.max(0, Math.min(100, pct)));
        } else if (line.contains("AUTO DUMP BEGIN")) {
            playback.clear();
            indexEvents.clear();
            playing = false;
            b.playInfo.setText("板子自动推送数据中…");
            b.dumpProgText.setText("回传 0%");
        } else if (line.contains("AUTO DUMP END")) {
            if (!playback.isEmpty() && (archive.isEmpty()
                    || archive.get(0).samples.size() != playback.size())) {
                archiveIncoming();
            }
            b.playInfo.setText(String.format(Locale.US, "自动接收完成 · 档案 %d 段", archive.size()));
            b.dumpProgText.setText("回传完成");
            b.dumpProgressBar.setProgress(100);
        } else if (line.contains("NOISE")) {
            b.monText.setText("噪声：临时池已丢弃");
        } else if (line.contains("disarm") || line.contains("MONITOR disarm")) {
            b.monText.setText("已停记 · 转速仍实时刷新");
        } else if (line.contains("armed")) {
            b.monText.setText("已武装：等待 |RPM|>10 且 I 过1圈（或1.5s）");
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
