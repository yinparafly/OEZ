package com.oezcon.motorblecontrol;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.oezcon.motorblecontrol.databinding.ActivityMainBinding;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 电机+磁编码器台架 BLE 遥控（非听声测转速）。
 * 对接 ESP32 OEZ-RPM，无需改固件。
 */
public class MainActivity extends AppCompatActivity implements BleUartClient.Listener {
    private static final int REQ_BT = 1001;
    /** 设计减速比，仅对照；界面显示霍尔/编码器实测 */
    private static final float GEAR_RATIO_DESIGN = (59f * 79f) / (12f * 14f);

    private ActivityMainBinding b;
    private BleUartClient ble;
    private final List<BleUartClient.DeviceItem> devices = new ArrayList<>();
    private ArrayAdapter<BleUartClient.DeviceItem> adapter;
    private float lastTarget;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        b = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(b.getRoot());

        ble = new BleUartClient(this, this);
        adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, devices);
        b.deviceSpinner.setAdapter(adapter);

        b.btnScan.setOnClickListener(v -> {
            if (!ensureBtPerm()) return;
            if (!ble.hasAdapter()) {
                toast("请打开手机蓝牙");
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
            if (!(sel instanceof BleUartClient.DeviceItem)) {
                toast("请先扫描并选择 OEZ-RPM");
                return;
            }
            ble.connect(((BleUartClient.DeviceItem) sel).address);
        });

        b.btnStart.setOnClickListener(v -> {
            if (!ble.isConnected()) {
                toast("请先连接");
                return;
            }
            ble.startMotor();
        });
        b.btnStop.setOnClickListener(v -> {
            if (!ble.isConnected()) return;
            ble.stopMotor();
        });
        b.btnSetRpm.setOnClickListener(v -> applyInputRpm());
        b.btnP1200.setOnClickListener(v -> applyPreset(1200));
        b.btnP2400.setOnClickListener(v -> applyPreset(2400));
        b.btnP3600.setOnClickListener(v -> applyPreset(3600));
        b.btnP4800.setOnClickListener(v -> applyPreset(4800));

        ensureBtPerm();
    }

    private void applyInputRpm() {
        if (!ble.isConnected()) {
            toast("请先连接");
            return;
        }
        String s = b.rpmInput.getText() != null ? b.rpmInput.getText().toString().trim() : "";
        try {
            float rpm = Float.parseFloat(s);
            lastTarget = rpm;
            ble.setRpm(rpm);
            b.targetText.setText(String.format(Locale.US, "目标 %.0f", rpm));
        } catch (NumberFormatException e) {
            toast("请输入有效转速");
        }
    }

    private void applyPreset(int rpm) {
        if (!ble.isConnected()) {
            toast("请先连接");
            return;
        }
        lastTarget = rpm;
        b.rpmInput.setText(String.valueOf(rpm));
        ble.setRpm(rpm);
        b.targetText.setText(String.format(Locale.US, "目标 %d", rpm));
        // 预设后自动启动，减少点按
        ble.startMotor();
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
        if (need.isEmpty()) return true;
        ActivityCompat.requestPermissions(this, need.toArray(new String[0]), REQ_BT);
        return false;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_BT) {
            boolean ok = true;
            for (int r : grantResults) {
                if (r != PackageManager.PERMISSION_GRANTED) ok = false;
            }
            b.statusText.setText(ok ? "权限已授予，可扫描" : "需要蓝牙权限才能连接台架");
        }
    }

    @Override
    public void onStatus(String msg) {
        b.statusText.setText(msg);
    }

    @Override
    public void onDevices(List<BleUartClient.DeviceItem> list) {
        devices.clear();
        devices.addAll(list);
        adapter.notifyDataSetChanged();
    }

    @Override
    public void onConnected(boolean ok) {
        b.btnConnect.setText(ok ? "断开" : "连接");
        showRpmPending();
        if (!ok) {
            b.rpmChart.clear();
            lastTarget = 0;
            b.targetText.setText("目标 —");
        }
    }

    @Override
    public void onRpm(float rpm, float target, int pulseUs, float outHzMeas, float gearMeas) {
        // 有遥测就显示数值（含 0）；未收到时才显示「未收到」
        b.rpmValue.setTextSize(56f);
        b.rpmValue.setText(String.format(Locale.US, "%.0f", rpm));
        b.rpmChart.addSample(rpm);
        if (outHzMeas >= 0f) {
            float outRpm = outHzMeas * 60f;
            if (gearMeas >= 0f) {
                float err = (gearMeas - GEAR_RATIO_DESIGN) / GEAR_RATIO_DESIGN * 100f;
                b.outFreqText.setText(String.format(Locale.US,
                        "输出(霍尔实测): %.2f Hz / %.1f RPM\n"
                                + "减速比实测 %.3f  设计 %.3f  偏差 %+.1f%%",
                        outHzMeas, outRpm, gearMeas, GEAR_RATIO_DESIGN, err));
            } else {
                b.outFreqText.setText(String.format(Locale.US,
                        "输出(霍尔实测): %.2f Hz / %.1f RPM\n减速比实测 —（需≥2次下扑）",
                        outHzMeas, outRpm));
            }
        } else {
            b.outFreqText.setText(String.format(Locale.US,
                    "输出(霍尔实测): —（等下扑≥2次）\n设计 i=(59×79)/(12×14)=%.3f",
                    GEAR_RATIO_DESIGN));
        }
        if (target > 0.5f) {
            lastTarget = target;
            b.targetText.setText(String.format(Locale.US,
                    "目标 %.0f RPM  ·  油门 %d μs", target, pulseUs));
        } else if (lastTarget > 0) {
            b.targetText.setText(String.format(Locale.US,
                    "目标 %.0f RPM  ·  油门 %d μs", lastTarget, pulseUs));
        } else {
            b.targetText.setText(String.format(Locale.US, "目标 —  ·  油门 %d μs", pulseUs));
        }
    }

    private void showRpmPending() {
        b.rpmValue.setTextSize(40f);
        b.rpmValue.setText("未收到");
        b.outFreqText.setText(String.format(Locale.US,
                "输出(霍尔实测): —\n设计 i=(59×79)/(12×14)=%.3f", GEAR_RATIO_DESIGN));
    }

    private void toast(String s) {
        Toast.makeText(this, s, Toast.LENGTH_SHORT).show();
    }

    @Override
    protected void onDestroy() {
        ble.disconnect();
        super.onDestroy();
    }
}
