package com.oezcon.motorpitchrpm;

import android.Manifest;
import android.content.pm.PackageManager;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.button.MaterialButton;
import com.google.android.material.slider.Slider;

/**
 * 听电机声测转速（类似 Motor Pitch Analyzer）：
 * 对麦克风做实时 FFT，找主频，RPM = f_Hz * 60 / eventsPerRevolution。
 */
public class MainActivity extends AppCompatActivity {
    private static final int REQ_MIC = 1001;
    private static final int SAMPLE_RATE = 44100;
    private static final int FFT_SIZE = 4096;

    private TextView rpmValue;
    private TextView rpmMax;
    private TextView freqValue;
    private TextView statusText;
    private TextView eventsLabel;
    private TextView maxRpmLabel;
    private SpectrumView spectrumView;
    private MaterialButton btnStart;
    private MaterialButton btnStop;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private PitchAnalyzer analyzer;
    private volatile boolean running;
    private float eventsPerRev = 1f;
    private float maxRpm = 20000f;
    private float peakRpm;
    private float smoothedRpm;
    private float smoothedHz;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        rpmValue = findViewById(R.id.rpmValue);
        rpmMax = findViewById(R.id.rpmMax);
        freqValue = findViewById(R.id.freqValue);
        statusText = findViewById(R.id.statusText);
        eventsLabel = findViewById(R.id.eventsLabel);
        maxRpmLabel = findViewById(R.id.maxRpmLabel);
        spectrumView = findViewById(R.id.spectrum);
        btnStart = findViewById(R.id.btnStart);
        btnStop = findViewById(R.id.btnStop);
        Slider eventsSlider = findViewById(R.id.eventsSlider);
        Slider maxRpmSlider = findViewById(R.id.maxRpmSlider);

        eventsSlider.addOnChangeListener((s, v, fromUser) -> {
            eventsPerRev = Math.max(1f, v);
            eventsLabel.setText(String.format(
                    "事件数 = %.0f  →  RPM = Hz × 60 / %.0f", eventsPerRev, eventsPerRev));
        });
        maxRpmSlider.addOnChangeListener((s, v, fromUser) -> {
            maxRpm = v;
            maxRpmLabel.setText(String.format("上限 %.0f RPM（缩小范围可提高精度）", maxRpm));
        });

        btnStart.setOnClickListener(v -> ensureMicAndStart());
        btnStop.setOnClickListener(v -> stopMeasure());
        findViewById(R.id.btnResetPeak).setOnClickListener(v -> {
            peakRpm = 0f;
            rpmMax.setText("峰值 — RPM");
        });
    }

    private void ensureMicAndStart() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(
                    this, new String[]{Manifest.permission.RECORD_AUDIO}, REQ_MIC);
            return;
        }
        startMeasure();
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_MIC
                && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startMeasure();
        } else {
            Toast.makeText(this, "需要麦克风权限才能听声测转速", Toast.LENGTH_LONG).show();
        }
    }

    private void startMeasure() {
        if (running) return;
        running = true;
        smoothedRpm = 0f;
        smoothedHz = 0f;
        btnStart.setEnabled(false);
        btnStop.setEnabled(true);
        statusText.setText("测听中… 请靠近电机");
        analyzer = new PitchAnalyzer(SAMPLE_RATE, FFT_SIZE, this::onFrame);
        new Thread(analyzer, "pitch-analyzer").start();
    }

    private void stopMeasure() {
        running = false;
        if (analyzer != null) {
            analyzer.stop();
            analyzer = null;
        }
        btnStart.setEnabled(true);
        btnStop.setEnabled(false);
        statusText.setText("已停止");
    }

    private void onFrame(PitchAnalyzer.Result r) {
        if (!running) return;
        float fMaxHz = (maxRpm / 60f) * eventsPerRev;
        float fMinHz = 20f; // 忽略过低噪声
        float hz = r.peakHz;
        float mag = r.peakMag;
        // 若峰值超出设定上限，在频带内重选
        if (hz < fMinHz || hz > fMaxHz) {
            hz = r.bandPeakHz;
            mag = r.bandPeakMag;
        }
        float rpm = (hz * 60f) / eventsPerRev;
        // 强度太弱则视为无有效信号
        boolean weak = mag < r.noiseFloor * 4f || hz < fMinHz;
        if (weak) {
            smoothedRpm = smoothedRpm * 0.92f;
            smoothedHz = smoothedHz * 0.92f;
        } else {
            smoothedRpm = 0.75f * smoothedRpm + 0.25f * rpm;
            smoothedHz = 0.75f * smoothedHz + 0.25f * hz;
            if (smoothedRpm > peakRpm) peakRpm = smoothedRpm;
        }
        final float showRpm = smoothedRpm;
        final float showHz = smoothedHz;
        final float showPeak = peakRpm;
        final float[] spec = r.spectrumDb;
        final float markHz = showHz;
        ui.post(() -> {
            if (weak && showRpm < 30f) {
                rpmValue.setText("— RPM");
                freqValue.setText("主频 — Hz（信号弱）");
            } else {
                rpmValue.setText(String.format("%.0f RPM", showRpm));
                freqValue.setText(String.format("主频 %.1f Hz", showHz));
            }
            if (showPeak > 1f) {
                rpmMax.setText(String.format("峰值 %.0f RPM", showPeak));
            }
            spectrumView.setSpectrum(spec, SAMPLE_RATE, markHz, fMaxHz);
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
        stopMeasure();
    }

    /** 麦克风 + FFT 主频估计 */
    static final class PitchAnalyzer implements Runnable {
        interface Listener {
            void onResult(Result result);
        }

        static final class Result {
            final float peakHz;
            final float peakMag;
            final float bandPeakHz;
            final float bandPeakMag;
            final float noiseFloor;
            final float[] spectrumDb;

            Result(float peakHz, float peakMag, float bandPeakHz, float bandPeakMag,
                   float noiseFloor, float[] spectrumDb) {
                this.peakHz = peakHz;
                this.peakMag = peakMag;
                this.bandPeakHz = bandPeakHz;
                this.bandPeakMag = bandPeakMag;
                this.noiseFloor = noiseFloor;
                this.spectrumDb = spectrumDb;
            }
        }

        private final int sampleRate;
        private final int fftSize;
        private final Listener listener;
        private volatile boolean stop;

        PitchAnalyzer(int sampleRate, int fftSize, Listener listener) {
            this.sampleRate = sampleRate;
            this.fftSize = fftSize;
            this.listener = listener;
        }

        void stop() {
            stop = true;
        }

        @Override
        public void run() {
            int minBuf = AudioRecord.getMinBufferSize(
                    sampleRate,
                    AudioFormat.CHANNEL_IN_MONO,
                    AudioFormat.ENCODING_PCM_16BIT);
            int bufSize = Math.max(minBuf, fftSize * 2);
            AudioRecord record = null;
            try {
                record = new AudioRecord(
                        MediaRecorder.AudioSource.MIC,
                        sampleRate,
                        AudioFormat.CHANNEL_IN_MONO,
                        AudioFormat.ENCODING_PCM_16BIT,
                        bufSize);
                if (record.getState() != AudioRecord.STATE_INITIALIZED) {
                    return;
                }
                short[] pcm = new short[fftSize];
                double[] re = new double[fftSize];
                double[] im = new double[fftSize];
                float[] window = new float[fftSize];
                for (int i = 0; i < fftSize; i++) {
                    window[i] = (float) (0.5 - 0.5 * Math.cos(2 * Math.PI * i / (fftSize - 1.0)));
                }
                record.startRecording();
                while (!stop) {
                    int got = 0;
                    while (got < fftSize && !stop) {
                        int n = record.read(pcm, got, fftSize - got);
                        if (n <= 0) break;
                        got += n;
                    }
                    if (got < fftSize) continue;

                    for (int i = 0; i < fftSize; i++) {
                        re[i] = (pcm[i] / 32768.0) * window[i];
                        im[i] = 0;
                    }
                    fft(re, im);

                    int bins = fftSize / 2;
                    float[] db = new float[bins];
                    float peakMag = 0f;
                    int peakBin = 1;
                    double magSum = 0;
                    for (int i = 1; i < bins; i++) {
                        float mag = (float) Math.hypot(re[i], im[i]);
                        db[i] = (float) (20 * Math.log10(mag + 1e-9));
                        magSum += mag;
                        if (mag > peakMag) {
                            peakMag = mag;
                            peakBin = i;
                        }
                    }
                    float noiseFloor = (float) (magSum / (bins - 1));
                    // 抛物线插值细化峰值频率
                    float peakHz = refineBinHz(peakBin, re, im, sampleRate, fftSize);

                    // 带限峰值：默认先找全局，MainActivity 会再按上限裁剪；这里提供 20Hz~Nyquist
                    float bandPeakMag = 0f;
                    int bandBin = peakBin;
                    int minBin = Math.max(1, hzToBin(20, sampleRate, fftSize));
                    int maxBin = Math.min(bins - 1, hzToBin(sampleRate / 2f - 100, sampleRate, fftSize));
                    for (int i = minBin; i <= maxBin; i++) {
                        float mag = (float) Math.hypot(re[i], im[i]);
                        if (mag > bandPeakMag) {
                            bandPeakMag = mag;
                            bandBin = i;
                        }
                    }
                    float bandPeakHz = refineBinHz(bandBin, re, im, sampleRate, fftSize);

                    listener.onResult(new Result(
                            peakHz, peakMag, bandPeakHz, bandPeakMag, noiseFloor, db));
                }
            } catch (SecurityException ignored) {
            } finally {
                if (record != null) {
                    try {
                        record.stop();
                    } catch (Exception ignored) {
                    }
                    record.release();
                }
            }
        }

        private static int hzToBin(float hz, int sr, int n) {
            return Math.round(hz * n / (float) sr);
        }

        private static float refineBinHz(int bin, double[] re, double[] im, int sr, int n) {
            if (bin <= 0 || bin >= n / 2 - 1) {
                return bin * (sr / (float) n);
            }
            float a = (float) Math.hypot(re[bin - 1], im[bin - 1]);
            float b = (float) Math.hypot(re[bin], im[bin]);
            float c = (float) Math.hypot(re[bin + 1], im[bin + 1]);
            float denom = (a - 2 * b + c);
            float delta = 0f;
            if (Math.abs(denom) > 1e-12f) {
                delta = 0.5f * (a - c) / denom;
                if (delta > 0.5f) delta = 0.5f;
                if (delta < -0.5f) delta = -0.5f;
            }
            return (bin + delta) * (sr / (float) n);
        }

        /** In-place Cooley–Tukey radix-2 FFT */
        private static void fft(double[] re, double[] im) {
            int n = re.length;
            int j = 0;
            for (int i = 1; i < n; i++) {
                int bit = n >> 1;
                for (; (j & bit) != 0; bit >>= 1) j ^= bit;
                j ^= bit;
                if (i < j) {
                    double tr = re[i];
                    re[i] = re[j];
                    re[j] = tr;
                    double ti = im[i];
                    im[i] = im[j];
                    im[j] = ti;
                }
            }
            for (int len = 2; len <= n; len <<= 1) {
                double ang = -2 * Math.PI / len;
                double wlenRe = Math.cos(ang);
                double wlenIm = Math.sin(ang);
                for (int i = 0; i < n; i += len) {
                    double wRe = 1;
                    double wIm = 0;
                    for (int k = 0; k < len / 2; k++) {
                        int u = i + k;
                        int v = i + k + len / 2;
                        double tRe = wRe * re[v] - wIm * im[v];
                        double tIm = wRe * im[v] + wIm * re[v];
                        re[v] = re[u] - tRe;
                        im[v] = im[u] - tIm;
                        re[u] += tRe;
                        im[u] += tIm;
                        double nwRe = wRe * wlenRe - wIm * wlenIm;
                        wIm = wRe * wlenIm + wIm * wlenRe;
                        wRe = nwRe;
                    }
                }
            }
        }
    }
}
