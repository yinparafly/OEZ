package com.oezcon.abirpmonitor;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;

import java.util.ArrayDeque;
import java.util.Locale;

/**
 * 双模式曲线：
 * - LIVE：最近若干秒墙钟转速
 * - SEGMENT：已接收段的 转速 vs 相对时间(s)，可设窗口 / 双指缩放 / 拖动平移
 */
public class RpmChartView extends View {
    public static final float LIVE_WINDOW_S = 8f;

    private static final class Sample {
        final long tMs;
        final float rpm;

        Sample(long tMs, float rpm) {
            this.tMs = tMs;
            this.rpm = rpm;
        }
    }

    private final ArrayDeque<Sample> live = new ArrayDeque<>();
    private float[] segT;   // 秒，相对段起点
    private float[] segRpm;
    private boolean segmentMode;
    private float winStart = 0f;
    private float winEnd = 2f;
    private float dataEnd = 2f;

    private final Path path = new Path();
    private final Paint linePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint axisPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float padL;
    private float padR;
    private float padT;
    private float padB;

    private ScaleGestureDetector scaleDet;
    private GestureDetector gestureDet;
    private float plotLeft;
    private float plotRight;

    public RpmChartView(Context context) {
        super(context);
        init(context);
    }

    public RpmChartView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public RpmChartView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init(context);
    }

    private void init(Context context) {
        float d = getResources().getDisplayMetrics().density;
        padL = 44f * d;
        padR = 12f * d;
        padT = 14f * d;
        padB = 28f * d;

        int accent = ContextCompat.getColor(context, R.color.accent);
        int muted = ContextCompat.getColor(context, R.color.muted);
        int grid = ContextCompat.getColor(context, R.color.chart_grid);

        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setStrokeWidth(2.2f * d);
        linePaint.setColor(accent);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
        linePaint.setStrokeCap(Paint.Cap.ROUND);

        fillPaint.setStyle(Paint.Style.FILL);
        fillPaint.setColor((accent & 0x00FFFFFF) | 0x28000000);

        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(1f * d);
        gridPaint.setColor(grid);

        axisPaint.setStyle(Paint.Style.STROKE);
        axisPaint.setStrokeWidth(1.2f * d);
        axisPaint.setColor(muted);

        textPaint.setColor(muted);
        textPaint.setTextSize(11f * d);

        scaleDet = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                if (!segmentMode) return false;
                float span = Math.max(1e-4f, winEnd - winStart);
                float mid = (winStart + winEnd) * 0.5f;
                float factor = 1f / detector.getScaleFactor();
                float half = span * factor * 0.5f;
                half = Math.max(0.01f, Math.min(half, Math.max(dataEnd, 0.05f)));
                setTimeWindow(mid - half, mid + half);
                return true;
            }
        });
        gestureDet = new GestureDetector(context, new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
                if (!segmentMode) return false;
                float plotW = Math.max(1f, plotRight - plotLeft);
                float span = Math.max(1e-4f, winEnd - winStart);
                float dt = distanceX / plotW * span;
                setTimeWindow(winStart + dt, winEnd + dt);
                return true;
            }

            @Override
            public boolean onDoubleTap(MotionEvent e) {
                if (!segmentMode) return false;
                setTimeWindow(0f, Math.max(dataEnd, 0.1f));
                return true;
            }
        });
    }

    public void addSample(float rpm) {
        if (segmentMode) return;
        long now = System.currentTimeMillis();
        live.addLast(new Sample(now, Math.abs(rpm)));
        long cut = now - (long) (LIVE_WINDOW_S * 1000f);
        while (!live.isEmpty() && live.peekFirst().tMs < cut) {
            live.removeFirst();
        }
        invalidate();
    }

    public void clearLive() {
        live.clear();
        if (!segmentMode) invalidate();
    }

    public void clear() {
        live.clear();
        segT = null;
        segRpm = null;
        segmentMode = false;
        invalidate();
    }

    /** 加载一段：tSec 相对段起点秒，rpm 可为有符号 */
    public void setSegmentData(float[] tSec, float[] rpm) {
        if (tSec == null || rpm == null || tSec.length < 2 || tSec.length != rpm.length) {
            segT = null;
            segRpm = null;
            segmentMode = false;
            invalidate();
            return;
        }
        segT = tSec;
        segRpm = rpm;
        dataEnd = tSec[tSec.length - 1];
        if (dataEnd < 0.05f) dataEnd = 0.05f;
        segmentMode = true;
        winStart = 0f;
        winEnd = dataEnd;
        invalidate();
    }

    public void setLiveMode() {
        segmentMode = false;
        invalidate();
    }

    public boolean isSegmentMode() {
        return segmentMode;
    }

    public float getWinStart() {
        return winStart;
    }

    public float getWinEnd() {
        return winEnd;
    }

    public float getDataEnd() {
        return dataEnd;
    }

    public void setTimeWindow(float startS, float endS) {
        if (endS < startS) {
            float tmp = startS;
            startS = endS;
            endS = tmp;
        }
        float minSpan = 0.01f;
        if (endS - startS < minSpan) endS = startS + minSpan;
        if (startS < 0f) {
            endS -= startS;
            startS = 0f;
        }
        float maxEnd = Math.max(dataEnd, minSpan);
        if (endS > maxEnd) {
            float shift = endS - maxEnd;
            endS = maxEnd;
            startS = Math.max(0f, startS - shift);
        }
        winStart = startS;
        winEnd = endS;
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (!segmentMode) return super.onTouchEvent(event);
        boolean a = scaleDet.onTouchEvent(event);
        boolean b = gestureDet.onTouchEvent(event);
        return a || b || true;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float w = getWidth();
        float h = getHeight();
        if (w <= 0 || h <= 0) return;

        float left = padL;
        float right = w - padR;
        float top = padT;
        float bottom = h - padB;
        plotLeft = left;
        plotRight = right;
        float plotW = Math.max(1f, right - left);
        float plotH = Math.max(1f, bottom - top);

        for (int i = 0; i <= 4; i++) {
            float y = top + plotH * i / 4f;
            canvas.drawLine(left, y, right, y, gridPaint);
        }
        canvas.drawLine(left, top, left, bottom, axisPaint);
        canvas.drawLine(left, bottom, right, bottom, axisPaint);

        if (segmentMode) {
            drawSegment(canvas, left, right, top, bottom, plotW, plotH);
        } else {
            drawLive(canvas, left, right, top, bottom, plotW, plotH, h);
        }
    }

    private void drawLive(Canvas canvas, float left, float right, float top, float bottom,
                          float plotW, float plotH, float h) {
        long now = System.currentTimeMillis();
        long t0 = now - (long) (LIVE_WINDOW_S * 1000f);
        float yMax = 50f;
        for (Sample s : live) {
            if (s.rpm > yMax) yMax = s.rpm;
        }
        yMax = Math.max(yMax * 1.15f, yMax + 30f);
        canvas.drawText(String.format(Locale.US, "%.0f", yMax), 4f, top + textPaint.getTextSize(), textPaint);
        canvas.drawText("0", 4f, bottom, textPaint);
        canvas.drawText("live -" + (int) LIVE_WINDOW_S + "s", left, h - 4f, textPaint);
        canvas.drawText("now", right - 28f * getResources().getDisplayMetrics().density, h - 4f, textPaint);
        if (live.size() < 2) {
            textPaint.setTextAlign(Paint.Align.CENTER);
            canvas.drawText("waiting…", left + plotW / 2f, top + plotH / 2f, textPaint);
            textPaint.setTextAlign(Paint.Align.LEFT);
            return;
        }
        path.reset();
        Path fill = new Path();
        boolean first = true;
        float lastX = left;
        float lastY = bottom;
        for (Sample s : live) {
            float x = left + plotW * ((s.tMs - t0) / (LIVE_WINDOW_S * 1000f));
            x = Math.max(left, Math.min(right, x));
            float y = bottom - (s.rpm / yMax) * plotH;
            if (first) {
                path.moveTo(x, y);
                fill.moveTo(x, bottom);
                fill.lineTo(x, y);
                first = false;
            } else {
                path.lineTo(x, y);
                fill.lineTo(x, y);
            }
            lastX = x;
            lastY = y;
        }
        fill.lineTo(lastX, bottom);
        fill.close();
        canvas.drawPath(fill, fillPaint);
        canvas.drawPath(path, linePaint);
    }

    private void drawSegment(Canvas canvas, float left, float right, float top, float bottom,
                             float plotW, float plotH) {
        float span = Math.max(1e-4f, winEnd - winStart);
        float yMax = 50f;
        for (int i = 0; i < segRpm.length; i++) {
            float t = segT[i];
            if (t < winStart || t > winEnd) continue;
            float a = Math.abs(segRpm[i]);
            if (a > yMax) yMax = a;
        }
        yMax = Math.max(yMax * 1.12f, yMax + 20f);

        canvas.drawText(String.format(Locale.US, "%.0f", yMax), 4f, top + textPaint.getTextSize(), textPaint);
        canvas.drawText("0", 4f, bottom, textPaint);
        canvas.drawText(String.format(Locale.US, "%.3fs", winStart), left, getHeight() - 4f, textPaint);
        String endLab = String.format(Locale.US, "%.3fs", winEnd);
        textPaint.setTextAlign(Paint.Align.RIGHT);
        canvas.drawText(endLab, right, getHeight() - 4f, textPaint);
        textPaint.setTextAlign(Paint.Align.CENTER);
        canvas.drawText("|RPM| vs t  (pinch/drag)", left + plotW / 2f, top - 2f, textPaint);
        textPaint.setTextAlign(Paint.Align.LEFT);

        // 竖向时间网格
        int nGrid = 4;
        for (int i = 0; i <= nGrid; i++) {
            float t = winStart + span * i / nGrid;
            float x = left + plotW * ((t - winStart) / span);
            canvas.drawLine(x, top, x, bottom, gridPaint);
        }

        path.reset();
        Path fill = new Path();
        boolean first = true;
        float lastX = left;
        for (int i = 0; i < segT.length; i++) {
            float t = segT[i];
            if (t < winStart - span * 0.02f || t > winEnd + span * 0.02f) continue;
            float x = left + plotW * ((t - winStart) / span);
            float y = bottom - (Math.abs(segRpm[i]) / yMax) * plotH;
            if (x < left) x = left;
            if (x > right) x = right;
            if (first) {
                path.moveTo(x, y);
                fill.moveTo(x, bottom);
                fill.lineTo(x, y);
                first = false;
            } else {
                path.lineTo(x, y);
                fill.lineTo(x, y);
            }
            lastX = x;
        }
        if (!first) {
            fill.lineTo(lastX, bottom);
            fill.close();
            canvas.drawPath(fill, fillPaint);
            canvas.drawPath(path, linePaint);
        } else {
            textPaint.setTextAlign(Paint.Align.CENTER);
            canvas.drawText("窗口内无数据", left + plotW / 2f, top + plotH / 2f, textPaint);
            textPaint.setTextAlign(Paint.Align.LEFT);
        }
    }
}
