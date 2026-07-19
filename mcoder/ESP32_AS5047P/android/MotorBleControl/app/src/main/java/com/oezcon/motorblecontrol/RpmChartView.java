package com.oezcon.motorblecontrol;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;

import java.util.ArrayDeque;
import java.util.Locale;

/**
 * 最近若干秒转速折线（对齐 PC 端 live 窗口观感）。
 */
public class RpmChartView extends View {
    public static final float WINDOW_S = 8f;

    private static final class Sample {
        final long tMs;
        final float rpm;

        Sample(long tMs, float rpm) {
            this.tMs = tMs;
            this.rpm = rpm;
        }
    }

    private final ArrayDeque<Sample> samples = new ArrayDeque<>();
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
        padL = 40f * d;
        padR = 10f * d;
        padT = 12f * d;
        padB = 22f * d;

        int accent = ContextCompat.getColor(context, R.color.accent);
        int muted = ContextCompat.getColor(context, R.color.muted);
        int grid = ContextCompat.getColor(context, R.color.chart_grid);

        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setStrokeWidth(2.5f * d);
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
    }

    public void addSample(float rpm) {
        long now = System.currentTimeMillis();
        samples.addLast(new Sample(now, Math.max(0f, rpm)));
        long cut = now - (long) (WINDOW_S * 1000f);
        while (!samples.isEmpty() && samples.peekFirst().tMs < cut) {
            samples.removeFirst();
        }
        invalidate();
    }

    public void clear() {
        samples.clear();
        invalidate();
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
        float plotW = Math.max(1f, right - left);
        float plotH = Math.max(1f, bottom - top);

        // 背景网格
        for (int i = 0; i <= 3; i++) {
            float y = top + plotH * i / 3f;
            canvas.drawLine(left, y, right, y, gridPaint);
        }
        canvas.drawLine(left, top, left, bottom, axisPaint);
        canvas.drawLine(left, bottom, right, bottom, axisPaint);

        long now = System.currentTimeMillis();
        long t0 = now - (long) (WINDOW_S * 1000f);

        float yMax = 50f;
        for (Sample s : samples) {
            if (s.rpm > yMax) yMax = s.rpm;
        }
        yMax = Math.max(yMax * 1.15f, yMax + 30f);

        // Y 轴标注
        canvas.drawText(String.format(Locale.US, "%.0f", yMax), 4f, top + textPaint.getTextSize(), textPaint);
        canvas.drawText("0", 4f, bottom, textPaint);
        // X 轴：窗口秒数
        canvas.drawText("-" + (int) WINDOW_S + "s", left, h - 4f, textPaint);
        canvas.drawText("现在", right - 28f * getResources().getDisplayMetrics().density, h - 4f, textPaint);

        if (samples.size() < 2) {
            textPaint.setTextAlign(Paint.Align.CENTER);
            canvas.drawText("等待转速数据…", left + plotW / 2f, top + plotH / 2f, textPaint);
            textPaint.setTextAlign(Paint.Align.LEFT);
            return;
        }

        path.reset();
        Path fill = new Path();
        boolean first = true;
        float lastX = left;
        float lastY = bottom;
        for (Sample s : samples) {
            float x = left + plotW * ((s.tMs - t0) / (WINDOW_S * 1000f));
            if (x < left) x = left;
            if (x > right) x = right;
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

        // 最新点
        canvas.drawCircle(lastX, lastY, 4f * getResources().getDisplayMetrics().density, linePaint);
    }
}
