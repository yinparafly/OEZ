package com.oezcon.motorpitchrpm;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;

/** 简易频谱条 + 主频标记 */
public class SpectrumView extends View {
    private final Paint barPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint markPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private float[] spectrumDb;
    private float markHz;
    private float fMaxHz = 20000f;
    private int sampleRate = 44100;

    public SpectrumView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        barPaint.setColor(ContextCompat.getColor(context, R.color.accent));
        markPaint.setColor(ContextCompat.getColor(context, R.color.warn));
        markPaint.setStrokeWidth(3f);
        gridPaint.setColor(ContextCompat.getColor(context, R.color.muted));
        gridPaint.setStrokeWidth(1f);
        gridPaint.setAlpha(80);
    }

    public void setSpectrum(float[] db, int sampleRate, float markHz, float fMaxHz) {
        this.spectrumDb = db;
        this.sampleRate = sampleRate;
        this.markHz = markHz;
        this.fMaxHz = Math.max(100f, fMaxHz);
        postInvalidateOnAnimation();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float w = getWidth();
        float h = getHeight();
        if (spectrumDb == null || spectrumDb.length < 2) return;

        float hzPerBin = sampleRate / (float) (spectrumDb.length * 2);
        int maxBin = Math.min(spectrumDb.length - 1, (int) (fMaxHz / hzPerBin));
        if (maxBin < 2) return;

        float minDb = -90f;
        float maxDb = -10f;
        float barW = Math.max(1f, w / maxBin);
        for (int i = 1; i <= maxBin; i++) {
            float db = spectrumDb[i];
            float norm = (db - minDb) / (maxDb - minDb);
            if (norm < 0) norm = 0;
            if (norm > 1) norm = 1;
            float bh = norm * (h - 8);
            float x = (i / (float) maxBin) * w;
            canvas.drawRect(x, h - bh, x + barW, h, barPaint);
        }

        if (markHz > 0 && markHz <= fMaxHz) {
            float x = (markHz / fMaxHz) * w;
            canvas.drawLine(x, 0, x, h, markPaint);
        }
        // 中线
        canvas.drawLine(0, h * 0.5f, w, h * 0.5f, gridPaint);
    }
}
