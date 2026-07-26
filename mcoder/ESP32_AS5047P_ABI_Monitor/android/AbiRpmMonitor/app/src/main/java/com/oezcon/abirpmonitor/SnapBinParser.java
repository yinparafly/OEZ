package com.oezcon.abirpmonitor;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

/**
 * 与固件 snap / PC parse_snap_bindump 一致。
 * v1 magic=0xAB1C0001：12B 点含板端 rpm_x10
 * v2 magic=0xAB1C0002：16B 点含 counts，转速在手机端算
 */
public final class SnapBinParser {
    public static final int MAGIC_V1 = 0xAB1C0001;
    public static final int MAGIC_V2 = 0xAB1C0002;
    public static final int POINT_SIZE_V1 = 12;
    public static final int POINT_SIZE_V2 = 16;
    public static final int ABI_STEPS_PER_REV = 4000;
    /** 与 PC HOST_VEL_WIN 对齐；曲线工作室可再平滑 */
    public static final int HOST_VEL_WIN = 32;

    public static final class Result {
        public final int n;
        public final int hz;
        /** 每点: tMs, rpm, dir, seg, indexN, tRel, unix */
        public final List<float[]> rows;

        Result(int n, int hz, List<float[]> rows) {
            this.n = n;
            this.hz = hz;
            this.rows = rows;
        }
    }

    private SnapBinParser() {
    }

    public static int crc32(byte[] data, int off, int len) {
        CRC32 c = new CRC32();
        c.update(data, off, len);
        return (int) c.getValue();
    }

    public static Result parse(byte[] raw) throws IllegalArgumentException {
        if (raw == null || raw.length < 12) {
            throw new IllegalArgumentException("bin too short: " + (raw == null ? 0 : raw.length));
        }
        ByteBuffer bb = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
        int magic = bb.getInt();
        int n = bb.getShort() & 0xFFFF;
        int hz = bb.getShort() & 0xFFFF;
        if (magic == MAGIC_V2) {
            return parsePayload(raw, n, hz, POINT_SIZE_V2, true);
        }
        if (magic == MAGIC_V1) {
            return parsePayload(raw, n, hz, POINT_SIZE_V1, false);
        }
        throw new IllegalArgumentException(String.format("bad magic 0x%08X", magic));
    }

    private static Result parsePayload(byte[] raw, int n, int hz, int pointSize, boolean countsFmt) {
        int need = 8 + n * pointSize + 4;
        if (raw.length < need) {
            throw new IllegalArgumentException("bin len " + raw.length + " < need " + need);
        }
        int payloadOff = 8;
        int payloadLen = n * pointSize;
        int expectCrc = ByteBuffer.wrap(raw, payloadOff + payloadLen, 4)
                .order(ByteOrder.LITTLE_ENDIAN).getInt();
        int gotCrc = crc32(raw, payloadOff, payloadLen);
        if (gotCrc != expectCrc) {
            throw new IllegalArgumentException(
                    String.format("CRC mismatch got=0x%08X expect=0x%08X", gotCrc, expectCrc));
        }
        ByteBuffer pb = ByteBuffer.wrap(raw, payloadOff, payloadLen).order(ByteOrder.LITTLE_ENDIAN);
        if (!countsFmt) {
            List<float[]> rows = new ArrayList<>(n);
            for (int i = 0; i < n; i++) {
                long tUs = pb.getInt() & 0xFFFFFFFFL;
                short rpmX10 = pb.getShort();
                byte dir = pb.get();
                pb.get(); // pad
                long indexN = pb.getInt() & 0xFFFFFFFFL;
                rows.add(new float[]{tUs / 1000f, rpmX10 / 10f, dir, 1, indexN, 0, 0});
            }
            return new Result(n, hz, rows);
        }
        long[] tUs = new long[n];
        long[] counts = new long[n];
        long[] indexN = new long[n];
        for (int i = 0; i < n; i++) {
            tUs[i] = pb.getInt() & 0xFFFFFFFFL;
            counts[i] = pb.getLong();
            indexN[i] = pb.getInt() & 0xFFFFFFFFL;
        }
        float[] rpms = rpmFromCounts(tUs, counts, ABI_STEPS_PER_REV, HOST_VEL_WIN);
        List<float[]> rows = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            float rpm = rpms[i];
            float dir = rpm > 0.5f ? 1f : (rpm < -0.5f ? -1f : 0f);
            rows.add(new float[]{tUs[i] / 1000f, rpm, dir, 1, indexN[i], 0, 0});
        }
        return new Result(n, hz, rows);
    }

    /** 多拍差分转速；EMA/大窗平滑留给曲线 UI。 */
    static float[] rpmFromCounts(long[] tUs, long[] counts, int steps, int velWin) {
        int n = counts.length;
        float[] out = new float[n];
        if (n == 0 || steps <= 0) {
            return out;
        }
        int w = Math.max(1, velWin);
        for (int i = 0; i < n; i++) {
            int j = i >= w ? i - w : 0;
            if (i == j) {
                out[i] = 0f;
                continue;
            }
            long dc = counts[i] - counts[j];
            long dt = tUs[i] - tUs[j];
            if (dt <= 0) {
                out[i] = i > 0 ? out[i - 1] : 0f;
                continue;
            }
            out[i] = (float) ((dc / (double) steps) * (1_000_000.0 / dt) * 60.0);
        }
        return out;
    }
}
