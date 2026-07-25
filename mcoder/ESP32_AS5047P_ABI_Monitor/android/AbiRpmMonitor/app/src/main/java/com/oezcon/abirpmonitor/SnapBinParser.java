package com.oezcon.abirpmonitor;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

/**
 * 与固件 snap / PC parse_snap_bindump 一致：magic+n+hz+payload+crc32。
 */
public final class SnapBinParser {
    public static final int MAGIC = 0xAB1C0001;
    public static final int POINT_SIZE = 12;

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
        if (magic != MAGIC) {
            throw new IllegalArgumentException(String.format("bad magic 0x%08X", magic));
        }
        int n = bb.getShort() & 0xFFFF;
        int hz = bb.getShort() & 0xFFFF;
        int need = 8 + n * POINT_SIZE + 4;
        if (raw.length < need) {
            throw new IllegalArgumentException("bin len " + raw.length + " < need " + need);
        }
        int payloadOff = 8;
        int payloadLen = n * POINT_SIZE;
        int expectCrc = ByteBuffer.wrap(raw, payloadOff + payloadLen, 4)
                .order(ByteOrder.LITTLE_ENDIAN).getInt();
        int gotCrc = crc32(raw, payloadOff, payloadLen);
        if (gotCrc != expectCrc) {
            throw new IllegalArgumentException(
                    String.format("CRC mismatch got=0x%08X expect=0x%08X", gotCrc, expectCrc));
        }
        List<float[]> rows = new ArrayList<>(n);
        ByteBuffer pb = ByteBuffer.wrap(raw, payloadOff, payloadLen).order(ByteOrder.LITTLE_ENDIAN);
        for (int i = 0; i < n; i++) {
            long tUs = pb.getInt() & 0xFFFFFFFFL;
            short rpmX10 = pb.getShort();
            byte dir = pb.get();
            pb.get(); // pad
            long indexN = pb.getInt() & 0xFFFFFFFFL;
            float tMs = tUs / 1000f;
            float rpm = rpmX10 / 10f;
            rows.add(new float[]{tMs, rpm, dir, 1, indexN, 0, 0});
        }
        return new Result(n, hz, rows);
    }
}
