package org.firstinspires.ftc.teamcode.utility.filter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.fail;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer.Snapshot;
import org.junit.Test;

/** {@link ReplayBuffer} 单元测试：时间戳单调、floor/after 查询、淘汰与封装。 */
public class ReplayBufferTest {

    private static SimpleMatrix diag(double a, double b, double c) {
        return new SimpleMatrix(new double[][]{
                {a, 0, 0},
                {0, b, 0},
                {0, 0, c}
        });
    }

    /** 测试用默认过程噪声 Q。 */
    private static SimpleMatrix defaultQ() {
        return diag(0.002, 0.002, 0.002);
    }

    /** 空缓冲：floor/after 均无结果。 */
    @Test
    public void emptyBuffer() {
        ReplayBuffer buf = new ReplayBuffer();
        assertEquals(0, buf.size());
        assertNull(buf.floor(1.0));
        assertEquals(0, buf.after(0.0).size());
    }

    /** 乱序或重复时间戳被丢弃，保持严格递增。 */
    @Test
    public void addRejectsNonMonotonicTimestamps() {
        ReplayBuffer buf = new ReplayBuffer(10);
        buf.add(1.0, new double[]{0, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        buf.add(2.0, new double[]{1, 0, 0}, diag(1, 1, 1), defaultQ(), 1, 0, 0);

        buf.add(1.5, new double[]{9, 9, 9}, diag(9, 9, 9), defaultQ(), 0, 0, 0); // 乱序
        buf.add(2.0, new double[]{9, 9, 9}, diag(9, 9, 9), defaultQ(), 0, 0, 0); // 重复

        assertEquals(2, buf.size());
        Snapshot latest = buf.floor(2.0);
        assertNotNull(latest);
        assertEquals(2.0, latest.t, 1e-9);
        assertEquals(1.0, latest.getX()[0], 1e-9);
    }

    /** floor 返回不晚于 t 的最新快照；早于窗口时返回 null。 */
    @Test
    public void floorReturnsLatestNotAfter() {
        ReplayBuffer buf = new ReplayBuffer(10);
        buf.add(1.0, new double[]{1, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        buf.add(3.0, new double[]{3, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        buf.add(5.0, new double[]{5, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);

        assertNull(buf.floor(0.5));
        assertEquals(1.0, buf.floor(1.0).t, 1e-9);
        assertEquals(3.0, buf.floor(4.0).t, 1e-9);
        assertEquals(5.0, buf.floor(5.1).t, 1e-9);
    }

    /** after 仅返回严格大于 t 的快照，且按时间升序。 */
    @Test
    public void afterReturnsStrictlyLaterInOrder() {
        ReplayBuffer buf = new ReplayBuffer(10);
        buf.add(1.0, new double[]{1, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        buf.add(3.0, new double[]{3, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        buf.add(5.0, new double[]{5, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);

        assertEquals(3, buf.after(0.0).size());
        assertEquals(1, buf.after(3.0).size());
        assertEquals(5.0, buf.after(3.0).get(0).t, 1e-9);
        assertEquals(0, buf.after(5.0).size());
    }

    /** 超过保留时长的旧快照被淘汰，但至少保留 2 个。 */
    @Test
    public void evictsOldSamplesButKeepsAtLeastTwo() {
        ReplayBuffer buf = new ReplayBuffer(1.0);
        for (int i = 0; i <= 10; i++) {
            double t = i * 0.5;
            buf.add(t, new double[]{t, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        }
        // t=5.0 时淘汰阈值 4.0，仅保留 4.0 / 4.5 / 5.0
        assertEquals(3, buf.size());
        assertNull(buf.floor(3.9));
        assertEquals(4.0, buf.floor(4.0).t, 1e-9);

        // 极短保留时长仍保留最近 2 个
        ReplayBuffer tiny = new ReplayBuffer(0.001);
        tiny.add(0.0, new double[]{0, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        tiny.add(100.0, new double[]{0, 0, 0}, diag(1, 1, 1), defaultQ(), 0, 0, 0);
        assertEquals(2, tiny.size());
    }

    /** 非法保留时长在构造时即报错。 */
    @Test
    public void rejectsNonPositiveDuration() {
        assertInvalidDuration(0.0);
        assertInvalidDuration(-1.0);
        assertInvalidDuration(Double.NaN);
    }

    /** 默认保留时长与 WPILib PoseEstimator 的 1.5 s 一致。 */
    @Test
    public void defaultDurationMatchesWpiLibBuffer() {
        assertEquals(1.5, ReplayBuffer.DEFAULT_DURATION, 1e-9);
    }

    /** 快照对外只暴露副本：外部修改入参或返回值都不影响缓冲。 */
    @Test
    public void snapshotCopiesAreDefensive() {
        ReplayBuffer buf = new ReplayBuffer(10);
        double[] x = {1, 2, 3};
        SimpleMatrix p = diag(1, 2, 3);
        buf.add(1.0, x, p, defaultQ(), 0, 0, 0);

        x[0] = 99;
        p.set(0, 0, 99); // 修改入参

        Snapshot s = buf.floor(1.0);
        assertEquals(1.0, s.getX()[0], 1e-9);
        assertEquals(1.0, s.getP().get(0, 0), 1e-9);

        s.getX()[1] = 77;
        s.getP().set(1, 1, 88); // 修改返回值

        assertEquals(2.0, buf.floor(1.0).getX()[1], 1e-9);
        assertEquals(2.0, buf.floor(1.0).getP().get(1, 1), 1e-9);
    }

    /** refresh 能回写重放后的新状态。 */
    @Test
    public void refreshRewritesSnapshotState() {
        ReplayBuffer buf = new ReplayBuffer(10);
        buf.add(1.0, new double[]{1, 1, 1}, diag(1, 1, 1), defaultQ(), 0, 0, 0);

        Snapshot s = buf.floor(1.0);
        s.refresh(new double[]{5, 6, 7}, diag(5, 6, 7));

        assertEquals(5.0, buf.floor(1.0).getX()[0], 1e-9);
        assertEquals(7.0, buf.floor(1.0).getX()[2], 1e-9);
        assertEquals(6.0, buf.floor(1.0).getP().get(1, 1), 1e-9);
    }

    /** 快照记录该段传播所用的历史 Q，且对入参为防御性拷贝。 */
    @Test
    public void snapshotStoresAndCopiesQ() {
        ReplayBuffer buf = new ReplayBuffer(10);
        SimpleMatrix q = diag(0.1, 0.2, 0.3);
        buf.add(1.0, new double[]{0, 0, 0}, diag(1, 1, 1), q, 0, 0, 0);
        q.set(0, 0, 99); // 修改入参

        Snapshot s = buf.floor(1.0);
        assertEquals(0.1, s.getQ().get(0, 0), 1e-9);
        assertEquals(0.2, s.getQ().get(1, 1), 1e-9);
        assertEquals(0.3, s.getQ().get(2, 2), 1e-9);
    }

    private static void assertInvalidDuration(double duration) {
        try {
            new ReplayBuffer(duration);
            fail("expected IllegalArgumentException for duration=" + duration);
        } catch (IllegalArgumentException expected) {
            // 预期
        }
    }
}
