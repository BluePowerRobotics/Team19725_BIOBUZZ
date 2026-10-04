package org.firstinspires.ftc.teamcode.utility.filter;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

/**
 * 视觉时间戳回滚重放缓冲区。
 *
 * <p>保存最近一段时间内每次 {@code predict} 之后的滤波器状态快照
 * (状态 {@code x}、协方差 {@code P}、当时使用的过程噪声 {@code Q}，以及产生该快照
 * 所用的速度输入)。当收到带
 * 延迟的视觉观测时，可先将滤波器回滚到观测时刻、在该时刻完成更新，再重放其后
 * 的里程计预测回到当前时刻，从而消除视觉链路延迟造成的滞后偏差。
 *
 * <p>该机制在思路上参考 WPILib {@code PoseEstimator} 的视觉延迟补偿 (把带时间戳
 * 的观测对齐到其真实发生时刻再融合)，但实现方式不同：{@code PoseEstimator} 对里程计
 * 位姿缓冲做插值并施加统一的位姿变换，而本类采用<b>滤波器状态回滚 + 运动模型重放</b>，
 * 更契合 EKF/UKF 这类递推滤波器，二者不可互推。
 *
 * <p>快照时间戳必须严格递增，由 {@link #add} 强制保证 (乱序样本被丢弃)；早于
 * {@link #DEFAULT_DURATION} 的旧快照会被淘汰。
 */
public final class ReplayBuffer {

    /** 默认保留时长 (秒)，与 WPILib PoseEstimator 的 1.5 s 缓冲一致。 */
    public static final double DEFAULT_DURATION = 1.5;

    /** 单帧状态快照。 */
    public static final class Snapshot {
        /** 快照时间戳 (秒)。 */
        public final double t;
        /** 状态 [x, y, theta]。 */
        private double[] x;
        /** 状态协方差 (3x3)。 */
        private SimpleMatrix P;
        /**
         * 传播到本快照所用的过程噪声协方差 Q (3x3)。
         *
         * <p>回放时必须复用当时的历史 Q，而不是滤波器当前的 Q：自适应策略下 Q 会随 IMU
         * 冲击/坡度变化逐帧跳变 (见 Theory.md §4.4)，若用当前 Q 回放整段历史，会把最新的
         * 噪声量重新灌进过去该区间的协方差，导致重放结果与原始传播不一致。
         */
        private final SimpleMatrix Q;
        /** 由上一快照传播到本快照所用的控制输入 (机器人局部速度)。 */
        public final double vx;
        public final double vy;
        public final double omega;

        Snapshot(double t, double[] x, SimpleMatrix P, SimpleMatrix Q,
                 double vx, double vy, double omega) {
            this.t = t;
            this.x = x.clone();
            this.P = P.copy();
            this.Q = Q.copy();
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }

        /** @return 状态 [x, y, theta] 的副本 */
        public double[] getX() {
            return x.clone();
        }

        /** @return 状态协方差 (3x3) 的副本 */
        public SimpleMatrix getP() {
            return P.copy();
        }

        /** @return 传播到本快照所用的过程噪声 Q (3x3) 的副本 */
        public SimpleMatrix getQ() {
            return Q.copy();
        }

        /**
         * 用重放后的新状态刷新本快照 (回滚重放完成后回写)。
         *
         * @param x 新的状态 [x, y, theta]
         * @param P 新的状态协方差 (3x3)
         */
        public void refresh(double[] x, SimpleMatrix P) {
            this.x = x.clone();
            this.P = P.copy();
        }
    }

    private final double duration;
    private final ArrayDeque<Snapshot> samples = new ArrayDeque<>();

    public ReplayBuffer() {
        this(DEFAULT_DURATION);
    }

    public ReplayBuffer(double duration) {
        if (!(duration > 0)) {
            throw new IllegalArgumentException("duration must be positive, got " + duration);
        }
        this.duration = duration;
    }

    public boolean isEmpty() {
        return samples.isEmpty();
    }

    public int size() {
        return samples.size();
    }

    public void clear() {
        samples.clear();
    }

    /**
     * 记录一次 predict 之后的快照。
     *
     * @param t     快照时间戳 (秒)
     * @param x     状态 [x, y, theta]
     * @param P     状态协方差 (会被拷贝)
     * @param Q     传播到本快照所用的过程噪声协方差 (会被拷贝)，回放时按历史值复用
     * @param vx    从上一快照到本快照所用的输入 vx (in/s)
     * @param vy    从上一快照到本快照所用的输入 vy (in/s)
     * @param omega 从上一快照到本快照所用的输入 omega (rad/s)
     */
    public void add(double t, double[] x, SimpleMatrix P, SimpleMatrix Q,
                    double vx, double vy, double omega) {
        // 强制时间戳严格递增：乱序样本会破坏 floor()/after() 的单调假设，直接丢弃
        if (!samples.isEmpty() && t <= samples.peekLast().t) {
            return;
        }

        samples.addLast(new Snapshot(t, x, P, Q, vx, vy, omega));

        // 淘汰过旧快照；至少保留 2 个以支持回滚
        while (samples.size() > 2 && samples.peekFirst().t < t - duration) {
            samples.pollFirst();
        }
    }

    /**
     * 返回时间戳不大于 {@code t} 的最新快照。
     *
     * @return 若所有快照都晚于 {@code t} (观测早于缓冲窗口) 则返回 null
     */
    public Snapshot floor(double t) {
        Snapshot result = null;
        for (Snapshot s : samples) {
            if (s.t <= t) {
                result = s;
            } else {
                break;
            }
        }
        return result;
    }

    /**
     * 返回时间戳严格大于 {@code t} 的所有快照，按时间升序。
     */
    public List<Snapshot> after(double t) {
        List<Snapshot> list = new ArrayList<>();
        for (Snapshot s : samples) {
            if (s.t > t) {
                list.add(s);
            }
        }
        return list;
    }
}
