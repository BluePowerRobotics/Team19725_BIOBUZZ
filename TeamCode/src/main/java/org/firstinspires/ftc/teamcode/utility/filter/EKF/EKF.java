package org.firstinspires.ftc.teamcode.utility.filter.EKF;

import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer;

/**
 * 3维扩展卡尔曼滤波器，用于Pinpoint(里程计速度)和Limelight(视觉位姿)的融合定位。
 *
 * 状态向量: [x, y, theta] (全局坐标系下的机器人位姿)
 * 控制输入(predict): Pinpoint提供的机器人局部速度 [vx, vy, omega]
 * 观测(update): Limelight提供的全局位姿 [x, y, theta]
 *
 * 参考: Kou & Haggenmiller (2023), "Extended Kalman Filter State Estimation
 *       for Autonomous Competition Robots"
 */
public class EKF {

    /** 状态向量 [x, y, theta] (3x1) */
    private SimpleMatrix state;

    /** 状态协方差矩阵 (3x3) */
    private SimpleMatrix P;

    /** 过程噪声协方差矩阵 Q (3x3) */
    private SimpleMatrix Q;

    /** 观测噪声协方差矩阵 R (3x3) */
    private SimpleMatrix R;

    /** 观测矩阵 H = I (3x3)，因为观测直接就是状态 */
    private final SimpleMatrix H = SimpleMatrix.identity(3);

    /** 上一次predict的时间戳，用于计算dt */
    private Double lastPredictTime = null;

    /** 上一次update的时间戳，用于拒绝过时观测 */
    private Double lastUpdateTime = null;

    /** 是否启用视觉时间戳回滚重放 */
    private final boolean allowReplay;

    /** 回滚重放缓冲区 (仅 allowReplay 为 true 时创建) */
    private final ReplayBuffer replayBuffer;

    /** 回滚基准与观测时刻允许的最大间隔 (秒)；超过则放弃回滚 (与 predict 的 dt 保护一致) */
    private static final double MAX_REPLAY_GAP = 1.0;

    // ==================== 构造 ====================

    /**
     * @param initialX      初始x
     * @param initialY      初始y
     * @param initialTheta  初始朝向 (弧度)
     */
    public EKF(double initialX, double initialY, double initialTheta) {
        this(initialX, initialY, initialTheta, false);
    }

    /**
     * @param initialX      初始x
     * @param initialY      初始y
     * @param initialTheta  初始朝向 (弧度)
     * @param allowReplay   是否启用视觉时间戳回滚重放
     *                      (收到带延迟的视觉观测时先回滚到观测时刻更新，再重放里程计预测)
     */
    public EKF(double initialX, double initialY, double initialTheta, boolean allowReplay) {
        state = new SimpleMatrix(new double[][]{
                {initialX},
                {initialY},
                {initialTheta}
        });

        // 初始协方差：位置中等不确定，角度不确定较小
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });

        // 默认过程噪声 (dt相关，实际predict中会乘以dt)
        Q = new SimpleMatrix(new double[][]{
                {0.002, 0,     0     },
                {0,     0.002, 0     },
                {0,     0,     0.002 }
        });

        // 默认观测噪声（视觉测量通常角度噪声比位置噪声大）
        R = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.05}
        });

        this.allowReplay = allowReplay;
        this.replayBuffer = allowReplay ? new ReplayBuffer() : null;
    }

    // ==================== 核心滤波 ====================

    /**
     * 预测步骤 —— 每帧调用，接收Pinpoint提供的机器人局部速度。
     * 根据时间戳自动计算dt，首次调用仅记录时间戳不做预测。
     *
     * @param vx        机器人局部坐标系下的x方向速度
     * @param vy        机器人局部坐标系下的y方向速度
     * @param omega     机器人角速度 (弧度/秒)
     * @param timestamp 当前时间戳 (秒)，需与update使用同一时间基准
     */
    public void predict(double vx, double vy, double omega, double timestamp) {
        // 首次调用：仅记录时间戳，不做预测
        if (lastPredictTime == null) {
            lastPredictTime = timestamp;
            if (allowReplay) {
                replayBuffer.add(timestamp, getPose(), P, Q, 0, 0, 0);
            }
            return;
        }

        double dt = timestamp - lastPredictTime;
        lastPredictTime = timestamp;

        // 异常dt保护
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        applyMotion(vx, vy, omega, dt, Q);

        // 记录快照供视觉回滚重放使用 (连同本段所用的历史 Q)
        if (allowReplay) {
            replayBuffer.add(timestamp, getPose(), P, Q, vx, vy, omega);
        }
    }

    /**
     * 状态与协方差传播 (非线性运动模型 + 雅可比线性化)，不含时间戳/缓冲逻辑。
     *
     * @param vx    机器人局部 x 方向速度 (in/s)
     * @param vy    机器人局部 y 方向速度 (in/s)
     * @param omega 角速度 (rad/s)
     * @param dt    传播时间 (秒)
     * @param Qused 本段传播使用的过程噪声 Q (常规 predict 传当前 Q；回放传快照的历史 Q)
     */
    private void applyMotion(double vx, double vy, double omega, double dt, SimpleMatrix Qused) {
        double x = state.get(0, 0);
        double y = state.get(1, 0);
        double theta = state.get(2, 0);

        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        // ---- 1. 状态传播 (非线性) ----
        // 将局部速度转换到全局坐标系
        double xNew = x + dt * (vx * cosTheta - vy * sinTheta);
        double yNew = y + dt * (vx * sinTheta + vy * cosTheta);
        double thetaNew = theta + dt * omega;

        // ---- 2. 计算雅可比矩阵 A = df/dx (3x3) ----
        // A[i][j] = ∂f_i / ∂x_j
        SimpleMatrix A = new SimpleMatrix(new double[][]{
                {1, 0, dt * (-vx * sinTheta - vy * cosTheta)},
                {0, 1, dt * ( vx * cosTheta - vy * sinTheta)},
                {0, 0, 1                                     }
        });

        // ---- 3. 协方差传播 P = A * P * A^T + Q*dt ----
        // Q 乘以 dt 体现过程噪声随时间累积；回放时传入该段快照记录的历史 Q
        SimpleMatrix Qdt = Qused.scale(dt);
        P = A.mult(P).mult(A.transpose()).plus(Qdt);

        // ---- 4. 更新状态 ----
        state = new SimpleMatrix(new double[][]{
                {xNew},
                {yNew},
                {normalizeAngle(thetaNew)}
        });
    }

    /**
     * 更新步骤 —— 仅在Limelight检测有效时调用，接收视觉全局位姿。
     * 会对比时间戳，拒绝比上次update更旧的观测数据。
     *
     * @param xMeas        Limelight测得的全局x
     * @param yMeas        Limelight测得的全局y
     * @param thetaMeas    Limelight测得的全局朝向 (弧度)
     * @param timestamp    观测时间戳 (秒)
     */
    public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        // 拒绝过时观测 (不论是否启用重放都保留)
        if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
            return;
        }

        if (allowReplay) {
            // 回滚到观测时刻更新，再重放其后预测；观测早于缓冲窗口时被丢弃
            if (replayUpdate(xMeas, yMeas, thetaMeas, timestamp)) {
                lastUpdateTime = timestamp;
            }
            return;
        }

        lastUpdateTime = timestamp;
        applyVisionUpdate(xMeas, yMeas, thetaMeas);
    }

    /**
     * 在当前状态处执行一次观测更新 (标准 EKF 更新步骤)。
     *
     * @param xMeas     视觉全局 x
     * @param yMeas     视觉全局 y
     * @param thetaMeas 视觉全局朝向 (弧度)
     */
    private void applyVisionUpdate(double xMeas, double yMeas, double thetaMeas) {
        // ---- 1. 观测向量 z = [xMeas, yMeas, thetaMeas]^T ----
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });

        // ---- 2. 新息 y_innov = z - H*x (H=I, 所以 y_innov = z - x) ----
        SimpleMatrix yInnov = z.minus(state);
        // 角度新息需要归一化到 [-π, π]
        yInnov.set(2, 0, normalizeAngle(yInnov.get(2, 0)));

        // ---- 3. 新息协方差 S = H * P * H^T + R ----
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

        // ---- 4. 卡尔曼增益 K = P * H^T * S^{-1} ----
        SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

        // ---- 5. 状态更新 x = x + K * y_innov ----
        state = state.plus(K.mult(yInnov));
        state.set(2, 0, normalizeAngle(state.get(2, 0)));

        // ---- 6. 协方差更新 P = (I - K*H) * P ----
        SimpleMatrix I = SimpleMatrix.identity(3);
        P = I.minus(K.mult(H)).mult(P);
    }

    /**
     * 回滚重放更新 —— 将滤波器回滚到视觉观测时刻、在该时刻完成更新，再重放其后的
     * 里程计预测回到当前时刻，从而消除视觉链路延迟造成的滞后偏差。
     *
     * @return true 表示观测已应用；false 表示观测早于缓冲窗口或与回滚基准间隔过大被丢弃
     */
    private boolean replayUpdate(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        ReplayBuffer.Snapshot base = replayBuffer.floor(timestamp);
        if (base == null) {
            // 观测早于缓冲窗口，无法回滚
            return false;
        }

        // 回滚基准与观测时刻间隔过大 (预测停更 / 时钟异常) 时放弃回滚，避免大 dt 重放
        if (timestamp - base.t > MAX_REPLAY_GAP) {
            return false;
        }

        // 观测时刻之后的所有快照 (时间戳均严格大于观测时刻)
        List<ReplayBuffer.Snapshot> tail = replayBuffer.after(base.t);

        // ---- 1. 回滚到观测时刻之前的最近快照 ----
        state = toColumn(base.getX());
        P = base.getP();

        // ---- 2. 从该快照推进到观测时刻 (使用其后第一段的输入与历史 Q) ----
        if (!tail.isEmpty()) {
            ReplayBuffer.Snapshot first = tail.get(0);
            applyMotion(first.vx, first.vy, first.omega, timestamp - base.t, first.getQ());
        }

        // ---- 3. 在观测时刻执行更新 ----
        // 注: R 是被应用的这次观测自身的噪声 (由本帧 adaptR 按该帧 stdDev 计算)，
        //     与时间轴无关，故沿用当前 R；只有 Q 是随时间变化的过程噪声，按快照历史 Q 回放。
        applyVisionUpdate(xMeas, yMeas, thetaMeas);

        // ---- 4. 重放其后所有预测回到当前时刻 (各段用各自快照的历史 Q)，并刷新快照 ----
        double prevT = timestamp;
        for (ReplayBuffer.Snapshot s : tail) {
            applyMotion(s.vx, s.vy, s.omega, s.t - prevT, s.getQ());
            prevT = s.t;
            s.refresh(getPose(), P);
        }
        return true;
    }

    /** 将 [x, y, theta] 转为 3x1 列向量。 */
    private SimpleMatrix toColumn(double[] v) {
        return new SimpleMatrix(new double[][]{
                {v[0]},
                {v[1]},
                {v[2]}
        });
    }

    /**
     * 视觉观测门控 —— 计算马氏距离 (Mahalanobis distance) 判断该视觉观测是否为离群点。
     *
     * <p>观测模型 H = I，因此新息 {@code y = z - x̂}，新息协方差 {@code S = P + R}。
     * 当机器人远离 AprilTag 时视觉误差极大，直接用其更新会将滤波器带偏；
     * 通过马氏距离门控可拒绝这类离群观测。
     *
     * @param xMeas         视觉全局 x (英寸)
     * @param yMeas         视觉全局 y (英寸)
     * @param thetaMeas     视觉全局朝向 (弧度)
     * @param gateThreshold 马氏距离门控阈值 (无量纲)
     * @return true 表示测量可接受 (马氏距离 ≤ gateThreshold)
     */
    public boolean gateVision(double xMeas, double yMeas, double thetaMeas, double gateThreshold) {
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });
        SimpleMatrix innov = z.minus(state);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        // H = I → S = P + R
        SimpleMatrix S = P.plus(R);
        SimpleMatrix d2 = innov.transpose().mult(safeInvert(S)).mult(innov);
        return d2.get(0, 0) <= gateThreshold * gateThreshold;
    }

    // ==================== 输出 ====================

    /**
     * @return 当前融合位姿 double[3] = {x, y, theta}
     */
    public double[] getPose() {
        return new double[]{
                state.get(0, 0),
                state.get(1, 0),
                state.get(2, 0)
        };
    }

    /**
     * @return 状态协方差矩阵的对角线元素 {var_x, var_y, var_theta}
     */
    public double[] getCovarianceDiag() {
        return new double[]{
                P.get(0, 0),
                P.get(1, 1),
                P.get(2, 2)
        };
    }

    /**
     * @return 当前过程噪声矩阵 Q 的对角线元素 {q_x, q_y, q_theta}
     */
    public double[] getQDiag() {
        return new double[]{
                Q.get(0, 0),
                Q.get(1, 1),
                Q.get(2, 2)
        };
    }

    /**
     * @return 当前观测噪声矩阵 R 的对角线元素 {r_x, r_y, r_theta}
     */
    public double[] getRDiag() {
        return new double[]{
                R.get(0, 0),
                R.get(1, 1),
                R.get(2, 2)
        };
    }

    // ==================== 噪声参数调整 ====================

    /**
     * 实时设置过程噪声协方差矩阵 Q 的对角线值。
     * 值越大 = 越信任Pinpoint速度输入，但同时累积误差增长越快。
     *
     * @param qx       x方向过程噪声
     * @param qy       y方向过程噪声
     * @param qtheta   theta方向过程噪声
     */
    public void setQ(double qx, double qy, double qtheta) {
        Q = new SimpleMatrix(new double[][]{
                {qx, 0,  0     },
                {0,  qy, 0     },
                {0,  0,  qtheta}
        });
    }

    /**
     * 直接设置完整的 Q 矩阵 (3x3)。
     * 由自适应逻辑传入已构建好的 SimpleMatrix 对象。
     */
    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    /**
     * 实时设置观测噪声协方差矩阵 R 的对角线值。
     * 值越小 = 越信任Limelight观测。
     *
     * @param rx       x方向观测噪声
     * @param ry       y方向观测噪声
     * @param rtheta   theta方向观测噪声
     */
    public void setR(double rx, double ry, double rtheta) {
        R = new SimpleMatrix(new double[][]{
                {rx, 0,  0     },
                {0,  ry, 0     },
                {0,  0,  rtheta}
        });
    }

    /**
     * 直接设置完整的 R 矩阵 (3x3)。
     * 由自适应逻辑传入已构建好的 SimpleMatrix 对象。
     */
    public void setR(SimpleMatrix R) {
        this.R = R;
    }

    // ==================== 重置 ====================

    /**
     * 重置滤波器到新的初始位姿，同时清空时间戳。
     */
    public void reset(double x, double y, double theta) {
        state = new SimpleMatrix(new double[][]{
                {x},
                {y},
                {theta}
        });
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });
        lastPredictTime = null;
        lastUpdateTime = null;
        if (replayBuffer != null) {
            replayBuffer.clear();
        }
    }

    // ==================== 回滚重放状态 ====================

    /** @return 是否启用视觉时间戳回滚重放 */
    public boolean isReplayEnabled() {
        return allowReplay;
    }

    /** @return 回滚缓冲中的快照数量 (未启用重放时为 0) */
    public int getReplayBufferSize() {
        return replayBuffer == null ? 0 : replayBuffer.size();
    }

    // ==================== 内部工具 ====================

    /**
     * 将角度归一化到 [-π, π] 范围。
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * 安全求逆 —— S = H·P·Hᵀ + R 在 R 配置过小或数值退化时可能奇异。
     * 求逆失败时对角加扰动 (jitter) 后重试, 避免抛出异常或产生爆炸增益。
     */
    private SimpleMatrix safeInvert(SimpleMatrix m) {
        try {
            return m.invert();
        } catch (RuntimeException e) {
            SimpleMatrix jittered = m.plus(SimpleMatrix.identity(m.numRows()).scale(1e-9));
            return jittered.invert();
        }
    }
}