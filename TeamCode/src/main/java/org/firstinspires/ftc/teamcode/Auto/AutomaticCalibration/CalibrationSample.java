package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

/**
 * 底盘标定过程中的单个采样点（不可变）。
 *
 * <p>每个采样点同时记录「施加的激励」与「两条独立的测量链路」：
 * <ul>
 *   <li><b>轮速链路</b>：由底盘编码器推出的 {@link #wheelVelTicks} 与差分得到的
 *       {@link #wheelAccelTicks}，用于前馈 S/V/A 的辨识；</li>
 *   <li><b>地面真值链路</b>：由融合定位器（Pinpoint 里程计 + Limelight 视觉 + EKF）
 *       给出的机器人线/角速度 {@link #poseVx} / {@link #poseVy} / {@link #poseOmega}，
 *       与轮速相互独立，用于打滑判据。</li>
 * </ul>
 *
 * <p>量纲统一到 Road Runner 底盘链路（与 {@code MecanumDrive} 一致）：
 * <ul>
 *   <li>{@code t}：秒（s）；</li>
 *   <li>{@code battVolts}：伏特（V），当前电池电压；</li>
 *   <li>{@code power}：无量纲，[-1, 1]，写入 {@code DcMotorEx.setPower} 的功率；</li>
 *   <li>{@code voltsEff}：伏特（V），等于 {@code power × battVolts}，
 *       即电机实际获得的有效电压，与 RR 模板的 {@code feedforward.compute(...)/voltage}
 *       逆运算完全对齐，使辨识结果与电池电压无关；</li>
 *   <li>{@code wheelVelTicks}：tick/s（编码器原始单位）；</li>
 *   <li>{@code wheelAccelTicks}：tick/s²；</li>
 *   <li>{@code poseVx} / {@code poseVy}：英寸/秒（场坐标系）；</li>
 *   <li>{@code poseOmega}：弧度/秒（场坐标系角速度）。</li>
 * </ul>
 *
 * <p>本类只承载数据，不含任何硬件依赖，可在 JVM 单元测试中直接构造。
 */
public final class CalibrationSample {

    /** 所属标定阶段名（与 {@code AutomaticChassisCalibration} 的状态机阶段同名） */
    public final String phase;
    /** 采样时刻，相对该阶段开始的时间，单位秒 */
    public final double t;
    /** 采样时的电池电压，单位伏特 */
    public final double battVolts;
    /** 施加的电机功率，无量纲 [-1, 1] */
    public final double power;
    /** 有效电压 = power × battVolts，单位伏特 */
    public final double voltsEff;
    /** 轮速（编码器原始单位），单位 tick/s */
    public final double wheelVelTicks;
    /** 轮加速度（差分），单位 tick/s² */
    public final double wheelAccelTicks;
    /** 地面真值 X 轴线速度（场坐标系），单位英寸/秒 */
    public final double poseVx;
    /** 地面真值 Y 轴线速度（场坐标系），单位英寸/秒 */
    public final double poseVy;
    /** 地面真值角速度（场坐标系），单位弧度/秒 */
    public final double poseOmega;

    public CalibrationSample(String phase, double t, double battVolts, double power, double voltsEff,
                             double wheelVelTicks, double wheelAccelTicks,
                             double poseVx, double poseVy, double poseOmega) {
        this.phase = phase;
        this.t = t;
        this.battVolts = battVolts;
        this.power = power;
        this.voltsEff = voltsEff;
        this.wheelVelTicks = wheelVelTicks;
        this.wheelAccelTicks = wheelAccelTicks;
        this.poseVx = poseVx;
        this.poseVy = poseVy;
        this.poseOmega = poseOmega;
    }

    /**
     * @return 地面真值合速度大小（场坐标系），单位英寸/秒。
     *         用于与轮速折算出的机器人速度比较，构成打滑判据。
     */
    public double poseSpeed() {
        return Math.hypot(poseVx, poseVy);
    }

    /**
     * 抛出 CSV 数据行（不含表头）。
     * 列顺序与 {@link CalibrationExporter#SAMPLES_HEADER} 严格一致。
     */
    public String toCsvRow() {
        return String.format("%s,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f",
                phase, t, battVolts, power, voltsEff,
                wheelVelTicks, wheelAccelTicks, poseVx, poseVy, poseOmega);
    }
}
