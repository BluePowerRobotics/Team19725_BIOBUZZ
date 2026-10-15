package org.firstinspires.ftc.teamcode.utility.Shooter;

/**
 * 飞轮（发射机构）速度环计算器 —— <b>纯计算类</b>，不持有任何硬件引用。
 *
 * <p>本类只做一件事：{@code (目标转速 ω*, 当前转速 ω, dt) → 占空比 duty}。
 * 电机、遥测、硬件初始化全部留在调用方
 * （{@code Controllers.Shooter.PrototypeShooter.PrototypeShooter}），
 * 因此本类可离线跑（给一串转速序列即可复算整条控制轨迹），也便于单独做数值验证。
 *
 * <p><b>控制律</b>（静摩擦前馈 + 一阶稳态前馈 + PI 反馈 + 掉速检测触发补偿）：
 * <pre>
 *   duty = kS·sgn(ω*) + Kff·ω* + Kp·err + Ki·∫err + boost
 *   err  = ω* - ω
 *   duty = sat(duty, dutyMin, dutyMax)
 * </pre>
 * 其中 ω* 为目标转速、ω 为实测转速，单位统一为 {@code DcMotorEx#getVelocity()} 的 tick/s；
 * 所有增益的量纲均为「占空比」，因此不读取电压传感器，本类直接输出占空比。
 *
 * <p><b>为什么要单独的 kS（静摩擦前馈）</b>：稳态实测占空比与转速的关系不是过原点的直线，
 * 而是 {@code d_ss(ω) = kS + Kff·ω}（kS 来自静摩擦、同步带预紧与齿槽转矩：这部分占空比不产生转速，
 * 只用来「推动」机构）。若只用一个 Kff 过原点拟合，低转速段会系统性欠给、高转速段系统性过给，
 * 差额全部压给积分项，表现为积分长期停在非零值（2026-09-04 实测各工作点 I 项都是 −0.04~−0.06）。
 * 拆出 kS 后积分得以真正回零，换速过渡与掉速补偿的起点都更干净。
 * 注意 {@code sgn(ω*)} 不可省：ω*=0 时该项必须为 0，否则停转指令下飞轮会自己转起来。
 *
 * <p><b>I 项语义（上限 / 生效节点 / 清零节点）</b>
 * <ul>
 *   <li>上限：对积分项 {@code Ki·∫err} 限幅到 ±{@link #maxI}，饱和时把内部积分回算为 ±maxI/Ki，
 *       防止积分无界增长（抗饱和）。</li>
 *   <li>生效节点：仅当 {@code targetVel != 0} 且 {@code |err| < iZone} 时才累加积分。</li>
 *   <li>清零节点：① 停机（targetVel == 0）；② 误差带外（|err| ≥ iZone）；③ 触发掉速补偿时。</li>
 * </ul>
 *
 * <p><b>掉速检测的「武装」规则</b>：只有当误差回到 {@link #dropThreshold} 以内（即系统已进入稳态）
 * 之后，掉速检测才被允许触发补偿；目标转速一旦改变立刻重新解除武装。这样从 0 拉起时的正常加速段
 * 不会被误判成掉速（否则 boost 会全程挂着并在到速后造成过冲）。
 *
 * <p><b>参数同步约定</b>：{@link #kff} 等增益字段是 public 可变字段，<b>由调用方每帧同步</b>
 * （见 {@code PrototypeShooter#syncCalculator()}）。这样参数仍留在调用方的 {@code @Config} 静态字段上，
 * FTC Dashboard 的分组与现场整定习惯都不受影响。下面的默认值取自 2026-09-04 实机整定结果
 * （kS=0.02945、kff=0.00038307、kp=0.00181917、ki=0.00259224），仅在独立使用本类时作为合理起点。
 */
public class FlywheelCalculator {

    // ==================== 可调参数（调用方每帧同步；单位见注释） ====================

    /**
     * 静摩擦前馈 (占空比)：稳态模型 {@code d_ss(ω) = kS + Kff·ω} 的截距，
     * 即「只对抗摩擦、不产生转速」的那份占空比，按 {@code kS·sgn(ω*)} 施加。
     */
    public double kS = 0.02945;
    /** 稳态前馈增益 (占空比)/(tick/s)，稳态模型斜率 */
    public double kff = 0.00038307;
    /** 比例增益 (占空比)/(tick/s) */
    public double kp = 0.00181917;
    /** 积分增益 (占空比)/(tick/s·s) */
    public double ki = 0.00259224;

    /** I 项上限 (占空比)：对积分项 Ki·∫err 的限幅 */
    public double maxI = 0.3;
    /** I 项生效节点 (tick/s)：|err| 小于该值时才累加积分 */
    public double iZone = 500.0;

    /** 掉速判定阈值 (tick/s)：err 超过该值判定为掉速 */
    public double dropThreshold = 100.0;
    /** 掉速速率阈值 (tick/s²)：dω/dt 低于其负值判定为掉速 */
    public double dropRateThreshold = 12000.0;

    /** 掉速补偿增益 (占空比)/(tick/s) */
    public double kBoost = 0.00362834;
    /** 掉速补偿上限 (占空比) */
    public double boostMax = 0.4;
    /** 掉速补偿持续时长 (s)，超时后自动退出 */
    public double boostDurationSec = 0.3;

    /** 输出下限 (占空比) */
    public double dutyMin = 0.0;
    /** 输出上限 (占空比) */
    public double dutyMax = 1.0;

    // ==================== 运行时状态 ====================

    /** 最近一次输出的占空比（供遥测） */
    private double duty = 0;

    /** 最近一次控制量各分量的值（供遥测：静摩擦 / 前馈 / 比例 / 积分 / 补偿） */
    private double lastKSTerm = 0, lastFFTerm = 0, lastPTerm = 0, lastITerm = 0, lastBoostTerm = 0;

    /** 积分累计量 ∫err·dt (tick/s·s) */
    private double integral = 0;

    /** 上一次的实测转速 (tick/s)，用于估计 dω/dt */
    private double previousVelocity = 0;

    /** 上一次 calculate 的时间戳 (s) */
    private double lastTime;

    /** 是否处于掉速补偿状态 */
    private boolean boostActive = false;

    /** 补偿已持续时长 (s) */
    private double boostTimer = 0;

    /** 掉速检测是否已「武装」：只有进入稳态（误差回到阈值内）后才允许触发补偿 */
    private boolean dropArmed = false;

    /** 上一次的目标转速，用于在目标改变时重新解除武装 */
    private double lastTargetVel = Double.NaN;

    /** dt 异常（首帧、长时间停顿、时钟回退）时使用的标称循环周期 (s)，避免微分项发散 */
    public static final double DEFAULT_DT = 0.02;

    public FlywheelCalculator() {
        lastTime = now();
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    // ==================== 计算 ====================

    /**
     * 单步计算：内部自行推进 dt（用单调时钟）。
     *
     * <p>每帧只可调用一次（依赖内部计时器推进 dt、维护积分与补偿状态）。
     *
     * @param targetVel  目标转速 (tick/s)，0 表示停转
     * @param currentVel 当前转速 (tick/s)
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel) {
        double currentTime = now();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        return calculate(targetVel, currentVel, dt);
    }

    /**
     * 单步计算（显式 dt）—— 纯函数入口，供离线复算与单元测试使用，不读写内部计时器。
     *
     * @param targetVel  目标转速 (tick/s)
     * @param currentVel 当前转速 (tick/s)
     * @param dt         距上一步的时间 (s)；≤0 或 &gt;1 时用 {@link #DEFAULT_DT} 兜底
     * @return 占空比，已限幅到 [dutyMin, dutyMax]
     */
    public double calculate(double targetVel, double currentVel, double dt) {
        if (dt <= 0 || dt > 1.0) {
            dt = DEFAULT_DT;
        }

        double error = targetVel - currentVel;
        double dVelocity = (currentVel - previousVelocity) / dt;
        previousVelocity = currentVel;

        // ---- 掉速检测「武装」：只有进入稳态（误差回到阈值内）后才允许触发补偿，
        //      否则从 0 拉起时的正常加速段会被误判为掉速，导致 boost 全程挂着并造成过冲 ----
        if (targetVel != lastTargetVel) {
            dropArmed = false; // 目标改变 → 重新等待进入稳态
            lastTargetVel = targetVel;
        }
        if (Math.abs(error) < dropThreshold) {
            dropArmed = true;
        }

        // ---- 掉速检测与触发补偿 ----
        boolean dropDetected = dropArmed && targetVel != 0
                && (error > dropThreshold || dVelocity < -dropRateThreshold);
        if (dropDetected) {
            boostActive = true;
            boostTimer = 0.0;
            integral = 0.0; // 清零节点③：触发补偿时清零，避免与 boost 叠加
        }
        if (boostActive) {
            boostTimer += dt;
            if (boostTimer >= boostDurationSec || error <= 0) {
                boostActive = false; // 恢复到位或超时退出
            }
        }

        // ---- 积分：生效节点 / 清零节点 ----
        boolean iActive = targetVel != 0 && Math.abs(error) < iZone;
        if (iActive) {
            integral += error * dt; // 生效节点：目标非 0 且 |err| < iZone
        } else {
            integral = 0.0; // 清零节点①②：停机 或 |err| ≥ iZone
        }
        double iTerm = ki * integral;
        // 上限：对积分项回算限幅，防止积分饱和
        if (iTerm > maxI) {
            iTerm = maxI;
            if (ki != 0) integral = maxI / ki;
        } else if (iTerm < -maxI) {
            iTerm = -maxI;
            if (ki != 0) integral = -maxI / ki;
        }

        // ---- 静摩擦前馈 + 稳态前馈 + PI + 补偿，输出占空比 ----
        double boost = boostActive ? Math.max(0.0, Math.min(kBoost * error, boostMax)) : 0.0;
        // sgn(ω*)：ω*=0 时为 0，避免停转指令下静摩擦前馈把飞轮推转
        double ffSign = targetVel > 0 ? 1.0 : (targetVel < 0 ? -1.0 : 0.0);
        lastKSTerm = kS * ffSign;
        lastFFTerm = kff * targetVel;
        lastPTerm = kp * error;
        lastITerm = iTerm;
        lastBoostTerm = boost;
        double out = lastKSTerm + lastFFTerm + lastPTerm + lastITerm + lastBoostTerm;
        if (out > dutyMax) out = dutyMax;
        if (out < dutyMin) out = dutyMin;

        this.duty = out;
        return out;
    }

    // ==================== 状态重置 ====================

    /** 重置积分与微分状态（从开环切回闭环前调用），假定当前转速为 0 */
    public void reset() {
        reset(0.0);
    }

    /**
     * 重置积分与微分状态，并把「上一次转速」设为给定值（避免切回闭环的瞬间微分项冲击）。
     *
     * @param currentVelocity 当前实测转速 (tick/s)
     */
    public void reset(double currentVelocity) {
        integral = 0.0;
        previousVelocity = currentVelocity;
        boostActive = false;
        boostTimer = 0.0;
        dropArmed = false;
        lastTargetVel = Double.NaN;
        lastKSTerm = 0;
        lastFFTerm = 0;
        lastPTerm = 0;
        lastITerm = 0;
        lastBoostTerm = 0;
        duty = 0.0;
        lastTime = now();
    }

    // ==================== 读取 ====================

    /** @return 最近一次输出的占空比 */
    public double getDuty() {
        return duty;
    }

    /** @return 内部积分累计量 ∫err·dt (tick/s·s) */
    public double getIntegral() {
        return integral;
    }

    /** @return 最近一次的静摩擦前馈分量 kS·sgn(ω*)（占空比） */
    public double getKSTerm() {
        return lastKSTerm;
    }

    /** @return 最近一次的前馈分量 Kff·ω*（占空比，不含静摩擦项） */
    public double getFfTerm() {
        return lastFFTerm;
    }

    /** @return 最近一次的比例分量 Kp·err（占空比） */
    public double getPTerm() {
        return lastPTerm;
    }

    /** @return 最近一次的积分分量（占空比，已限幅） */
    public double getITerm() {
        return lastITerm;
    }

    /** @return 最近一次的掉速补偿分量（占空比） */
    public double getBoostTerm() {
        return lastBoostTerm;
    }

    /** @return 是否处于掉速补偿状态 */
    public boolean isBoostActive() {
        return boostActive;
    }

    /** @return 掉速检测是否已武装（已进入稳态、允许触发补偿） */
    public boolean isDropArmed() {
        return dropArmed;
    }
}
