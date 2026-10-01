package org.firstinspires.ftc.teamcode.utility.PID;

import java.util.HashMap;
import java.util.Map;

/**
 * PIDSVAController类实现了带有SVA前馈的PID控制器
 * 支持多slot配置，可根据不同场景切换参数
 */
public class PIDSVAController {
    /** 存储不同slot的配置 */
    private final Map<Integer, SlotConfig> slots = new HashMap<>();
    /** 当前使用的slot */
    private int currentSlot = 0;
    /** 积分值 */
    private double integral = 0;
    /** 上一次的误差值 */
    private double previousError = 0;
    /** 上一次的 setpoint，用于由 setpoint 变化率推导参考速度（位置闭环）；NaN 表示尚无历史 */
    private double previousSetpoint = Double.NaN;
    /** 上一次的参考速度，用于推导参考加速度；NaN 表示尚无历史 */
    private double previousRefVelocity = Double.NaN;

    // ==================== 最近一次输出的分量（供实时显示/诊断） ====================
    /** 最近一次总输出 */
    private double lastOutput = 0;
    /** 最近一次 P 项贡献 */
    private double lastPTerm = 0;
    /** 最近一次 I 项贡献 */
    private double lastITerm = 0;
    /** 最近一次 D 项贡献 */
    private double lastDTerm = 0;
    /** 最近一次 S 项贡献 */
    private double lastSTerm = 0;
    /** 最近一次 V 项贡献 */
    private double lastVTerm = 0;
    /** 最近一次 A 项贡献 */
    private double lastATerm = 0;

    /**
     * 快速设置默认slot(0号slot)的PID和SVA参数
     * @param config SlotConfig配置对象
     * @return 当前PIDSVAController实例，用于链式调用
     */
    public PIDSVAController withSlot0(SlotConfig config) {
        currentSlot = 0;
        return withSlot(0, config);
    }

    /**
     * 设置第n个slot的PID和SVA参数
     * @param slot slot编号
     * @param config SlotConfig配置对象
     * @return 当前PIDSVAController实例，用于链式调用
     */
    public PIDSVAController withSlot(int slot, SlotConfig config) {
        currentSlot = slot;
        slots.put(slot, config);
        return this;
    }

    /**
     * 切换到第n个slot，重置积分和微分状态
     * @param slot 要切换到的slot编号
     * @throws IllegalArgumentException 如果slot未配置
     */
    public void setSlot(int slot) {
        if (!slots.containsKey(slot)) throw new IllegalArgumentException("Slot not configured");
        currentSlot = slot;
        reset();
    }

    /**
     * 重置默认slot(0号slot)的配置
     * @param config 新的SlotConfig配置对象
     * @throws IllegalArgumentException 如果0号slot未配置
     */
    public void resetSlot(SlotConfig config) {
        if (!slots.containsKey(0)) throw new IllegalArgumentException("Slot not configured");
        slots.put(0, config);
    }

    /**
     * 重置指定slot的配置
     * @param slot slot编号
     * @param config 新的SlotConfig配置对象
     * @throws IllegalArgumentException 如果指定slot未配置
     */
    public void resetSlot(int slot, SlotConfig config) {
        if (!slots.containsKey(slot)) throw new IllegalArgumentException("Slot not configured");
        slots.put(slot, config);
    }

    /**
     * PIDSVA闭环计算输出（速度闭环，{@code velocityLoop = true} 的简写）。
     * @param setpoint 目标速度
     * @param measurement 当前速度  
     * @param dt 时间间隔（秒），必须大于0
     * @return 控制器输出
     */
    public double calculate(double setpoint, double measurement, double dt) {
        return calculate(setpoint, measurement, dt, true);
    }

    /**
     * 完整PIDSVA闭环，计算输出。
     * 输入仅需 setpoint 与 measurement：SVA 前馈所需的参考速度/参考加速度由控制器
     * 内部根据 setpoint 的变化率推导，不使用实测速度，避免前馈与微分项相互抵消。
     * @param setpoint 目标值（速度闭环时为速度，位置闭环时为位置）
     * @param measurement 当前值（速度或位置）
     * @param dt 时间间隔（秒），必须大于0
     * @param velocityLoop true=速度闭环（setpoint 即参考速度）；false=位置闭环（参考速度由 setpoint 变化率推导）
     * @return 控制器输出
     * @throws IllegalArgumentException 当前slot未配置
     */
    public double calculate(double setpoint, double measurement, double dt, boolean velocityLoop) {
        // 获取当前slot的配置；未配置时快速失败，避免空指针
        SlotConfig cfg = slots.get(currentSlot);
        if (cfg == null) throw new IllegalArgumentException("Slot not configured: " + currentSlot);

        // 由 setpoint 推导参考速度/参考加速度，作为 SVA 前馈量（不使用实测速度）
        double refVelocity;
        if (velocityLoop) {
            // 速度闭环：setpoint 本身即参考速度
            refVelocity = setpoint;
        } else {
            // 位置闭环：参考速度 = d(setpoint)/dt，首帧无历史时取 0
            refVelocity = Double.isNaN(previousSetpoint) ? 0.0 : (setpoint - previousSetpoint) / dt;
        }
        // 参考加速度 = d(参考速度)/dt，首帧无历史时取 0
        double refAcceleration = Double.isNaN(previousRefVelocity) ? 0.0 : (refVelocity - previousRefVelocity) / dt;
        previousSetpoint = setpoint;
        previousRefVelocity = refVelocity;

        // 计算误差
        double error = setpoint - measurement;
        // 仅在误差小于Izone时才累加积分
        if (Math.abs(error) < cfg.iZone) {
            integral += error * dt;
            // 积分限幅
            if (integral > cfg.maxI) integral = cfg.maxI;
            if (integral < -cfg.maxI) integral = -cfg.maxI;
        } else {
            integral = 0; // 误差超出Izone时不积分
        }
        // 计算微分
        double derivative = (error - previousError) / dt;
        // 更新上一次误差
        previousError = error;
        // 计算PID输出
        double pid = cfg.kP * error + cfg.kI * integral + cfg.kD * derivative;
        // 计算SVA前馈输出（使用参考速度/加速度）
        double sva = cfg.kS * Math.signum(refVelocity) + cfg.kV * refVelocity + cfg.kA * refAcceleration;
        // 计算总输出
        double output = pid + sva;
        // 输出限幅
        if (output > cfg.outputMax) output = cfg.outputMax;
        if (output < cfg.outputMin) output = cfg.outputMin;
        // 记录各分量，供实时显示/诊断
        lastOutput = output;
        lastPTerm = cfg.kP * error;
        lastITerm = cfg.kI * integral;
        lastDTerm = cfg.kD * derivative;
        lastSTerm = cfg.kS * Math.signum(refVelocity);
        lastVTerm = cfg.kV * refVelocity;
        lastATerm = cfg.kA * refAcceleration;
        // 返回输出
        return output;
    }

    /**
     * 重置积分、微分状态以及 setpoint 历史（不切换slot）
     */
    public void reset() {
        integral = 0;
        previousError = 0;
        previousSetpoint = Double.NaN;
        previousRefVelocity = Double.NaN;
    }

    // ==================== 输出分量读取（供实时显示/诊断） ====================

    /** @return 最近一次总输出（限幅后） */
    public double getLastOutput() { return lastOutput; }
    /** @return 最近一次 P 项贡献 */
    public double getLastPTerm() { return lastPTerm; }
    /** @return 最近一次 I 项贡献 */
    public double getLastITerm() { return lastITerm; }
    /** @return 最近一次 D 项贡献 */
    public double getLastDTerm() { return lastDTerm; }
    /** @return 最近一次 S 项贡献 */
    public double getLastSTerm() { return lastSTerm; }
    /** @return 最近一次 V 项贡献 */
    public double getLastVTerm() { return lastVTerm; }
    /** @return 最近一次 A 项贡献 */
    public double getLastATerm() { return lastATerm; }
}