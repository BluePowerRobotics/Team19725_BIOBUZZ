package org.firstinspires.ftc.teamcode.架构练习;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 状态估计器（架构练习版）—— 对应架构图中位于 OpMode 与传感器之间的「中间层」。
 *
 * <p>在原工程里 {@code RobotPosition} 负责定位融合（EKF / 视觉 / 里程计）。本练习聚焦球道计数，
 * 因此本类只承担「球道状态估计」这一职责：持有 {@link BallCounter1}，把它每帧推进，并向
 * Sweeper / Turret / OpMode 暴露统一的计数接口，实现跨模块数据同步。
 *
 * <p><b>架构图中的四个函数与本类（及其下属）的对应关系</b>
 * <ul>
 *   <li>{@code IsBlocked}       → {@link #isBlocked()}</li>
 *   <li>{@code BallCount}       → {@link #getBallCount()}</li>
 *   <li>{@code resetBallCount}  → {@link #resetBallCount()}（只清计数，发射完毕 / 手动轻量清空）</li>
 *   <li>{@code reset}           → {@link #reset()}（深复位，手动强制复位）</li>
 * </ul>
 *
 * <p><b>数据同步约定</b>：只有 {@link #update(int)} 推进球道状态（内部再推进 {@link BallCounter1}
 * → {@link BreakBeamSensor1}），全工程只有这一个硬件采样点；其它模块只读 getter，不重复采样。
 *
 * <p><b>Init 顺序</b>：构造时先创建下属 {@link BallCounter1}（其内部再创建 {@link BreakBeamSensor1}），
 * 再初始化自身字段，避免未写先用。
 */
public class RobotPosition1 {

    /** 球道满仓球数：BallCount 达到该值即 {@link #isFull()} 为 true */
    public static int FULL_COUNT = 4;

    /** 下属对象：球道计数器（先声明，构造时先创建） */
    private final BallCounter1 ballCounter;

    /**
     * @param hardwareMap 硬件映射
     */
    public RobotPosition1(HardwareMap hardwareMap) {
        this.ballCounter = new BallCounter1(hardwareMap);
    }

    /**
     * 每帧推进球道状态（唯一硬件采样点）。
     *
     * @param mode 计数方向：{@link BallCounter1#MODE_INTAKE} / {@link BallCounter1#MODE_OUTPUT}
     */
    public void update(int mode) {
        ballCounter.update(mode);
    }

    // ==================== 只读接口（架构图中的四个函数） ====================

    /** @return IsBlocked：光电门当前是否遮挡（本帧已推进的结果） */
    public boolean isBlocked() {
        return ballCounter.isBlocked();
    }

    /** @return BallCount：当前累计球数（始终 ≥ 0） */
    public int getBallCount() {
        return ballCounter.getCount();
    }

    /** @return 是否满仓：BallCount ≥ {@link #FULL_COUNT} */
    public boolean isFull() {
        return getBallCount() >= FULL_COUNT;
    }

    // ==================== 清空（resetBallCount / reset） ====================

    /** resetBallCount：只清计数（用于炮台发射完毕 / 手动轻量清空），不动光电门滤波状态。 */
    public void resetBallCount() {
        ballCounter.clearCount();
    }

    /** reset：深复位——计数归零，同时重置光电门 EMA 滤波与边缘基准（手动强制复位）。 */
    public void reset() {
        ballCounter.reset();
    }

    // ==================== 遥测 ====================

    /** 输出球道状态遥测（只读本帧缓存；调用方需自行 telemetry.update()） */
    public void setTelemetry(Telemetry telemetry) {
        telemetry.addData("Ball count", "%d / %d", getBallCount(), FULL_COUNT);
        telemetry.addData("Ball full", isFull());
        telemetry.addData("Ball blocked", isBlocked() ? 1 : 0);
    }
}
