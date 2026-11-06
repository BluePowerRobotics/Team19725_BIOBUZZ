package org.firstinspires.ftc.teamcode.架构练习;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

/**
 * 集球器（架构练习版）—— 对应架构图中的「Sweeper」节点。
 *
 * <p>组合原 {@link Sweeper}（复用其固件速度闭环 + 铲子舵机），并在其上增加架构图的跨层同步逻辑：
 * <b>满仓停吸</b>——每帧下发速度前，从 {@link RobotPosition1} 读 {@code BallCount}，
 * 满仓且意图为吸球（正速度）时强制归零，出球方向（负速度）不受限制。
 *
 * <p><b>数据同步方向</b>：Sweeper1 →（只读）→ RobotPosition1.getBallCount()/isFull()。
 *
 * <p><b>注意</b>：原 Sweeper 的 {@code update()} 每帧会把 targetVelocity 归零，因此上层必须每帧
 * 重新调用 {@link #setEat()} / {@link #setOutput()} / {@link #setStop()} 设置意图。
 */
public class Sweeper1 {

    /** 被组合的原集球器控制器 */
    private final Sweeper sweeper;

    /** 只读依赖：状态估计器（取球道满仓状态） */
    private final RobotPosition1 robotPosition;

    /** 本帧意图速度（满仓保护在 {@link #update()} 里施加） */
    private int targetVelocity = 0;

    /**
     * @param hardwareMap   硬件映射
     * @param telemetry     遥测（传给原 Sweeper）
     * @param robotPosition 只读依赖的状态估计器
     */
    public Sweeper1(HardwareMap hardwareMap, Telemetry telemetry, RobotPosition1 robotPosition) {
        // Init 顺序：先下属（原 Sweeper），再记录依赖
        this.sweeper = new Sweeper(hardwareMap, telemetry);
        this.robotPosition = robotPosition;
    }

    /** 吸球意图（intake，正速度，计数方向 +1） */
    public void setEat() {
        targetVelocity = Sweeper.EatVel;
    }

    /** 倒球意图（output，负速度，计数方向 -1） */
    public void setOutput() {
        targetVelocity = Sweeper.OutputVel;
    }

    /** 停止意图 */
    public void setStop() {
        targetVelocity = 0;
    }

    /**
     * 每帧调用：先施加满仓停吸保护，再把目标速度下发给原 Sweeper。
     */
    public void update() {
        // 满仓停吸：满仓时强制停吸（出球/负速度方向不受限制）
        if (targetVelocity > 0 && robotPosition.isFull()) {
            targetVelocity = 0;
        }
        sweeper.setTargetVelocity(targetVelocity);
        sweeper.update();
        targetVelocity = 0; // 每帧必须重新调用 set 函数，否则自动归零
    }

    /** @return 本帧实际下发的目标速度（满仓保护后），供遥测 */
    public int getTargetVelocity() {
        return sweeper.getTargetVelocity();
    }

    /** 停止集球器（退出时调用） */
    public void stop() {
        sweeper.setStop();
        sweeper.update();
    }
}
