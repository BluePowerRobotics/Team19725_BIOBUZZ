package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.Shooter.PrototypeShooter.PrototypeShooter;
import org.firstinspires.ftc.teamcode.OpModes.Tuning.TuningLogger;
import org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelCalculator;

/**
 * 飞轮运转程序（第 4 代控制系统验证用）：Start 后飞轮<b>不转</b>，目标转速完全由按键给出。
 *
 * <p>面向「现场验证第 4 代控制律」而非整定：整定参数固化在
 * {@link org.firstinspires.ftc.teamcode.utility.Shooter.FlywheelParams} 的 {@code BIG} 参数集里
 * （Dashboard 分组 {@code FlywheelParams}），本程序只做目标转速管理、发起闭环绕组、并按需落 CSV。
 *
 * <p><b>操作</b>
 * <ul>
 *   <li>Start → 目标转速 {@code 0}（飞轮不转；这是旧版最大的差别：旧版 Start 会直接拉到待发转速）</li>
 *   <li>A → 目标转速置为 {@link #aVelocity}（默认 1600 tick/s）</li>
 *   <li>B → 目标转速置为 {@link #bVelocity}（默认 1900 tick/s）</li>
 *   <li>十字键上 / 下 → 按 {@link #velocityStep} 步长增减目标转速，按住则按 {@link #repeatPeriodMs} 连续调节</li>
 *   <li>X → 立即停转（目标转速置 0，闭环立刻收油门，飞轮惯性滑行）</li>
 * </ul>
 *
 * <p>目标转速限幅在 [0, {@link #maxVelocity}]。
 *
 * <p><b>CSV</b>：{@link #logToCsv} 打开时把每帧状态写
 * {@code <外部目录>/tuning/FlywheelRun_<时间戳>.csv}，表头为
 * {@code t,target,velocity,duty,voltage,error,vScale,inDrop,iActive,settled}，
 * 可直接喂给 {@code tools/tuning} 下的出图脚本（见 {@code tools/tuning/PLOT_GUIDE.md}）。
 *
 * <p>遥测为飞轮统一列表（5 行）：目标转速 / 当前转速 / power(占空比) / 电流 / 电压。
 */
@Config
@TeleOp(name = "FlywheelRun", group = "Tests")
public class FlywheelRun extends LinearOpMode {

    /** A 键目标转速 (tick/s) */
    public static int aVelocity = 1600;

    /** B 键目标转速 (tick/s) */
    public static int bVelocity = 1900;

    /** 方向键单次调节步长 (tick/s) */
    public static int velocityStep = 100;

    /** 目标转速上限 (tick/s) */
    public static int maxVelocity = 3000;

    /** 按住方向键时的连续调节周期 (毫秒) */
    public static long repeatPeriodMs = 100;

    /** 是否把运行数据落 CSV（默认关，避免日常跑车时写卡） */
    public static boolean logToCsv = false;

    /** CSV 采样周期 (毫秒)：与 tuning 日志的 ~100 Hz 对齐，避免每帧都写盘 */
    public static long logPeriodMs = 10;

    /** CSV 表头（列序与 {@link #logRow(double[])} 一致） */
    private static final String CSV_HEADER =
            "t,target,velocity,duty,voltage,error,vScale,inDrop,iActive,settled";

    private PrototypeShooter shooter;

    /** CSV 记录器（logToCsv 关闭时保持 null，不产生任何文件） */
    private TuningLogger logger;

    /** 上一次的方向键状态：-1 下 / 0 未按 / 1 上 */
    private int lastDir = 0;

    /** 连续调节计时器 */
    private final ElapsedTime repeatTimer = new ElapsedTime();

    /** CSV 采样计时器 */
    private final ElapsedTime logTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new PrototypeShooter(hardwareMap, telemetry);
        shooter.stop();

        telemetry.addLine("Start -> no spin (target = 0)");
        telemetry.addLine("A -> " + aVelocity + " tick/s    B -> " + bVelocity + " tick/s");
        telemetry.addLine("Dpad Up/Down: adjust target (hold to repeat)    X: stop");
        telemetry.addLine("Params: utility.Shooter.FlywheelParams.BIG (Dashboard group: FlywheelParams)");
        telemetry.addLine("CSV: " + (logToCsv ? "ON (tuning/FlywheelRun_*.csv)" : "OFF (logToCsv=false)"));
        shooter.setTelemetry();
        telemetry.update();

        waitForStart();

        // Start 后不转：目标转速为 0，等按键指令
        shooter.setTargetVelocity(0);

        if (logToCsv) {
            logger = new TuningLogger();
            logger.start(hardwareMap.appContext, "FlywheelRun", CSV_HEADER);
            logTimer.reset();
        }

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                shooter.stop(); // 立即停转
            }
            if (gamepad1.aWasPressed()) {
                shooter.setTargetVelocity(clampVelocity(aVelocity));
            }
            if (gamepad1.bWasPressed()) {
                shooter.setTargetVelocity(clampVelocity(bVelocity));
            }

            // 方向键上下调节目标转速：按下立即走一步，按住则按 repeatPeriodMs 连续调节
            int dir = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            if (dir != lastDir) {
                if (dir != 0) {
                    shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                }
                repeatTimer.reset();
            } else if (dir != 0 && repeatTimer.milliseconds() >= repeatPeriodMs) {
                shooter.setTargetVelocity(clampVelocity(shooter.getTargetVelocity() + dir * velocityStep));
                repeatTimer.reset();
            }
            lastDir = dir;

            shooter.update();
            shooter.setTelemetry();
            telemetry.update();

            if (logger != null && logTimer.milliseconds() >= logPeriodMs) {
                logTimer.reset();
                logRow();
            }
        }

        // 退出前收油门
        shooter.stop();
        shooter.update();
        if (logger != null) {
            logger.close();
            logger = null;
        }
    }

    /** 写一行 CSV：列序必须与 {@link #CSV_HEADER} 一致。 */
    private void logRow() {
        FlywheelCalculator calc = shooter.getCalculator();
        logger.logRowValues(
                getRuntime(),
                (double) shooter.getTargetVelocity(),
                shooter.getVelocity(),
                shooter.getDuty(),
                shooter.getBusVoltage(),
                shooter.getError(),
                calc.getVScale(),
                calc.isInDrop() ? 1.0 : 0.0,
                calc.isIntegralActive() ? 1.0 : 0.0,
                calc.isSettled() ? 1.0 : 0.0);
    }

    private static int clampVelocity(int velocity) {
        return Math.max(0, Math.min(maxVelocity, velocity));
    }
}
