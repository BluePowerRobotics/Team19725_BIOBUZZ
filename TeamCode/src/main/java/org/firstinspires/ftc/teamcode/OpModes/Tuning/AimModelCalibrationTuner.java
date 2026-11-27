package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import org.firstinspires.ftc.teamcode.utility.DpadStepper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.Algebra.LinearStatistics;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * 瞄准线性回归模型的自动标定台。
 *
 * <p>操作手手动驾驶底盘、手动控制集球器与炮台，按不同射程/仰角/飞轮转速逐发打门；
 * 每次扳机释放（上升沿）自动记录「位姿 + 射程 + 指令/实际转速 + 仰角 + 瞄准偏差 + 是否已稳定」，
 * 随后用主控按键把某发标记为「命中」。程序按仰角档位对命中样本做一元线性回归
 * {@code speed = slope·range + intercept}，并用相邻档实测射程推断各档的
 * {@code minRange/maxRange}，最终以 telemetry（FTC Dashboard Logging）输出建议值。
 *
 * <p>本程序<b>不</b>写回 {@code Shooter} 的静态参数：结果需人工抄回静态块。
 *
 * <p><b>主控（gamepad1）</b>
 * <ul>
 *   <li>左摇杆 → 底盘平移；右摇杆 X → 底盘旋转</li>
 *   <li>LB / RB → 集球器吃球 / 输出</li>
 *   <li>A / B → 把最近一发「大球 / 小球」标为命中（未标记的样本不参与回归）</li>
 *   <li>X → 重新做 {@link #CALIB_DURATION_MS} 毫秒视觉均值校准并重置定位
 *       （与 {@code FullTest} 相同；无有效视觉帧时保持原位姿，不退回 ResetPose）</li>
 *   <li>Y → 撤销最近一条样本（两个发射器按时间取最新）</li>
 * </ul>
 *
 * <p><b>副控（gamepad2）</b>
 * <ul>
 *   <li>方向键 上/下 → 大球目标转速 ±{@link #speedStep}；左/右 → 小球目标转速 ±{@link #speedStep}</li>
 *   <li>A（按住） → 同时释放大球与小球扳机（启动后需先松开一次 A 才允许发射）；
 *       每次上升沿两个发射器各记一发</li>
 * </ul>
 * 炮台平转角速度<b>恒为 0</b>（不接受手柄控制，平转角保持在上电/切模式时的位置）。
 * 送球联动与 {@code FullTest} 相同：扳机释放期间集球器强制 {@code setGiveBall()}，
 * 忽略一操的吃球/吐出；扳机复位后恢复 GP1 LB 吃球 &gt; RB 吐出 &gt; 均未按则停转。
 * 仰角舵机位置由 Dashboard 的 {@link #targetPitchBig} / {@link #targetPitchSmall} 直接设置
 * （与 {@code TurretManualTest} 一致，手柄不控制仰角）；标定时请把它们设成现有档位的
 * {@code pitchPos}，否则该发样本匹配不到档位、不参与回归。
 *
 * <p><b>启动流程</b>：{@code waitForStart()} 之后先执行一次与 {@code FullTest} 完全相同的
 * 强制视觉均值校准——按 A 触发，采集 {@link #CALIB_DURATION_MS} 毫秒的 MT1 有效位姿并
 * 用均值覆盖当前定位；采样窗口内没有有效帧时提示重按 A，不进入打点主循环。
 *
 * <p><b>现场流程</b>：Dashboard → Logging → Start → 按 A 做视觉校准 → 打点 → Stop → 导出 CSV；
 * 同时检查 Driver Station 上的 {@code >>>} 建议行，按格式抄回 {@code Shooter} 静态块。
 */
@Config
@TeleOp(name = "AimModelCalibration", group = "Tuning")
public class AimModelCalibrationTuner extends LinearOpMode {

    // ==================== Dashboard 可调参数 ====================

    /** 球门与定位 pipeline 使用的颜色；<b>改动后需重新 Init</b>（Chassis 在 init 阶段读取） */
    public static TeamColor teamColor = TeamColor.RED;

    /** 样本参与回归的最大瞄准偏差（度）；超过视为"没瞄准"，不参与回归 */
    public static double aimToleranceDeg = 2.0;
    /** true = 只接受发射瞬间飞轮已稳定（{@code Shooter.isReady()}）的样本 */
    public static boolean requireSettled = true;

    /** 大球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchBig = 0.5;
    /** 小球仰角舵机目标位置 [0,1]，由 Dashboard 直接设置 */
    public static double targetPitchSmall = 0.5;

    /** 视觉校准采样时长（毫秒），GP1.X 触发 */
    public static int CALIB_DURATION_MS = 2000;

    /** 俯仰舵机位置匹配到现有档位的容差（舵机单位） */
    public static double pitchMatchTol = 0.02;

    /** 每次方向键的转速增量 (tick/s) */
    public static int speedStep = 100;
    /** 长按方向键的连续步进周期（毫秒） */
    public static int repeatPeriodMs = 120;

    /** 拟合所需最小射程跨度（英寸） */
    public static double minSpanIn = 8.0;
    /** 首/末档边界向观测极值外扩的裕量（英寸） */
    public static double edgeMarginIn = 5.0;
    /** 每个发射器保留的最大样本数（超出丢最旧） */
    public static int maxShotsKept = 60;

    // ==================== 部件 ====================

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;

    // ==================== 运行时状态 ====================

    private int targetSpeedBig = 0;
    private int targetSpeedSmall = 0;

    private final DpadStepper vertical = new DpadStepper();
    private final DpadStepper horizontal = new DpadStepper();

    private final List<ShotSample> shotsBig = new ArrayList<>();
    private final List<ShotSample> shotsSmall = new ArrayList<>();

    private boolean lastTriggerBig = false;
    private boolean lastTriggerSmall = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    /** 扳机"已武装"：启动后必须先松开 A 再按下，才允许发射（原 GP2.X 平转模式切换已移除） */
    private boolean fireArmed = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionRunner actionRunner = new ActionRunner();
        // 初始位姿是视觉校准失败/未采到有效帧时的回退值：上电时请把车放在 ResetPose 位置
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, resetPose());
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        sweeper.setStop();
        sweeper.update();
        turret.stop();

        telemetry.addLine("AimModelCalibration: team=" + teamColor);
        telemetry.addLine("A (after start): forced 2 s vision calibration");
        telemetry.addLine("GP1: drive | LB/RB eat/output | A/B mark HIT big/small | X vision recalibrate | Y undo");
        telemetry.addLine("GP2: Dpad UD big speed | Dpad LR small speed | A trigger (forces feed)");
        telemetry.addLine("Turret yaw rate fixed at 0; pitch set on Dashboard (targetPitchBig/Small)");
        telemetry.update();

        waitForStart();

        // ==== 强制视觉均值校准（与 FullTest 启动逻辑相同）====
        Pose2d calibratedPose = calibrateViaVision();
        RobotPosition.getInstance().ResetPoseTo(calibratedPose);

        // 校准结束时若 A 仍被按住，需等其松开，避免主循环的 A 上升沿误标「命中」
        while (opModeIsActive() && gamepad1.a) {
            idle();
        }

        while (opModeIsActive()) {
            // ---- 定位（单帧推进点）----
            Pose2d pose = RobotPosition.getInstance().update();

            // ---- 底盘 ----
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // ---- 飞轮目标转速步进 ----
            int v = vertical.step(gamepad2.dpad_down, gamepad2.dpad_up, repeatPeriodMs);
            int h = horizontal.step(gamepad2.dpad_left, gamepad2.dpad_right, repeatPeriodMs);
            targetSpeedBig = clamp(targetSpeedBig + v * speedStep,
                    0, Shooter.BIG_PARAMS.flywheel.speedMax);
            targetSpeedSmall = clamp(targetSpeedSmall + h * speedStep,
                    0, Shooter.SMALL_PARAMS.flywheel.speedMax);

            // ---- 下发炮台目标：A 松开过一次才武装，按住 A 同时释放两个扳机（与 FullTest 一致）----
            if (!gamepad2.a) {
                fireArmed = true;
            }
            boolean triggerBig = gamepad2.a && fireArmed;
            boolean triggerSmall = triggerBig;
            turret.updateManual(0.0, targetPitchBig, targetPitchSmall,
                    targetSpeedBig, targetSpeedSmall, triggerBig, triggerSmall);

            // ---- 送球联动：扳机释放 → 强制供球，其余时间由 GP1 LB/RB 控制（与 FullTest 一致）----
            if (triggerBig) {
                sweeper.setGiveBall();
            } else if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else if (gamepad1.right_bumper) {
                sweeper.setOutput();
            } else {
                sweeper.setStop();
            }
            sweeper.update();

            // ---- 扳机上升沿：记一发 ----
            if (triggerBig && !lastTriggerBig) {
                recordShot(true, pose);
            }
            if (triggerSmall && !lastTriggerSmall) {
                recordShot(false, pose);
            }
            lastTriggerBig = triggerBig;
            lastTriggerSmall = triggerSmall;

            // ---- 命中标记 / 撤销 / 视觉重同步 ----
            if (gamepad1.a && !lastA) {
                markLastHit(shotsBig);
            }
            if (gamepad1.b && !lastB) {
                markLastHit(shotsSmall);
            }
            if (gamepad1.y && !lastY) {
                undoLatest();
            }
            if (gamepad1.x && !lastX) {
                // 与 FullTest 相同：采集 CALIB_DURATION_MS 的视觉均值覆盖当前定位；
                // 无有效视觉帧时保持原位姿（不退回 ResetPose）
                Pose2d calibrated = sampleVisionMean();
                if (calibrated != null) {
                    RobotPosition.getInstance().ResetPoseTo(calibrated);
                }
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
            lastX = gamepad1.x;

            // ---- 回归 + 边界推断（n 很小，每帧重算开销可忽略）----
            BandFit[] fitsBig = fitBands(shotsBig, Shooter.BIG_PARAMS);
            BandFit[] fitsSmall = fitBands(shotsSmall, Shooter.SMALL_PARAMS);

            // ---- 遥测 ----
            telemetry.addLine("--- AimModelCalibration (team " + teamColor + ") ---");
            emitStatus(pose);
            emitBandFits("Big", fitsBig);
            emitBandFits("Small", fitsSmall);
            emitLastShot("Big", shotsBig);
            emitLastShot("Small", shotsSmall);
            addRecentShotLines("Big", shotsBig);
            addRecentShotLines("Small", shotsSmall);
            addSuggestionLines("BIG", fitsBig);
            addSuggestionLines("SMALL", fitsSmall);
            telemetry.update();
        }

        // ---- 结束：再算一次最终建议（不再访问硬件）----
        chassis.stop();
        chassis.update(0, 0, 0);
        turret.stop();
        BandFit[] finalBig = fitBands(shotsBig, Shooter.BIG_PARAMS);
        BandFit[] finalSmall = fitBands(shotsSmall, Shooter.SMALL_PARAMS);
        telemetry.addLine("========== FINAL ==========");
        addSuggestionLines("BIG", finalBig);
        addSuggestionLines("SMALL", finalSmall);
        telemetry.update();
    }

    // ==================== 样本采集 ====================

    /** 单发样本；{@link #hit} 由操作手事后标记（未标记的样本不参与回归） */
    private static final class ShotSample {
        final long timeMs;
        final double range;
        final int targetSpeed;
        final double actualSpeed;
        final double pitch;
        final double aimErrorDeg;
        final boolean ready;
        final double busV;
        boolean hit = false;

        ShotSample(long timeMs, double range, int targetSpeed, double actualSpeed,
                   double pitch, double aimErrorDeg, boolean ready, double busV) {
            this.timeMs = timeMs;
            this.range = range;
            this.targetSpeed = targetSpeed;
            this.actualSpeed = actualSpeed;
            this.pitch = pitch;
            this.aimErrorDeg = aimErrorDeg;
            this.ready = ready;
            this.busV = busV;
        }
    }

    private void recordShot(boolean big, Pose2d pose) {
        Pose2d goal = goalPose();
        Vector2D shooterPos = shooterPosition(big, pose);
        double range = Vector2D.getDistance(
                shooterPos, new Vector2D(goal.position.x, goal.position.y));
        if (!(range > 0 && range <= 200)) {
            return;
        }

        Shooter shooter = big ? turret.shooterBig : turret.shooterSmall;
        double bearing = Math.atan2(goal.position.y - shooterPos.getY(),
                goal.position.x - shooterPos.getX());
        double aimErrorDeg = Math.toDegrees(Vector2D.normalizeAngle(
                bearing - (pose.heading.toDouble() + turret.yawController.getCurrentYaw())));

        ShotSample sample = new ShotSample(
                System.currentTimeMillis(),
                range,
                shooter.getTargetSpeed(),
                shooter.getVelocity(),
                shooter.getTargetPitch(),
                aimErrorDeg,
                shooter.isReady(),
                shooter.getBusVoltage());

        List<ShotSample> list = big ? shotsBig : shotsSmall;
        list.add(sample);
        while (list.size() > maxShotsKept) {
            list.remove(0);
        }
    }

    /** @return 发射器中心（车体系靠 {@code RelPos} 偏置，按航向旋到场地系） */
    private static Vector2D shooterPosition(boolean big, Pose2d pose) {
        Vector2D rel = big ? Turret.RelPosBig : Turret.RelPosSmall;
        Vector2D rotated = Vector2D.rotate(rel, pose.heading.toDouble());
        return new Vector2D(pose.position.x + rotated.getX(),
                pose.position.y + rotated.getY());
    }

    /** @return {@link RobotPosition} 由 HIVE 状态解出的当前球门（炮台 GOAL 模式使用同一来源） */
    private Pose2d goalPose() {
        return RobotPosition.getInstance().getGoalPose();
    }

    private Pose2d resetPose() {
        return teamColor == TeamColor.RED ? HypParams.ResetPoseRed : HypParams.ResetPoseBlue;
    }

    private static void markLastHit(List<ShotSample> shots) {
        if (shots.isEmpty()) {
            return;
        }
        shots.get(shots.size() - 1).hit = true;
    }

    private void undoLatest() {
        ShotSample lastBig = shotsBig.isEmpty() ? null : shotsBig.get(shotsBig.size() - 1);
        ShotSample lastSmall = shotsSmall.isEmpty() ? null : shotsSmall.get(shotsSmall.size() - 1);
        if (lastBig == null && lastSmall == null) {
            return;
        }
        if (lastSmall == null || (lastBig != null && lastBig.timeMs >= lastSmall.timeMs)) {
            shotsBig.remove(shotsBig.size() - 1);
        } else {
            shotsSmall.remove(shotsSmall.size() - 1);
        }
    }

    // ==================== 回归与边界推断 ====================

    /** 单个仰角档位的拟合结果与建议边界 */
    private static final class BandFit {
        final int index;
        final double pitchPos;
        final boolean valid;
        final int n;
        final double slope;
        final double intercept;
        final double r2;
        final double obsMin;
        final double obsMax;
        final double origMinRange;
        final double origMaxRange;
        final double origSlope;
        final double origIntercept;
        double minRange;
        double maxRange;
        String note;

        BandFit(int index, double pitchPos, boolean valid, int n, double slope, double intercept,
                double r2, double obsMin, double obsMax, double origMinRange, double origMaxRange,
                double origSlope, double origIntercept, String note) {
            this.index = index;
            this.pitchPos = pitchPos;
            this.valid = valid;
            this.n = n;
            this.slope = slope;
            this.intercept = intercept;
            this.r2 = r2;
            this.obsMin = obsMin;
            this.obsMax = obsMax;
            this.origMinRange = origMinRange;
            this.origMaxRange = origMaxRange;
            this.origSlope = origSlope;
            this.origIntercept = origIntercept;
            this.minRange = origMinRange;
            this.maxRange = origMaxRange;
            this.note = note;
        }

        double suggestedSlope() {
            return valid ? slope : origSlope;
        }

        double suggestedIntercept() {
            return valid ? intercept : origIntercept;
        }

        double suggestedMinRange() {
            return minRange;
        }

        double suggestedMaxRange() {
            return maxRange;
        }
    }

    private static BandFit[] fitBands(List<ShotSample> samples, Shooter.Params params) {
        Shooter.PitchModel[] models = params.pitchModels;
        if (models == null || models.length == 0) {
            return new BandFit[0];
        }

        BandFit[] fits = new BandFit[models.length];
        for (int i = 0; i < models.length; i++) {
            Shooter.PitchModel model = models[i];
            LinearStatistics stats = new LinearStatistics();
            int n = 0;
            double obsMin = Double.POSITIVE_INFINITY;
            double obsMax = Double.NEGATIVE_INFINITY;

            for (ShotSample s : samples) {
                if (!s.hit) {
                    continue;
                }
                if (requireSettled && !s.ready) {
                    continue;
                }
                if (Math.abs(s.aimErrorDeg) > aimToleranceDeg) {
                    continue;
                }
                if (Math.abs(s.pitch - model.pitchPos) > pitchMatchTol) {
                    continue;
                }
                stats.add(s.range, s.targetSpeed);
                n++;
                if (s.range < obsMin) {
                    obsMin = s.range;
                }
                if (s.range > obsMax) {
                    obsMax = s.range;
                }
            }

            double slope = stats.weight();
            double intercept = stats.bias();
            double r2 = stats.calculateR2();
            boolean valid = true;
            String note = "";
            if (n < 2) {
                valid = false;
                note = "样本不足(n<2)";
            } else if (obsMax - obsMin < minSpanIn) {
                valid = false;
                note = "射程跨度不足";
            } else if (slope <= 0) {
                valid = false;
                note = "斜率非正";
            } else if (slope * obsMax + intercept > params.flywheel.speedMax) {
                note = "末段将撞speedMax";
            }

            fits[i] = new BandFit(i, model.pitchPos, valid, n, slope, intercept, r2,
                    obsMin, obsMax, model.minRange, model.maxRange,
                    model.speedSlope, model.speedIntercept, note);
        }

        // ---- 相邻档边界：实测射程极值中点，仅当两档都 valid ----
        for (int i = 0; i + 1 < fits.length; i++) {
            if (fits[i].valid && fits[i + 1].valid) {
                double boundary = 0.5 * (fits[i].obsMax + fits[i + 1].obsMin);
                fits[i].maxRange = boundary;
                fits[i + 1].minRange = boundary;
            }
        }
        // ---- 首/末档外扩 ----
        if (fits[0].valid) {
            fits[0].minRange = Math.max(0.0, fits[0].obsMin - edgeMarginIn);
        }
        BandFit last = fits[fits.length - 1];
        if (last.valid) {
            last.maxRange = last.obsMax + edgeMarginIn;
        }
        // ---- 边界自检：数据矛盾时回退原值 ----
        for (BandFit f : fits) {
            if (!(f.minRange < f.maxRange)) {
                f.minRange = f.origMinRange;
                f.maxRange = f.origMaxRange;
                f.note = f.note.isEmpty() ? "边界矛盾，保留原值" : f.note + "；边界矛盾，保留原值";
            }
        }
        return fits;
    }

    // ==================== 遥测 ====================

    private void emitStatus(Pose2d pose) {
        telemetry.addData("AimCalPoseX", "%.1f", pose.position.x);
        telemetry.addData("AimCalPoseY", "%.1f", pose.position.y);
        telemetry.addData("AimCalPoseHeading", "%.1f", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("AimCalGoalX", "%.1f", goalPose().position.x);
        telemetry.addData("AimCalGoalY", "%.1f", goalPose().position.y);
        telemetry.addData("AimCalYawMode", turret.getManualYawMode());
        telemetry.addData("AimCalCmdYawDeg", "%.1f",
                Math.toDegrees(turret.yawController.getTargetYaw()));
        telemetry.addData("AimCalActYawDeg",
                "%.1f", Math.toDegrees(turret.yawController.getCurrentYaw()));
        telemetry.addData("AimCalRelBigX", "%.2f", Turret.RelPosBig.getX());
        telemetry.addData("AimCalRelBigY", "%.2f", Turret.RelPosBig.getY());
        telemetry.addData("AimCalRelSmallX", "%.2f", Turret.RelPosSmall.getX());
        telemetry.addData("AimCalRelSmallY", "%.2f", Turret.RelPosSmall.getY());
        telemetry.addData("AimCalShotsBig_total", shotsBig.size());
        telemetry.addData("AimCalShotsBig_hits", countHits(shotsBig));
        telemetry.addData("AimCalShotsSmall_total", shotsSmall.size());
        telemetry.addData("AimCalShotsSmall_hits", countHits(shotsSmall));
        telemetry.addData("AimCalTgtSpeedBig", targetSpeedBig);
        telemetry.addData("AimCalTgtSpeedSmall", targetSpeedSmall);
        telemetry.addData("AimCalPitchBig", "%.3f", targetPitchBig);
        telemetry.addData("AimCalPitchSmall", "%.3f", targetPitchSmall);
    }

    private void emitBandFits(String tag, BandFit[] fits) {
        for (BandFit f : fits) {
            String key = "AimCal" + tag + "B" + f.index + "_";
            telemetry.addData(key + "n", f.n);
            telemetry.addData(key + "valid", f.valid ? 1 : 0);
            telemetry.addData(key + "slope", "%.2f", f.suggestedSlope());
            telemetry.addData(key + "intercept", "%.1f", f.suggestedIntercept());
            telemetry.addData(key + "r2", "%.3f", f.r2);
            telemetry.addData(key + "obsMin", f.n > 0 ? String.format(Locale.US, "%.1f", f.obsMin) : "NaN");
            telemetry.addData(key + "obsMax", f.n > 0 ? String.format(Locale.US, "%.1f", f.obsMax) : "NaN");
            telemetry.addData(key + "minRange", "%.1f", f.suggestedMinRange());
            telemetry.addData(key + "maxRange", "%.1f", f.suggestedMaxRange());
        }
    }

    private void emitLastShot(String tag, List<ShotSample> shots) {
        if (shots.isEmpty()) {
            return;
        }
        ShotSample s = shots.get(shots.size() - 1);
        String key = "AimCalLast" + tag + "_";
        telemetry.addData(key + "range", "%.1f", s.range);
        telemetry.addData(key + "targetSpeed", s.targetSpeed);
        telemetry.addData(key + "actualSpeed", "%.0f", s.actualSpeed);
        telemetry.addData(key + "pitch", "%.3f", s.pitch);
        telemetry.addData(key + "aimErr", "%.1f", s.aimErrorDeg);
        telemetry.addData(key + "ready", s.ready ? 1 : 0);
        telemetry.addData(key + "hit", s.hit ? 1 : 0);
    }

    private void addRecentShotLines(String tag, List<ShotSample> shots) {
        telemetry.addLine("--- " + tag + " recent ---");
        for (int i = Math.max(0, shots.size() - 4); i < shots.size(); i++) {
            ShotSample s = shots.get(i);
            telemetry.addLine(String.format(Locale.US,
                    "%s r=%.1f v=%d/%.0f p=%.3f err=%.1f %s",
                    s.hit ? "HIT " : "----",
                    s.range, s.targetSpeed, s.actualSpeed, s.pitch, s.aimErrorDeg,
                    s.ready ? "RDY" : "   "));
        }
    }

    private void addSuggestionLines(String tag, BandFit[] fits) {
        if (fits.length == 0) {
            telemetry.addLine(">>> " + tag + ": pitchModels 为空，无可标定档位");
            return;
        }
        for (BandFit f : fits) {
            String note = f.note.isEmpty() ? "" : "  (" + f.note + ")";
            telemetry.addLine(String.format(Locale.US,
                    ">>> %s_MODEL[%d] new PitchModel(%.3f, %.1f, %.1f, %.2f, %.1f)  n=%d R2=%.2f%s",
                    tag, f.index, f.pitchPos,
                    f.suggestedMinRange(), f.suggestedMaxRange(),
                    f.suggestedSlope(), f.suggestedIntercept(),
                    f.n, f.r2, note));
        }
    }

    private static int countHits(List<ShotSample> shots) {
        int n = 0;
        for (ShotSample s : shots) {
            if (s.hit) {
                n++;
            }
        }
        return n;
    }

    // ==================== 视觉校准（与 FullTest 相同） ====================

    /**
     * 启动阶段的强制视觉均值校准：等待按 A 后采集 {@link #CALIB_DURATION_MS} 毫秒内的 MT1
     * 均值位姿（位置算术平均，朝向圆周平均）。
     *
     * <p>采样窗口内没有任何有效帧时提示重按 A，不会退回 {@link #resetPose()}。
     * 校准期间底盘保持静止，但仍每帧调用 {@link RobotPosition#update()}，
     * 由融合定位器内部推进 Pinpoint 并拉取最新 MT1 结果。
     */
    private Pose2d calibrateViaVision() {
        MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();
        List<Pose2d> samples = new ArrayList<>();
        boolean prevA = false;

        while (opModeIsActive()) {
            chassis.stop();
            chassis.update(0, 0, 0);
            RobotPosition.getInstance().update();

            boolean usable = mt1.isValid() && mt1.isHiveEstimated();
            addCalibrationTelemetry(mt1, usable ? mt1.getPose() : null, samples, -1);

            boolean a = gamepad1.a;
            if (a && !prevA) {
                Pose2d mean = sampleVisionMean();
                if (mean != null) {
                    telemetry.addData("Calibrated start pose", formatPose(mean));
                    telemetry.update();
                    return mean;
                }
                telemetry.addLine("No valid samples, press A to retry");
            }
            prevA = a;
            telemetry.update();
        }
        // OpMode 被停止时才会执行到这里
        return resetPose();
    }

    /**
     * 立即采集 {@link #CALIB_DURATION_MS} 毫秒的视觉均值位姿（不等待按键）。
     *
     * <p>采样期间底盘保持静止，但仍每帧调用 {@link RobotPosition#update()}，由融合定位器
     * 内部推进 Pinpoint 并拉取最新 MT1 结果。
     *
     * @return 位置算术平均 / 朝向圆周平均的位姿；采样窗口内无任何 HIVE 有效帧时返回
     *         {@code null}（调用方应保持原位姿）
     */
    private Pose2d sampleVisionMean() {
        MT1Localizer mt1 = RobotPosition.getInstance().getFusionLocalizer().getMT1();
        List<Pose2d> samples = new ArrayList<>();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            // 采样期间底盘必须静止：持续下发零速并推进里程计
            chassis.stop();
            chassis.update(0, 0, 0);
            RobotPosition.getInstance().update();

            long elapsed = System.currentTimeMillis() - startMs;
            boolean usable = mt1.isValid() && mt1.isHiveEstimated();
            if (usable && elapsed < CALIB_DURATION_MS) {
                samples.add(mt1.getPose());
            }
            addCalibrationTelemetry(mt1, usable ? mt1.getPose() : null, samples, elapsed);

            if (elapsed >= CALIB_DURATION_MS) {
                break;
            }
            telemetry.update();
        }
        return samples.isEmpty() ? null : computeMeanPose(samples);
    }

    /**
     * 校准界面遥测。
     *
     * @param usablePose 本帧可用的 MT1 位姿，不可用传 {@code null}
     * @param elapsedMs  采样已进行的毫秒数；{@code < 0} 表示尚未开始采样
     */
    private void addCalibrationTelemetry(MT1Localizer mt1, Pose2d usablePose,
                                         List<Pose2d> samples, long elapsedMs) {
        telemetry.addLine("=== Vision Calibration (MT1) ===");
        if (usablePose != null) {
            telemetry.addData("Live pose", formatPose(usablePose));
        } else {
            telemetry.addLine("No valid pose (need valid + hiveEstimated)");
        }
        telemetry.addData("Tag Count", mt1.getTagCount());
        telemetry.addData("Tag Ids", mt1.getTagIds());
        telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());
        telemetry.addData("Hive State", mt1.getHiveState());
        telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
        telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());
        if (elapsedMs < 0) {
            telemetry.addLine("Press A to start 2 s sampling");
        } else {
            telemetry.addData("Recording", "Active");
            telemetry.addData("Samples", samples.size());
            telemetry.addData("Time left (ms)", Math.max(0, CALIB_DURATION_MS - elapsedMs));
        }
    }

    /** 位置算术平均，朝向圆周平均。 */
    private static Pose2d computeMeanPose(List<Pose2d> samples) {
        double sumX = 0, sumY = 0;
        double sumSin = 0, sumCos = 0;
        for (Pose2d p : samples) {
            sumX += p.position.x;
            sumY += p.position.y;
            sumSin += Math.sin(p.heading.toDouble());
            sumCos += Math.cos(p.heading.toDouble());
        }
        return new Pose2d(sumX / samples.size(), sumY / samples.size(), Math.atan2(sumSin, sumCos));
    }

    private static String formatPose(Pose2d pose) {
        return String.format(Locale.US, "(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }

    // ==================== 工具 ====================

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** 方向键步进器见 {@link DpadStepper} */
}
