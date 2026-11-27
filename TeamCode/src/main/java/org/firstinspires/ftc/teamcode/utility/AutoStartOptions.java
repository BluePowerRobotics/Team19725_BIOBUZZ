package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 自动阶段的开局选项。
 *
 * <p><b>开局是否发射</b>：在 {@code waitForStart()} 之前按 <b>gamepad1 的 X 键</b>切换，
 * 默认 {@link #DEFAULT_START_SHOOT}（{@code true}，即自动一开始就先发射预载）。
 *
 * <p><b>第二次循环去第一发射区域</b>：在 {@code waitForStart()} 之前按 <b>gamepad1 的 Y 键</b>切换，
 * 默认 {@link #DEFAULT_SECOND_CYCLE_FIRST_AREA}（{@code false}）。开启后，第二次及以后的
 * 「前往发射点」动作改为前往<b>第一发射区域</b>并自动瞄准发射。
 *
 * <p><b>循环数驱动发射区域切换模式</b>：在 {@code waitForStart()} 之前按 <b>gamepad1 的 A 键</b>
 * 循环切换，默认 {@link #AREA_CYCLE_MODE_OFF}（关闭，所有循环默认去<b>第二发射区域</b>）。
 * 枚举值含义见 {@link #AREA_CYCLE_MODE_OFF} / {@link #AREA_CYCLE_MODE_EVEN} /
 * {@link #AREA_CYCLE_MODE_ODD} / {@link #AREA_CYCLE_MODE_EVERY}。
 *
 * <p><b>优先级</b>：AREA_CYCLE_MODE 非 OFF 时按其逻辑；AREA_CYCLE_MODE==OFF 且
 * secondCycleFirstArea=true 时按上一轮 SECOND_CYCLE_FIRST_AREA 逻辑；两者都关时
 * 默认去第二发射区域。
 *
 * <p><b>用法</b>（各自动 OpMode 统一写法）：
 * <pre>
 *   AutoStartOptions.reset();                  // 恢复默认值，避免上一局选择残留
 *   while (opModeInInit()) {                   // waitForStart 之前的等待窗口
 *       AutoStartOptions.update(gamepad1);     // 按下沿切换
 *       telemetry.addData("Start Shoot (gamepad1.X)", AutoStartOptions.startShoot());
 *       telemetry.update();
 *   }
 *   waitForStart();
 *   final boolean startShoot = AutoStartOptions.startShoot();
 * </pre>
 */
public final class AutoStartOptions {

    /** 默认值：自动一开始就发射预载 */
    public static final boolean DEFAULT_START_SHOOT = true;

    /** 默认值：第二次循环仍按原逻辑选发射区域（不改去第一发射区域） */
    public static final boolean DEFAULT_SECOND_CYCLE_FIRST_AREA = false;

    /** 默认值：AREA_CYCLE_MODE 关闭 */
    public static final int DEFAULT_AREA_CYCLE_MODE = 0;

    /** AREA_CYCLE_MODE：关闭（所有循环默认去<b>第二发射区域</b>） */
    public static final int AREA_CYCLE_MODE_OFF = 0;
    /** AREA_CYCLE_MODE：偶数次循环末尾切到第一发射区域（奇数次保持第二区域） */
    public static final int AREA_CYCLE_MODE_EVEN = 1;
    /** AREA_CYCLE_MODE：奇数次循环末尾切到第一发射区域（偶数次保持第二区域） */
    public static final int AREA_CYCLE_MODE_ODD = 2;
    /** AREA_CYCLE_MODE：每次循环末尾都切（交替：第一次 SECOND → 第二次 FIRST → 第三次 SECOND...） */
    public static final int AREA_CYCLE_MODE_EVERY = 3;

    /** AREA_CYCLE_MODE 枚举总数（用于 A 键循环切值） */
    public static final int AREA_CYCLE_MODE_COUNT = 4;

    /** 本局是否在自动一开始就发射 */
    private static boolean startShoot = DEFAULT_START_SHOOT;

    /** 本局第二次循环是否改去第一发射区域 */
    private static boolean secondCycleFirstArea = DEFAULT_SECOND_CYCLE_FIRST_AREA;

    /** 本局循环数驱动发射区域切换模式（枚举值 0~3） */
    private static int areaCycleMode = DEFAULT_AREA_CYCLE_MODE;

    /** 上一次 X 键状态（用于按下沿检测，避免按住时连续翻转） */
    private static boolean lastX = false;

    /** 上一次 Y 键状态（用于按下沿检测） */
    private static boolean lastY = false;

    /** 上一次 A 键状态（用于按下沿检测） */
    private static boolean lastA = false;

    private AutoStartOptions() {
    }

    /** 每局 init 阶段调用，恢复默认值。 */
    public static void reset() {
        startShoot = DEFAULT_START_SHOOT;
        secondCycleFirstArea = DEFAULT_SECOND_CYCLE_FIRST_AREA;
        areaCycleMode = DEFAULT_AREA_CYCLE_MODE;
        lastX = false;
        lastY = false;
        lastA = false;
    }

    /** 在 {@code waitForStart()} 之前每帧调用：检测 gamepad1.X / Y / A 的按下沿并切换对应开关。 */
    public static void update(Gamepad gamepad) {
        boolean x = gamepad.x;
        if (x && !lastX) {
            startShoot = !startShoot;
        }
        lastX = x;

        boolean y = gamepad.y;
        if (y && !lastY) {
            secondCycleFirstArea = !secondCycleFirstArea;
        }
        lastY = y;

        boolean a = gamepad.a;
        if (a && !lastA) {
            areaCycleMode = (areaCycleMode + 1) % AREA_CYCLE_MODE_COUNT;
        }
        lastA = a;
    }

    /** @return 本局是否在自动一开始就发射 */
    public static boolean startShoot() {
        return startShoot;
    }

    /** @return 本局第二次及以后循环是否改去第一发射区域瞄准发射 */
    public static boolean secondCycleFirstArea() {
        return secondCycleFirstArea;
    }

    /** @return 本局循环数驱动发射区域切换模式（枚举值 0~3，见 {@link #AREA_CYCLE_MODE_*}） */
    public static int getAreaCycleMode() {
        return areaCycleMode;
    }

    /** @return AREA_CYCLE_MODE 的人类可读名称，用于遥测 */
    public static String areaCycleModeName(int mode) {
        switch (mode) {
            case AREA_CYCLE_MODE_EVEN:  return "EVEN";
            case AREA_CYCLE_MODE_ODD:   return "ODD";
            case AREA_CYCLE_MODE_EVERY: return "EVERY";
            default:                    return "OFF";
        }
    }
}
