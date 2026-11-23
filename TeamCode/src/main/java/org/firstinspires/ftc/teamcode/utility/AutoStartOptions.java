package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * 自动阶段的开局选项。
 *
 * <p><b>开局是否发射</b>：在 {@code waitForStart()} 之前按 <b>gamepad1 的 X 键</b>切换，
 * 默认 {@link #DEFAULT_START_SHOOT}（{@code true}，即自动一开始就先发射预载）。
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

    /** 本局是否在自动一开始就发射 */
    private static boolean startShoot = DEFAULT_START_SHOOT;

    /** 上一次 X 键状态（用于按下沿检测，避免按住时连续翻转） */
    private static boolean lastX = false;

    private AutoStartOptions() {
    }

    /** 每局 init 阶段调用，恢复默认值。 */
    public static void reset() {
        startShoot = DEFAULT_START_SHOOT;
        lastX = false;
    }

    /** 在 {@code waitForStart()} 之前每帧调用：检测 gamepad1.X 的按下沿并切换开关。 */
    public static void update(Gamepad gamepad) {
        boolean x = gamepad.x;
        if (x && !lastX) {
            startShoot = !startShoot;
        }
        lastX = x;
    }

    /** @return 本局是否在自动一开始就发射 */
    public static boolean startShoot() {
        return startShoot;
    }
}
