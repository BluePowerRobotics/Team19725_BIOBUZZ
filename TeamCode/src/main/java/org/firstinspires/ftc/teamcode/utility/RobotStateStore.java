package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.Properties;

/**
 * 机器状态持久化存储：把跨 OpMode 传递的状态写入一个文件，断电后仍然有效。
 *
 * <p><b>用途</b>：自动阶段结束时记录机器位姿等状态，手动阶段初始化时读回，
 * 使两个阶段之间无需人工重新摆放/重置定位。
 *
 * <p><b>生命周期约定</b>
 * <ul>
 *   <li>自动阶段开始（{@code waitForStart()} 之后）调用 {@link #clear()} 清空上一局遗留状态；</li>
 *   <li>自动阶段结束调用 {@link #savePose(Pose2d)}（或 {@link #put(String, String)}）写入状态；</li>
 *   <li>手动阶段初始化时调用 {@link #loadPose()} / {@link #get(String)} 读回。</li>
 * </ul>
 *
 * <p><b>存储位置</b>：{@link AppUtil#getSettingsFile(String)} 返回 FTC App 私有目录下的文件，
 * 不随 OpMode 结束、App 退出或机器人断电而丢失。写入时调用 {@code FileDescriptor.sync()}
 * 强制落盘，避免写入后立刻断电导致数据还在页缓存中丢失。
 *
 * <p><b>文件格式</b>：{@link Properties} 的 {@code key=value} 文本，缺失的键读取时返回 null。
 * 新增字段（如炮台状态）只需用 {@link #put(String, String)} 追加，不影响已有字段。
 */
public final class RobotStateStore {

    private static final String TAG = "RobotStateStore";

    /** 状态文件名（单一文件，所有状态共用） */
    private static final String FILE_NAME = "robot_state.txt";

    // ---- 键名 ----
    private static final String KEY_POSE_X = "pose.x";
    private static final String KEY_POSE_Y = "pose.y";
    private static final String KEY_POSE_HEADING = "pose.heading";

    private RobotStateStore() {
    }

    // ==================== 对外接口 ====================

    /**
     * 清空状态文件，删除全部已记录的状态。
     * 自动阶段开始时调用，确保不会读到上一局的残留数据。
     */
    public static void clear() {
        File file = stateFile();
        if (file.exists() && !file.delete()) {
            RobotLog.ee(TAG, "清空状态文件失败: %s", file.getAbsolutePath());
        }
    }

    /**
     * 写入机器位姿（自动阶段结束时调用）。
     *
     * @param pose 结束时刻的位姿估计
     */
    public static void savePose(Pose2d pose) {
        Properties state = readAll();
        state.setProperty(KEY_POSE_X, Double.toString(pose.position.x));
        state.setProperty(KEY_POSE_Y, Double.toString(pose.position.y));
        state.setProperty(KEY_POSE_HEADING, Double.toString(pose.heading.toDouble()));
        writeAll(state);
    }

    /**
     * 读回机器位姿。
     *
     * @return 上次记录的位姿；文件不存在或记录不完整/损坏时返回 null
     */
    public static Pose2d loadPose() {
        Properties state = readAll();
        String x = state.getProperty(KEY_POSE_X);
        String y = state.getProperty(KEY_POSE_Y);
        String heading = state.getProperty(KEY_POSE_HEADING);
        if (x == null || y == null || heading == null) {
            return null;
        }
        try {
            return new Pose2d(Double.parseDouble(x), Double.parseDouble(y), Double.parseDouble(heading));
        } catch (NumberFormatException e) {
            RobotLog.ee(TAG, e, "状态文件中的位姿格式损坏");
            return null;
        }
    }

    /**
     * 追加一个自定义状态字段（预留扩展，如炮台状态），不会影响文件中已有的其他字段。
     * 需要在自动阶段结束时与位姿一起保存的字段，都在结束前调用本方法写入即可。
     *
     * @param key   键名
     * @param value 键值（字符串形式）
     */
    public static void put(String key, String value) {
        Properties state = readAll();
        state.setProperty(key, value);
        writeAll(state);
    }

    /**
     * 读取一个自定义状态字段。
     *
     * @param key 键名
     * @return 对应的值；不存在时返回 null
     */
    public static String get(String key) {
        return readAll().getProperty(key);
    }

    // ==================== 内部实现 ====================

    /** @return FTC App 私有目录下的状态文件（目录不存在时会自动创建） */
    private static File stateFile() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    /** 读取文件中的全部键值；文件不存在或读取失败时返回空集合 */
    private static Properties readAll() {
        Properties state = new Properties();
        File file = stateFile();
        if (!file.exists()) {
            return state;
        }
        try (Reader reader = new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8)) {
            state.load(reader);
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "读取状态文件失败: %s", file.getAbsolutePath());
        }
        return state;
    }

    /** 覆写整个文件；写入失败时记录日志（不抛出，避免中断 OpMode 收尾流程） */
    private static void writeAll(Properties state) {
        File file = stateFile();
        try (FileOutputStream stream = new FileOutputStream(file);
             Writer writer = new OutputStreamWriter(stream, StandardCharsets.UTF_8)) {
            state.store(writer, "robot state");
            writer.flush();
            // 强制写入闪存：机器人在写入后立刻断电时，数据也不停留在页缓存中
            stream.getFD().sync();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "写入状态文件失败: %s", file.getAbsolutePath());
        }
    }
}