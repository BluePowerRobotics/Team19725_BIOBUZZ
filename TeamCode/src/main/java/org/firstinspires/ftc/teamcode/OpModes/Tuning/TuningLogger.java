package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import android.content.Context;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * 标定数据记录器：把采样写入 Robot Controller 外部存储的 CSV，便于离线复核与拟合
 * （对应研究报告第 6 章各方法给出的 Python 脚本）。
 *
 * <p>文件路径：{@code <RC 外部存储>/tuning/<name>_<时间戳>.csv}，可在
 * Robot Controller 应用的 "Files" 或用 USB/`adb pull` 取出。
 *
 * <p>文件格式：以 {@code #} 开头的注释行（含 OpMode 写入的参数说明），随后是表头
 * {@code t,duty,velocity} 与数据行，列序固定为「时间(s), 占空比, 转速(tick/s)」。
 *
 * <p>用法：
 * <pre>
 *   logger.start(hardwareMap.appContext, "step01");
 *   logger.logRow(t, duty, velocity);   // 每采样一次调用
 *   logger.log("# Kd=..., tau=...");    // 追加自定义注释行
 *   logger.close();                     // OpMode 结束前务必调用
 * </pre>
 */
public class TuningLogger {

    /** 每写入多少行 flush 一次（频繁 flush 会拖慢控制循环） */
    private static final int FLUSH_EVERY = 20;

    private BufferedWriter writer;
    private int sinceFlush = 0;
    private String path = "(未写入)";

    /**
     * 创建并打开一个 CSV 文件（默认三列 {@code t,duty,velocity}）。
     *
     * @param context Robot Controller 上下文（{@code hardwareMap.appContext}）
     * @param name    文件名前缀（不含扩展名）
     */
    public void start(Context context, String name) {
        start(context, name, "t,duty,velocity");
    }

    /**
     * 创建并打开一个 CSV 文件，表头可自定义（第 4 代新增：滤波器整定需要额外的电压/电流列）。
     *
     * @param context Robot Controller 上下文（{@code hardwareMap.appContext}）
     * @param name    文件名前缀（不含扩展名）
     * @param header  表头行，列名以逗号分隔，例如 {@code "t,velocity,duty,vbus,current"}
     */
    public void start(Context context, String name, String header) {
        close();
        try {
            File dir = new File(context.getExternalFilesDir(null), "tuning");
            if (!dir.exists() && !dir.mkdirs()) {
                path = "(目录创建失败: " + dir.getAbsolutePath() + ")";
                return;
            }
            File file = new File(dir, name + "_" + System.currentTimeMillis() + ".csv");
            writer = new BufferedWriter(new FileWriter(file));
            path = file.getAbsolutePath();
            sinceFlush = 0;
            // 统一表头：注释行 + 固定列序，便于离线用 numpy/pandas 直接读取画图
            // （以 '#' 开头的行是注释，解析时跳过；第一行非注释即为列名）
            log("# flywheel tuning data (" + name + ")");
            log("# " + header);
            log(header);
        } catch (IOException e) {
            writer = null;
            path = "(打开失败: " + e.getMessage() + ")";
        }
    }

    /** @param line 完整的一行内容（不含换行） */
    public void log(String line) {
        if (writer == null) return;
        try {
            writer.write(line);
            writer.write("\n");
            if (++sinceFlush >= FLUSH_EVERY) {
                writer.flush();
                sinceFlush = 0;
            }
        } catch (IOException ignored) {
            // 记录失败不应中断控制循环
        }
    }

    /** 记录一行 (时间, 占空比, 转速) 采样 */
    public void logRow(double t, double duty, double velocity) {
        log(String.format(Locale.US, "%.4f,%.4f,%.2f", t, duty, velocity));
    }

    /** 记录一行任意列数的采样（每列统一 {@code %.4f}，列数需与 {@link #start(Context, String, String)} 的表头一致） */
    public void logRowValues(double... values) {
        StringBuilder sb = new StringBuilder(values.length * 10);
        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(String.format(Locale.US, "%.4f", values[i]));
        }
        log(sb.toString());
    }

    /** @return 当前 CSV 文件路径（供遥测显示） */
    public String getPath() {
        return path;
    }

    /** 刷盘并关闭文件。 */
    public void close() {
        if (writer == null) return;
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
        }
        writer = null;
    }
}
