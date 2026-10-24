package org.firstinspires.ftc.teamcode.Auto.AutomaticCalibration;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * 标定结果导出器：把「原始采样数据」与「静态字段建议值」写成 CSV，并把拟合曲线 /
 * 残差 / 阶跃响应画成 SVG 图片落盘，供操作手确认后手动填回参数文件。
 *
 * <p><b>不写回 {@code MecanumDrive.PARAMS}</b>：本导出器只落盘文件与返回建议值，
 * 绝不修改任何静态字段。是否采用建议值由操作手决定。
 *
 * <p>输出目录：{@code /sdcard/AutomaticCalibration/<yyyyMMdd_HHmmss>/}，包含：
 * <ul>
 *   <li>{@code samples.csv}：逐帧原始采样（列见 {@link #SAMPLES_HEADER}）；</li>
 *   <li>{@code results.csv}：静态字段名 / 量纲 / 当前值 / 建议值 / 残差；</li>
 *   <li>{@code fit_sva.svg}：有效电压对轮速散点 + 前馈拟合直线；</li>
 *   <li>{@code residual_ka.svg}：固定 kS/kV 后的残差对加速度散点 + kA 拟合直线；</li>
 *   <li>{@code fit_feedback.svg}：闭环阶跃响应 + 目标线 + 峰值标记。</li>
 * </ul>
 * SVG 为纯文本矢量图，可直接用浏览器打开、截图或转成 PNG，满足「图像可导出」。
 *
 * <p>所有写盘异常都被捕获并记入 {@code RobotLog}，不向上抛出，避免标定程序因
 * 存储异常中断。
 */
public final class CalibrationExporter {

    /** samples.csv 表头，列顺序与 {@link CalibrationSample#toCsvRow()} 严格一致。 */
    public static final String SAMPLES_HEADER =
            "phase,t,battV,power,voltsEff,wheelVelTicks,wheelAccelTicks,poseVx,poseVy,poseOmega";
    /** results.csv 表头。 */
    public static final String RESULTS_HEADER = "field,unit,current,suggested,residual";

    private static final String TAG = "CalibrationExporter";
    private static final String ROOT = "AutomaticCalibration";

    private CalibrationExporter() {
    }

    /** results.csv 中的一行：某个静态字段的当前值与建议值。 */
    public static final class ResultEntry {
        public final String field;
        public final String unit;
        public final double current;
        public final double suggested;
        public final double residual;

        public ResultEntry(String field, String unit, double current, double suggested, double residual) {
            this.field = field;
            this.unit = unit;
            this.current = current;
            this.suggested = suggested;
            this.residual = residual;
        }

        String toCsvRow() {
            return String.format(Locale.US, "%s,%s,%.8g,%.8g,%.8g",
                    field, unit, current, suggested, residual);
        }
    }

    /**
     * 建立本次标定的输出目录 {@code /sdcard/AutomaticCalibration/<时间戳>/}。
     *
     * @return 输出目录；创建失败时返回 {@code null}
     */
    public static File newSessionDir() {
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File dir = new File(new File(Environment.getExternalStorageDirectory(), ROOT), stamp);
        if (!dir.exists() && !dir.mkdirs()) {
            RobotLog.ee(TAG, "无法创建标定输出目录: " + dir.getAbsolutePath());
            return null;
        }
        return dir;
    }

    /** 写出逐帧原始采样 CSV。 */
    public static void writeSamples(File dir, List<CalibrationSample> samples) {
        if (dir == null) {
            return;
        }
        StringBuilder sb = new StringBuilder(samples.size() * 64);
        sb.append(SAMPLES_HEADER).append('\n');
        for (CalibrationSample s : samples) {
            sb.append(s.toCsvRow()).append('\n');
        }
        writeFile(new File(dir, "samples.csv"), sb.toString());
    }

    /** 写出静态字段建议值 CSV。 */
    public static void writeResults(File dir, List<ResultEntry> results) {
        if (dir == null) {
            return;
        }
        StringBuilder sb = new StringBuilder(results.size() * 48);
        sb.append(RESULTS_HEADER).append('\n');
        for (ResultEntry r : results) {
            sb.append(r.toCsvRow()).append('\n');
        }
        writeFile(new File(dir, "results.csv"), sb.toString());
    }

    /**
     * 画「有效电压 − 轮速」散点与前馈拟合直线（{@code V = kS·sign(v) + kV·v}）。
     *
     * @param v      轮速序列，tick/s（横轴）
     * @param volts  有效电压序列，V（纵轴）
     * @param kS     拟合静态补偿，V
     * @param kV     拟合速度增益，V/(tick/s)
     * @param title  图表标题
     */
    public static void writeSvaChart(File dir, double[] v, double[] volts, double kS, double kV,
                                     String title) {
        if (dir == null || v.length == 0) {
            return;
        }
        SvgChart chart = new SvgChart(720, 480, title, "wheel vel (tick/s)", "effective volts (V)");
        chart.autoRange(v, volts);
        chart.addPoints(v, volts, "#1565C0", 2.2);

        // 拟合直线：在横轴范围内按正负两段各画一条（sign 项在 v=0 处有跳变）
        double xMin = chart.xMin, xMax = chart.xMax;
        chart.addLine(new double[]{xMin, 0, 0, xMax},
                new double[]{kS * Math.signum(xMin) + kV * xMin, kS, -kS, kS * Math.signum(xMax) + kV * xMax},
                "#E53935", 1.6);
        chart.write(new File(dir, "fit_sva.svg"));
    }

    /**
     * 画「残差 − 加速度」散点与 kA 拟合直线。
     *
     * @param a        轮加速度序列，tick/s²（横轴）
     * @param residual 固定 kS/kV 后的残差 {@code V − kS·sign(v) − kV·v}，V（纵轴）
     * @param kA       拟合加速度增益，V/(tick/s²)
     */
    public static void writeResidualChart(File dir, double[] a, double[] residual, double kA) {
        if (dir == null || a.length == 0) {
            return;
        }
        SvgChart chart = new SvgChart(720, 480, "kA fit: residual vs accel",
                "wheel accel (tick/s^2)", "residual volts (V)");
        chart.autoRange(a, residual);
        chart.addPoints(a, residual, "#6A1B9A", 2.2);
        chart.addLine(new double[]{chart.xMin, chart.xMax},
                new double[]{kA * chart.xMin, kA * chart.xMax}, "#E53935", 1.6);
        chart.write(new File(dir, "residual_ka.svg"));
    }

    /**
     * 画闭环阶跃响应：响应曲线 + 目标线 + 峰值标记。
     *
     * @param fileName 输出文件名（如 {@code fit_feedback_FB_AXIAL.svg}）
     * @param t        时间序列，s（横轴）
     * @param y        响应序列（如沿轴位移，in）（纵轴）
     * @param target   阶跃目标值（同 y 量纲）
     * @param peakTime 峰值时刻，s；{@code < 0} 表示不标记
     * @param title    图表标题
     */
    public static void writeStepChart(File dir, String fileName, double[] t, double[] y, double target,
                                      double peakTime, String title) {
        if (dir == null || t.length == 0) {
            return;
        }
        SvgChart chart = new SvgChart(720, 480, title, "time (s)", "response");
        chart.autoRange(t, y);
        chart.includeY(target);
        chart.addPoints(t, y, "#1565C0", 1.8);
        chart.addLine(new double[]{chart.xMin, chart.xMax},
                new double[]{target, target}, "#43A047", 1.6);
        if (peakTime >= 0 && peakTime <= chart.xMax && peakTime >= chart.xMin) {
            double sx = chart.toSvgX(peakTime);
            chart.raw("<line x1=\"" + fmt(sx) + "\" y1=\"" + chart.pad + "\" x2=\"" + fmt(sx)
                    + "\" y2=\"" + (chart.height - chart.pad) + "\" stroke=\"#FB8C00\" stroke-width=\"1\""
                    + " stroke-dasharray=\"4 4\"/>");
        }
        chart.write(new File(dir, fileName));
    }

    // ---- 底层写文件 ----

    private static void writeFile(File file, String content) {
        try (FileOutputStream fos = new FileOutputStream(file);
             OutputStreamWriter w = new OutputStreamWriter(fos, StandardCharsets.UTF_8)) {
            w.write(content);
            w.flush();
            fos.getFD().sync();
        } catch (IOException e) {
            RobotLog.ee(TAG, "写入 " + file.getAbsolutePath() + " 失败: " + e.getMessage());
        }
    }

    private static String fmt(double x) {
        return String.format(Locale.US, "%.2f", x);
    }

    /**
     * 极简 SVG 图表：支持散点、折线、目标线、自动坐标范围与简单刻度。
     * 不依赖任何绘图库，输出纯文本 SVG。
     */
    private static final class SvgChart {
        final int width, height, pad = 48;
        final String title, xLabel, yLabel;
        double xMin, xMax, yMin, yMax;
        private final StringBuilder body = new StringBuilder();

        SvgChart(int width, int height, String title, String xLabel, String yLabel) {
            this.width = width;
            this.height = height;
            this.title = title;
            this.xLabel = xLabel;
            this.yLabel = yLabel;
            this.xMin = 0;
            this.xMax = 1;
            this.yMin = 0;
            this.yMax = 1;
        }

        /** 由两组序列自动确定横纵轴范围（含 5% 边距，退化时给默认跨度）。 */
        void autoRange(double[] x, double[] y) {
            double x0 = Double.POSITIVE_INFINITY, x1 = Double.NEGATIVE_INFINITY;
            double y0 = Double.POSITIVE_INFINITY, y1 = Double.NEGATIVE_INFINITY;
            for (int i = 0; i < x.length; i++) {
                x0 = Math.min(x0, x[i]);
                x1 = Math.max(x1, x[i]);
                y0 = Math.min(y0, y[i]);
                y1 = Math.max(y1, y[i]);
            }
            setRange(x0, x1, y0, y1);
        }

        void includeY(double v) {
            yMin = Math.min(yMin, v - Math.abs(v) * 0.05 - 1e-3);
            yMax = Math.max(yMax, v + Math.abs(v) * 0.05 + 1e-3);
        }

        private void setRange(double x0, double x1, double y0, double y1) {
            double xSpan = x1 - x0, ySpan = y1 - y0;
            if (xSpan <= 0) {
                xSpan = Math.max(1e-6, Math.abs(x0) * 0.1 + 1e-3);
            }
            if (ySpan <= 0) {
                ySpan = Math.max(1e-6, Math.abs(y0) * 0.1 + 1e-3);
            }
            xMin = x0 - xSpan * 0.05;
            xMax = x1 + xSpan * 0.05;
            yMin = y0 - ySpan * 0.05;
            yMax = y1 + ySpan * 0.05;
        }

        double toSvgX(double x) {
            return pad + (x - xMin) / (xMax - xMin) * (width - 2.0 * pad);
        }

        double toSvgY(double y) {
            return height - pad - (y - yMin) / (yMax - yMin) * (height - 2.0 * pad);
        }

        void addPoints(double[] x, double[] y, String color, double radius) {
            for (int i = 0; i < x.length; i++) {
                raw("<circle cx=\"" + fmt(toSvgX(x[i])) + "\" cy=\"" + fmt(toSvgY(y[i]))
                        + "\" r=\"" + fmt(radius) + "\" fill=\"" + color + "\"/>");
            }
        }

        void addLine(double[] x, double[] y, String color, double strokeWidth) {
            StringBuilder pts = new StringBuilder();
            for (int i = 0; i < x.length; i++) {
                pts.append(fmt(toSvgX(x[i]))).append(',').append(fmt(toSvgY(y[i]))).append(' ');
            }
            raw("<polyline fill=\"none\" stroke=\"" + color + "\" stroke-width=\""
                    + fmt(strokeWidth) + "\" points=\"" + pts.toString().trim() + "\"/>");
        }

        void raw(String svgFragment) {
            body.append("  ").append(svgFragment).append('\n');
        }

        void write(File file) {
            StringBuilder sb = new StringBuilder(body.length() + 1024);
            sb.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            sb.append("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"").append(width)
                    .append("\" height=\"").append(height).append("\" viewBox=\"0 0 ")
                    .append(width).append(' ').append(height).append("\">\n");
            sb.append("  <rect x=\"0\" y=\"0\" width=\"").append(width).append("\" height=\"")
                    .append(height).append("\" fill=\"#FFFFFF\"/>\n");
            sb.append("  <text x=\"").append(width / 2).append("\" y=\"22\" text-anchor=\"middle\" "
                    + "font-family=\"sans-serif\" font-size=\"16\" fill=\"#212121\">")
                    .append(escape(title)).append("</text>\n");
            // 绘图区边框
            sb.append("  <rect x=\"").append(pad).append("\" y=\"").append(pad).append("\" width=\"")
                    .append(width - 2 * pad).append("\" height=\"").append(height - 2 * pad)
                    .append("\" fill=\"none\" stroke=\"#BDBDBD\" stroke-width=\"1\"/>\n");
            sb.append(body);
            // 坐标轴文字
            sb.append("  <text x=\"").append(width / 2).append("\" y=\"").append(height - 8)
                    .append("\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" "
                            + "fill=\"#424242\">").append(escape(xLabel)).append("</text>\n");
            sb.append("  <text x=\"14\" y=\"").append(height / 2).append("\" text-anchor=\"middle\" "
                    + "transform=\"rotate(-90 14 ").append(height / 2)
                    .append(")\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#424242\">")
                    .append(escape(yLabel)).append("</text>\n");
            // 范围标注
            sb.append(rangeLabel(xMin, pad, height - pad + 14, "start"));
            sb.append(rangeLabel(xMax, width - pad, height - pad + 14, "end"));
            sb.append(rangeLabelY(yMax, pad, pad - 6));
            sb.append(rangeLabelY(yMin, pad, height - pad + 14));
            sb.append("</svg>\n");
            writeFile(file, sb.toString());
        }

        private String rangeLabel(double value, double x, double y, String anchor) {
            return "  <text x=\"" + fmt(x) + "\" y=\"" + fmt(y) + "\" text-anchor=\"" + anchor
                    + "\" font-family=\"sans-serif\" font-size=\"11\" fill=\"#757575\">"
                    + fmt(value) + "</text>\n";
        }

        private String rangeLabelY(double value, double x, double y) {
            return "  <text x=\"" + fmt(x - 6) + "\" y=\"" + fmt(y)
                    + "\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"11\" "
                    + "fill=\"#757575\">" + fmt(value) + "</text>\n";
        }

        private static String escape(String s) {
            return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
        }
    }
}
