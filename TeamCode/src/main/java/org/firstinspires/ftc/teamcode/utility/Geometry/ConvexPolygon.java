package org.firstinspires.ftc.teamcode.utility.Geometry;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.ArrayList;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 凸多边形类，用于描述平面凸多边形并基于其几何性质提供常用运算。
 *
 * <p>每个凸多边形由一组顶点定义，顶点总数 {@code n >= 3}，否则构造时抛出异常。
 * 构造函数会将传入顶点按<b>逆时针</b>顺序排序，并校验多边形的凸性；
 * 若顶点不能构成凸多边形则抛出 {@link IllegalArgumentException}。
 *
 * <p>支持的能力包括：坐标转换（绝对坐标 ↔ 相对坐标）、点/多边形包含检测、
 * 多边形相交判定、多边形交集计算（Sutherland-Hodgman 裁剪算法）、
 * 以及点到多边形边界的最近向量计算。
 */
public class ConvexPolygon {
    /** 多边形顶点数组，已按逆时针顺序排列 */
    private Vector2D[] vertices;
    /** 顶点数量 */
    private int n;

    /**
     * 构造凸多边形。
     *
     * <p>将传入的顶点数组克隆后按逆时针方向排序，并校验凸性。
     * 若顶点数不足 3 个或不构成凸多边形则抛出异常。
     *
     * @param vertices 顶点数组，长度必须 >= 3
     * @throws IllegalArgumentException 顶点为空、数量不足 3，或不构成凸多边形时抛出
     */
    public ConvexPolygon(Vector2D[] vertices) {
        if (vertices == null || vertices.length < 3) {
            throw new IllegalArgumentException("ConvexPolygon requires at least 3 vertices");
        }
        this.n = vertices.length;
        this.vertices = sortVerticesCCW(vertices.clone());
        if (!isConvex()) {
            throw new IllegalArgumentException("Given vertices do not form a convex polygon");
        }
    }

    /** 三角形便捷构造函数 */
    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3) {
        this(new Vector2D[]{p1, p2, p3});
    }

    /** 四边形便捷构造函数 */
    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        this(new Vector2D[]{p1, p2, p3, p4});
    }

    /** 五边形便捷构造函数 */
    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4, Vector2D p5) {
        this(new Vector2D[]{p1, p2, p3, p4, p5});
    }

    /** 六边形便捷构造函数 */
    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4, Vector2D p5, Vector2D p6) {
        this(new Vector2D[]{p1, p2, p3, p4, p5, p6});
    }

    /**
     * 将顶点按逆时针（CCW）方向排序。
     *
     * <p>先计算所有顶点的几何中心，再按各顶点相对中心的极角升序排列；
     * 最后通过有符号面积判断方向，若为顺时针则反转数组，确保结果为逆时针。
     */
    private Vector2D[] sortVerticesCCW(Vector2D[] verts) {
        double centerX = 0, centerY = 0;
        for (Vector2D p : verts) {
            centerX += p.getX();
            centerY += p.getY();
        }
        centerX /= verts.length;
        centerY /= verts.length;

        final double cx = centerX;
        final double cy = centerY;

        Arrays.sort(verts, (a, b) -> {
            double angleA = Math.atan2(a.getY() - cy, a.getX() - cx);
            double angleB = Math.atan2(b.getY() - cy, b.getX() - cx);
            return Double.compare(angleA, angleB);
        });

        if (signedArea(verts) < 0) {
            reverseArray(verts);
        }

        return verts;
    }

    /**
     * 计算多边形的有符号面积（鞋带公式）。
     *
     * <p>逆时针多边形返回正值，顺时针返回负值，绝对值为面积的两倍。
     */
    private double signedArea(Vector2D[] verts) {
        double area = 0;
        int m = verts.length;
        for (int i = 0; i < m; i++) {
            Vector2D p1 = verts[i];
            Vector2D p2 = verts[(i + 1) % m];
            area += p1.getX() * p2.getY() - p2.getX() * p1.getY();
        }
        return area / 2;
    }

    /** 原地反转数组 */
    private void reverseArray(Vector2D[] verts) {
        int i = 0, j = verts.length - 1;
        while (i < j) {
            Vector2D tmp = verts[i];
            verts[i] = verts[j];
            verts[j] = tmp;
            i++;
            j--;
        }
    }

    /**
     * 判断当前顶点序列是否构成凸多边形。
     *
     * <p>依次计算每三个连续顶点的叉积：若叉积同时出现正负号则存在凹角，返回 false；
     * 若所有叉积均接近零（共线）也返回 false。
     *
     * @return 是否为凸多边形
     */
    public boolean isConvex() {
        if (n < 3) return false;

        boolean hasPositive = false;
        boolean hasNegative = false;
        boolean isAllCollinear = true;

        for (int i = 0; i < n; i++) {
            Vector2D p1 = vertices[i];
            Vector2D p2 = vertices[(i + 1) % n];
            Vector2D p3 = vertices[(i + 2) % n];

            double cross = crossProduct(p1, p2, p3);
            if (Math.abs(cross) > 1e-10) {
                isAllCollinear = false;
                if (cross > 0) hasPositive = true;
                if (cross < 0) hasNegative = true;
            }

            if (hasPositive && hasNegative) return false;
        }
        if (isAllCollinear) return false;
        return true;
    }

    /**
     * 计算三个点 p1→p2→p3 的叉积，用于判断转向。
     *
     * @return 正值表示左转（逆时针），负值表示右转（顺时针），零表示共线
     */
    private double crossProduct(Vector2D p1, Vector2D p2, Vector2D p3) {
        double vx1 = p2.getX() - p1.getX();
        double vy1 = p2.getY() - p1.getY();
        double vx2 = p3.getX() - p2.getX();
        double vy2 = p3.getY() - p2.getY();
        return vx1 * vy2 - vy1 * vx2;
    }

    /**
     * 将当前多边形从<b>绝对坐标系</b>转换到以 {@code (x, y)} 为原点、
     * 坐标轴旋转 {@code theta} 的<b>相对坐标系</b>。
     *
     * <p>变换顺序：先平移 {@code (-x, -y)}，再旋转 {@code theta} 角度。
     *
     * @param x     相对坐标系原点在绝对坐标系中的 x 坐标
     * @param y     相对坐标系原点在绝对坐标系中的 y 坐标
     * @param theta 相对坐标系相对绝对坐标系的旋转角（弧度）
     * @return 变换后的新凸多边形
     */
    public ConvexPolygon inRelative(double x, double y, double theta) {
        Vector2D[] transformed = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            transformed[i] = Vector2D.rotate(
                new Vector2D(vertices[i].getX() - x, vertices[i].getY() - y),
                theta
            );
        }
        return new ConvexPolygon(transformed);
    }

    /**
     * 将当前多边形从绝对坐标系转换到以位姿 {@code p} 为参考的相对坐标系。
     *
     * <p>等价于 {@code inRelative(p.position.x, p.position.y, p.heading)}。
     *
     * @param p 参考位姿（原点 + 朝向）
     * @return 变换后的新凸多边形
     */
    public ConvexPolygon inRelative(Pose2d p) {
        return inRelative(p.position.x, p.position.y, p.heading.toDouble());
    }

    /**
     * 将当前多边形从<b>相对坐标系</b>转换回<b>绝对坐标系</b>。
     *
     * <p>变换顺序：先旋转 {@code -theta}，再平移 {@code (x, y)}。
     * 即 {@link #inRelative(double, double, double)} 的逆变换。
     *
     * @param x     相对坐标系原点在绝对坐标系中的 x 坐标
     * @param y     相对坐标系原点在绝对坐标系中的 y 坐标
     * @param theta 相对坐标系相对绝对坐标系的旋转角（弧度）
     * @return 变换后的新凸多边形
     */
    public ConvexPolygon inAbsolute(double x, double y, double theta) {
        Vector2D[] transformed = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            Vector2D rotated = Vector2D.rotate(vertices[i], -theta);
            transformed[i] = new Vector2D(rotated.getX() + x, rotated.getY() + y);
        }
        return new ConvexPolygon(transformed);
    }

    /**
     * 将当前多边形从相对坐标系转换回以位姿 {@code p} 为参考的绝对坐标系。
     *
     * <p>等价于 {@code inAbsolute(p.position.x, p.position.y, p.heading)}。
     *
     * @param p 参考位姿（原点 + 朝向）
     * @return 变换后的新凸多边形
     */
    public ConvexPolygon inAbsolute(Pose2d p) {
        return inAbsolute(p.position.x, p.position.y, p.heading.toDouble());
    }

    /**
     * 计算由点 {@code (x, y)} 指向当前多边形上离其最近点的向量。
     *
     * <p>若点在多边形内部（含边界），返回零向量。
     *
     * @param x 点的 x 坐标
     * @param y 点的 y 坐标
     * @return 指向多边形最近点的向量
     */
    public Vector2D NearestVectorFrom(double x, double y) {
        return NearestVectorFrom(new Vector2D(x, y));
    }

    /**
     * 计算由点 {@code p} 指向当前多边形上离其最近点的向量。
     *
     * <p>遍历每条边，求点到线段的最近点，取距离最小者。
     * 若点在多边形内部（含边界），返回零向量。
     *
     * @param p 输入点
     * @return 指向多边形最近点的向量
     */
    public Vector2D NearestVectorFrom(Vector2D p) {
        if (Contains(p)) {
            return new Vector2D(0, 0);
        }

        Vector2D nearest = null;
        double minDistSq = Double.MAX_VALUE;

        for (int i = 0; i < n; i++) {
            Vector2D v1 = vertices[i];
            Vector2D v2 = vertices[(i + 1) % n];

            Vector2D closest = closestPointOnSegment(p, v1, v2);
            double distSq = (closest.getX() - p.getX()) * (closest.getX() - p.getX()) +
                            (closest.getY() - p.getY()) * (closest.getY() - p.getY());

            if (distSq < minDistSq) {
                minDistSq = distSq;
                nearest = closest;
            }
        }

        return new Vector2D(nearest.getX() - p.getX(), nearest.getY() - p.getY());
    }

    /**
     * 求点 {@code p} 在线段 {@code v1-v2} 上的最近点（投影并夹到 [0,1]）。
     */
    private Vector2D closestPointOnSegment(Vector2D p, Vector2D v1, Vector2D v2) {
        double dx = v2.getX() - v1.getX();
        double dy = v2.getY() - v1.getY();
        double lenSq = dx * dx + dy * dy;

        if (lenSq < 1e-10) return new Vector2D(v1.getX(), v1.getY());

        double t = Math.max(0, Math.min(1, ((p.getX() - v1.getX()) * dx + (p.getY() - v1.getY()) * dy) / lenSq));

        return new Vector2D(v1.getX() + t * dx, v1.getY() + t * dy);
    }

    /**
     * 判断点 {@code (x, y)} 是否在当前凸多边形内部（含边界）。
     *
     * <p>对严格逆时针凸多边形，点在内部（含边界）的充要条件是：
     * 对每条边 ViVi+1，点 P 均在边的左侧，即叉积 (ViVi+1) × (ViP) >= 0（允许数值误差）。
     *
     * @param x 点的 x 坐标
     * @param y 点的 y 坐标
     * @return 点是否在多边形内（含边界）
     */
    public boolean Contains(double x, double y) {
        return Contains(new Vector2D(x, y));
    }

    /**
     * 判断点 {@code p} 是否在当前凸多边形内部（含边界）。
     *
     * @param p 输入点
     * @return 点是否在多边形内（含边界）
     */
    public boolean Contains(Vector2D p) {
        for (int i = 0; i < n; i++) {
            Vector2D v1 = vertices[i];
            Vector2D v2 = vertices[(i + 1) % n];

            if (crossProduct(v1, v2, p) < -1e-10) {
                return false;
            }
        }
        return true;
    }

    /**
     * 判断另一个凸多边形 {@code other} 是否完全包含在当前多边形内（含边界）。
     *
     * <p>实现方法：逐一检查 {@code other} 的所有顶点是否都在当前多边形内。
     *
     * @param other 待检测的凸多边形
     * @return other 是否完全包含于当前多边形
     */
    public boolean Contains(ConvexPolygon other) {
        for (int i = 0; i < other.n; i++) {
            if (!Contains(other.vertices[i])) {
                return false;
            }
        }
        return true;
    }

    /**
     * 判断当前凸多边形与 {@code other} 是否相交（含边界接触）。
     *
     * <p>判定方式：任一多边形的顶点落在另一多边形内，或两边的线段相交。
     *
     * @param other 另一个凸多边形
     * @return 是否相交
     */
    public boolean IsIntersected(ConvexPolygon other) {
        for (int i = 0; i < n; i++) {
            if (other.Contains(vertices[i])) {
                return true;
            }
        }

        for (int i = 0; i < other.n; i++) {
            if (Contains(other.vertices[i])) {
                return true;
            }
        }

        for (int i = 0; i < n; i++) {
            Vector2D a1 = vertices[i];
            Vector2D a2 = vertices[(i + 1) % n];

            for (int j = 0; j < other.n; j++) {
                Vector2D b1 = other.vertices[j];
                Vector2D b2 = other.vertices[(j + 1) % other.n];

                if (segmentsIntersect(a1, a2, b1, b2)) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * 判断两条线段 p1p2 与 p3p4 是否相交（含端点接触与共线重叠）。
     */
    private boolean segmentsIntersect(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        double d1 = isLeft(p3, p4, p1);
        double d2 = isLeft(p3, p4, p2);
        double d3 = isLeft(p1, p2, p3);
        double d4 = isLeft(p1, p2, p4);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }

        if (Math.abs(d1) < 1e-10 && onSegment(p3, p4, p1)) return true;
        if (Math.abs(d2) < 1e-10 && onSegment(p3, p4, p2)) return true;
        if (Math.abs(d3) < 1e-10 && onSegment(p1, p2, p3)) return true;
        if (Math.abs(d4) < 1e-10 && onSegment(p1, p2, p4)) return true;

        return false;
    }

    /**
     * 判断点 q 是否在线段 p1p2 的轴对齐包围盒内（共线时用于判定点是否落在线段上）。
     */
    private boolean onSegment(Vector2D p1, Vector2D p2, Vector2D q) {
        return Math.min(p1.getX(), p2.getX()) - 1e-10 <= q.getX() &&
               q.getX() <= Math.max(p1.getX(), p2.getX()) + 1e-10 &&
               Math.min(p1.getY(), p2.getY()) - 1e-10 <= q.getY() &&
               q.getY() <= Math.max(p1.getY(), p2.getY()) + 1e-10;
    }

    /**
     * 计算当前凸多边形与 {@code clip} 的交集多边形。
     *
     * <p>采用 <b>Sutherland-Hodgman</b> 多边形裁剪算法：依次用裁剪多边形（clip）
     * 的每条边作为裁剪线，对被裁剪多边形（subject）进行裁剪。由于两个多边形均为凸，
     * 交集仍为凸多边形。处理完所有裁剪边后，移除顶点序列中的共线点，得到严格凸的交集。
     *
     * <p>若两个多边形不相交，或交集退化为点/线段（最终顶点数 < 3），抛出异常。
     *
     * @param clip 裁剪多边形
     * @return 交集凸多边形
     * @throws IllegalStateException 多边形不相交或交集退化时抛出
     */
    public ConvexPolygon IntersectWith(ConvexPolygon clip) {
        if (!this.IsIntersected(clip)) {
            throw new IllegalStateException("Polygons do not intersect");
        }
        ArrayList<Vector2D> subject = new ArrayList<>();
        for (Vector2D p : this.vertices) subject.add(p);

        for (int i = 0; i < clip.n; i++) {
            if (subject.size() < 3) break;

            Vector2D clipEdgeStart = clip.vertices[i];
            Vector2D clipEdgeEnd = clip.vertices[(i + 1) % clip.n];

            ArrayList<Vector2D> output = new ArrayList<>();

            for (int j = 0; j < subject.size(); j++) {
                Vector2D current = subject.get(j);
                Vector2D next = subject.get((j + 1) % subject.size());

                boolean currentInside = isLeft(clipEdgeStart, clipEdgeEnd, current) >= -1e-10;
                boolean nextInside = isLeft(clipEdgeStart, clipEdgeEnd, next) >= -1e-10;

                if (currentInside) {
                    if (nextInside) {
                        output.add(next);
                    } else {
                        Vector2D intersection = lineIntersection(
                            current, next, clipEdgeStart, clipEdgeEnd
                        );
                        output.add(intersection);
                    }
                } else {
                    if (nextInside) {
                        Vector2D intersection = lineIntersection(
                            current, next, clipEdgeStart, clipEdgeEnd
                        );
                        output.add(intersection);
                        output.add(next);
                    }
                }
            }

            subject = output;
        }

        if (subject.size() < 3) {
            throw new IllegalStateException("Intersection is empty or degenerate (less than 3 vertices)");
        }

        Vector2D[] result = removeCollinearPoints(subject.toArray(new Vector2D[0]));
        if (result.length < 3) {
            throw new IllegalStateException("Intersection is empty or degenerate (less than 3 vertices)");
        }
        return new ConvexPolygon(result);
    }

    /**
     * 计算点 p 在有向边 v1→v2 的左侧还是右侧。
     *
     * @return 正值表示 p 在边左侧，负值表示右侧，零表示共线
     */
    private double isLeft(Vector2D v1, Vector2D v2, Vector2D p) {
        return (v2.getX() - v1.getX()) * (p.getY() - v1.getY()) -
               (v2.getY() - v1.getY()) * (p.getX() - v1.getX());
    }

    /**
     * 计算直线 p1p2 与直线 p3p4 的交点。
     *
     * <p>若两直线平行（分母接近零），返回 p1 作为退化结果。
     */
    private Vector2D lineIntersection(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        double x1 = p1.getX(), y1 = p1.getY();
        double x2 = p2.getX(), y2 = p2.getY();
        double x3 = p3.getX(), y3 = p3.getY();
        double x4 = p4.getX(), y4 = p4.getY();

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-10) {
            return new Vector2D(x1, y1);
        }

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;

        return new Vector2D(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
    }

    /**
     * 移除顶点序列中前后共线的中间点，得到严格凸的顶点列表。
     */
    private Vector2D[] removeCollinearPoints(Vector2D[] pts) {
        int n = pts.length;
        if (n <= 3) return pts;

        ArrayList<Vector2D> result = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            Vector2D prev = pts[(i - 1 + n) % n];
            Vector2D curr = pts[i];
            Vector2D next = pts[(i + 1) % n];

            if (Math.abs(crossProduct(prev, curr, next)) > 1e-10) {
                result.add(curr);
            }
        }

        return result.toArray(new Vector2D[0]);
    }

    /** @return 顶点数量 */
    public int getVertexCount() {
        return n;
    }

    /** @return 顶点数组的拷贝（逆时针顺序） */
    public Vector2D[] getVertices() {
        return vertices.clone();
    }

    /**
     * 获取指定索引的顶点。
     *
     * @param index 顶点索引
     * @return 对应的顶点
     * @throws IndexOutOfBoundsException 索引越界时抛出
     */
    public Vector2D getVertex(int index) {
        if (index < 0 || index >= n) {
            throw new IndexOutOfBoundsException("Vertex index out of bounds");
        }
        return vertices[index];
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ConvexPolygon[");
        for (int i = 0; i < n; i++) {
            sb.append(vertices[i].toString());
            if (i < n - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}
