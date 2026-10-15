package org.firstinspires.ftc.teamcode.utility.Geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.utility.Vector2D;
import org.junit.Test;

/**
 * {@link ConvexPolygon} 单元测试，覆盖构造、凸性校验、坐标变换、包含、相交、交集、最近向量。
 */
public class ConvexPolygonTest {

    private static final double EPS = 1e-6;

    private ConvexPolygon unitSquare() {
        return new ConvexPolygon(
                new Vector2D(0, 0),
                new Vector2D(10, 0),
                new Vector2D(10, 10),
                new Vector2D(0, 10)
        );
    }

    // ==================== 构造与凸性 ====================

    @Test(expected = IllegalArgumentException.class)
    public void constructor_lessThanThreeVertices_throws() {
        new ConvexPolygon(new Vector2D[]{new Vector2D(0, 0), new Vector2D(1, 1)});
    }

    @Test(expected = IllegalArgumentException.class)
    public void constructor_nonConvex_throws() {
        // 凹四边形（箭头形）
        new ConvexPolygon(
                new Vector2D(0, 0),
                new Vector2D(10, 0),
                new Vector2D(5, 5),
                new Vector2D(10, 10),
                new Vector2D(0, 10)
        );
    }

    @Test
    public void constructor_verticesSortedCCW() {
        ConvexPolygon p = unitSquare();
        Vector2D[] v = p.getVertices();
        // 逆时针：面积为正
        double area = 0;
        for (int i = 0; i < v.length; i++) {
            Vector2D a = v[i];
            Vector2D b = v[(i + 1) % v.length];
            area += a.getX() * b.getY() - b.getX() * a.getY();
        }
        assertTrue("多边形应按逆时针排序", area > 0);
    }

    // ==================== Contains ====================

    @Test
    public void contains_pointInside_true() {
        assertTrue(unitSquare().Contains(5, 5));
    }

    @Test
    public void contains_pointOnEdge_true() {
        assertTrue(unitSquare().Contains(5, 0));
    }

    @Test
    public void contains_pointOutside_false() {
        assertFalse(unitSquare().Contains(15, 5));
    }

    @Test
    public void contains_polygonInside_true() {
        ConvexPolygon outer = unitSquare();
        ConvexPolygon inner = new ConvexPolygon(
                new Vector2D(2, 2), new Vector2D(4, 2),
                new Vector2D(4, 4), new Vector2D(2, 4)
        );
        assertTrue(outer.Contains(inner));
    }

    @Test
    public void contains_polygonPartial_false() {
        ConvexPolygon a = unitSquare();
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(5, 5), new Vector2D(15, 5),
                new Vector2D(15, 15), new Vector2D(5, 15)
        );
        assertFalse(a.Contains(b));
    }

    // ==================== NearestVectorFrom ====================

    @Test
    public void nearestVectorFrom_pointInside_zero() {
        Vector2D n = unitSquare().NearestVectorFrom(5, 5);
        assertEquals(0, n.getX(), EPS);
        assertEquals(0, n.getY(), EPS);
    }

    @Test
    public void nearestVectorFrom_pointOutside_correct() {
        // 点 (15,5) 在正方形右侧，最近点应为 (10,5)，向量 (-5,0)
        Vector2D n = unitSquare().NearestVectorFrom(15, 5);
        assertEquals(-5, n.getX(), EPS);
        assertEquals(0, n.getY(), EPS);
    }

    @Test
    public void nearestVectorFrom_pointOutsideCorner_correct() {
        // 点 (15,15) 最近点为 (10,10)
        Vector2D n = unitSquare().NearestVectorFrom(15, 15);
        assertEquals(-5, n.getX(), EPS);
        assertEquals(-5, n.getY(), EPS);
    }

    // ==================== 坐标变换 ====================

    @Test
    public void inRelative_thenInAbsolute_recoversOriginal() {
        ConvexPolygon original = unitSquare();
        double x = 3, y = 4, theta = 0.5;
        ConvexPolygon rel = original.inRelative(x, y, theta);
        ConvexPolygon back = rel.inAbsolute(x, y, theta);
        Vector2D[] ov = original.getVertices();
        Vector2D[] bv = back.getVertices();
        assertEquals(ov.length, bv.length);
        for (int i = 0; i < ov.length; i++) {
            assertEquals(ov[i].getX(), bv[i].getX(), EPS);
            assertEquals(ov[i].getY(), bv[i].getY(), EPS);
        }
    }

    // ==================== IsIntersected ====================

    @Test
    public void isIntersected_overlapping_true() {
        ConvexPolygon a = unitSquare();
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(5, 5), new Vector2D(15, 5),
                new Vector2D(15, 15), new Vector2D(5, 15)
        );
        assertTrue(a.IsIntersected(b));
    }

    @Test
    public void isIntersected_disjoint_false() {
        ConvexPolygon a = unitSquare();
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(20, 20), new Vector2D(30, 20),
                new Vector2D(30, 30), new Vector2D(20, 30)
        );
        assertFalse(a.IsIntersected(b));
    }

    @Test
    public void isIntersected_touchingAtEdge_true() {
        ConvexPolygon a = unitSquare();
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(10, 0), new Vector2D(20, 0),
                new Vector2D(20, 10), new Vector2D(10, 10)
        );
        assertTrue(a.IsIntersected(b));
    }

    // ==================== IntersectWith ====================

    @Test
    public void intersectWith_overlappingSquares_correctArea() {
        ConvexPolygon a = unitSquare(); // (0,0)-(10,10)
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(5, -5), new Vector2D(15, -5),
                new Vector2D(15, 5), new Vector2D(5, 5)
        );
        // 交集为 (5,0)-(10,0)-(10,5)-(5,5)，面积 25
        ConvexPolygon inter = a.IntersectWith(b);
        // 用鞋带公式算面积
        Vector2D[] v = inter.getVertices();
        double area = 0;
        for (int i = 0; i < v.length; i++) {
            Vector2D p1 = v[i];
            Vector2D p2 = v[(i + 1) % v.length];
            area += p1.getX() * p2.getY() - p2.getX() * p1.getY();
        }
        area = Math.abs(area) / 2;
        assertEquals(25, area, EPS);
    }

    @Test(expected = IllegalStateException.class)
    public void intersectWith_disjoint_throws() {
        ConvexPolygon a = unitSquare();
        ConvexPolygon b = new ConvexPolygon(
                new Vector2D(20, 20), new Vector2D(30, 20),
                new Vector2D(30, 30), new Vector2D(20, 30)
        );
        a.IntersectWith(b);
    }
}
