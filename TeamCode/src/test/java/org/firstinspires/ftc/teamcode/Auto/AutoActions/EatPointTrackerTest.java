package org.firstinspires.ftc.teamcode.Auto.AutoActions;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.EatPointTracker;
import org.firstinspires.ftc.teamcode.utility.Vector2D;
import org.junit.Before;
import org.junit.Test;

/** {@link EatPointTracker} 单元测试。 */
public class EatPointTrackerTest {

    @Before
    public void setUp() {
        EatPointTracker.clear();
    }

    @Test
    public void red_usesSharedEatPoints() {
        Vector2D p1 = EatPointTracker.eatPoint(TeamColor.RED, EatPointTracker.FIRST);
        assertEquals(HypParams.EatPoint1.getX(), p1.getX(), 1e-9);
        assertEquals(HypParams.EatPoint1.getY(), p1.getY(), 1e-9);
    }

    @Test
    public void blue_isCentralSymmetric() {
        for (int idx : new int[]{EatPointTracker.FIRST, EatPointTracker.SECOND}) {
            Vector2D red = EatPointTracker.eatPoint(TeamColor.RED, idx);
            Vector2D blue = EatPointTracker.eatPoint(TeamColor.BLUE, idx);
            assertEquals(-red.getX(), blue.getX(), 1e-9);
            assertEquals(-red.getY(), blue.getY(), 1e-9);
        }
    }

    @Test
    public void markVisited_and_isVisited() {
        assertFalse(EatPointTracker.isVisited(EatPointTracker.FIRST));
        EatPointTracker.markVisited(EatPointTracker.FIRST);
        assertTrue(EatPointTracker.isVisited(EatPointTracker.FIRST));
        assertFalse(EatPointTracker.allVisited());
        EatPointTracker.markVisited(EatPointTracker.SECOND);
        assertTrue(EatPointTracker.allVisited());
    }

    @Test
    public void nearestUnvisited_prefersUnvisitedPoint() {
        // 红队：起点靠近 EatPoint1=(24,-72)，默认应选 FIRST
        assertEquals(EatPointTracker.FIRST,
                EatPointTracker.nearestUnvisitedIndex(TeamColor.RED, 24, -60));
        // 标记 FIRST 已访问后，即使更近也应改选 SECOND
        EatPointTracker.markVisited(EatPointTracker.FIRST);
        assertEquals(EatPointTracker.SECOND,
                EatPointTracker.nearestUnvisitedIndex(TeamColor.RED, 24, -60));
    }

    @Test
    public void nearestUnvisited_fallsBackToNearestWhenAllVisited() {
        EatPointTracker.markVisited(EatPointTracker.FIRST);
        EatPointTracker.markVisited(EatPointTracker.SECOND);
        assertEquals(EatPointTracker.FIRST,
                EatPointTracker.nearestUnvisitedIndex(TeamColor.RED, 24, -60));
    }
}
