package edu.ahs.robotics.hardware;

import org.junit.Test;

import java.util.ArrayList;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;

import static org.junit.Assert.*;

public class PathTest {

    @Test
    public void getTargetPosition() {
        ArrayList<Path.Point> points = new ArrayList<>();
        points.add(new Path.Point(0,0));
        points.add(new Path.Point(1,1));
        Path path = new Path(points);
        Position targetPosition = path.getTargetPosition(0);
        assertEquals(targetPosition.x, 0, .001);
        assertEquals(targetPosition.y, 0, .001);
        assertEquals(targetPosition.getHeadingInDegrees(), 45, .01);

    }
}