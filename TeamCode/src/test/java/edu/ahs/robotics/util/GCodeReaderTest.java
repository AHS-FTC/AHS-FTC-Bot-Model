package edu.ahs.robotics.util;

import org.junit.Before;
import org.junit.Test;

import java.util.List;

import edu.ahs.robotics.control.Point;

import static org.junit.Assert.*;

public class GCodeReaderTest {

    @Before
    public void init() {
        FTCUtilities.startTestMode();
    }

    @Test
    public void testFile(){
        System.out.println(FTCUtilities.getLogDirectory());
        //GCodeReader.openFile(Boolean.TRUE,"1001.csv");
        List<List<Point>> testRoute = GCodeReader.openFile("GCodeReaderTestFile.csv");

        //First path starting point
        assertEquals(new Point(-0.3121,1.7034), testRoute.get(0).get(0));

        //First path middle point
        //Line 7 in document but actually 5 because 2 first points discarded due to z not equal to 0
        assertEquals(new Point(-0.1225,1.1547), testRoute.get(0).get(5));

        //First path end point
        //Line 21 in document but actually 5 because 2 first points discarded due to z not equal to 0
        assertEquals(new Point(0.6346,0.6876), testRoute.get(0).get(19));

        //Second path starting point
        assertEquals(new Point(0.6346,0.6876), testRoute.get(1).get(0));

        //Second path middle point
        assertEquals(new Point(1.2549,0.4041), testRoute.get(1).get(12));

        //Second path end point (Full size is Line 74 from start of second path in doc. but last 2 points discarded due to z)
        assertEquals(new Point(-1.5213,-1.0398), testRoute.get(1).get(72));
    }
}