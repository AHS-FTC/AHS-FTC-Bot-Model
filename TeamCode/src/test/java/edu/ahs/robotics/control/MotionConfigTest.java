package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class MotionConfigTest {

    @Test
    public void testNoOBMCommands(){
        MotionConfig mc = new MotionConfig();

        assertFalse(mc.checkOBMCommands(null));
    }

}