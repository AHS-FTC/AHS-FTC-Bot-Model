package edu.ahs.robotics.util;

/**
 * Class to mock out time based operations for unit tests.
 * Should be included in the test packages, but in util to enable visibility to FTCUtilities without an interface
 * @author Alex Appleby
 */
public class MockClock {
    private int timeStep = 1;
    private int i;
    private Mode mode;

    public enum Mode{
        ADVANCE_BY_1_MILLI,
        ADVANCE_BY_10_MILLIS,
        CUSTOM_TIME_STEP
    }

    public MockClock(Mode mode) {
        this.mode = mode;
        if(mode == Mode.ADVANCE_BY_10_MILLIS){
            timeStep = 10;
        }
    }

    public MockClock(){
        this(Mode.ADVANCE_BY_1_MILLI);
    }

    public void setTimeStep(int timeStep){
        this.timeStep = timeStep;
    }


    /**
     * Intended to act as a mock of System.getCurrentTimeMillis.
     * @return the time in milliseconds
     */
    public long getCurrentTimeMillis(){
        long time = i * timeStep;
        i++;
        return time;
    }
}
