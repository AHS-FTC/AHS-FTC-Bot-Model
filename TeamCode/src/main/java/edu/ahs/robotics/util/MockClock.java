package edu.ahs.robotics.util;

/**
 * Class to mock out time based operations for unit tests.
 * Should be included in the test packages, but in util to enable visibility to FTCUtilities without an interface
 * @author Alex Appleby
 */
public class MockClock {
    private int timeStep = 1;
    private int i;

    public enum Mode{
        ADVANCE_BY_1_MILLI,
        ADVANCE_BY_10_MILLIS,
        CUSTOM_TIME_PROFILE
    }

    public MockClock(Mode mode) {
        if(mode == Mode.ADVANCE_BY_10_MILLIS){
            timeStep = 10;
        } else if(mode == Mode.CUSTOM_TIME_PROFILE){
            throw new UnsupportedOperationException("The custom time profile clock hasn't been developed yet in MockClock. Mock it.");
        }
    }

    public MockClock(){
        this(Mode.ADVANCE_BY_1_MILLI);
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
