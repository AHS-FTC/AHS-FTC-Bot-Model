package edu.ahs.robotics.util.loggers;

/**
 * Can be injected in place of a normal data logger to prevent null pointers or other funny logger business during tests.
 * Not per se a mock object, although it can be used as a mock. May also be used outside of testing to remove logging.
 */
public class MockDataLogger extends DataLogger {
    public MockDataLogger(String key) {
        super("test", key);
    }

    @Override
    public void startWriting() {

    }

    @Override
    public void writeLine() {

    }

    @Override
    public void stopWriting() {

    }

    @Override
    public void append(String category, String data) {

    }
}
