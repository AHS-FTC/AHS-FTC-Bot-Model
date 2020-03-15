package edu.ahs.robotics.util.ftc;

/**
 * Some code that ensures button presses aren't counted twice. Use an and bracket with your condition.
 * </br>
 * <pre>
 * {@code
 * if(button.isPressed() && switch.canFlip()){
 *    //do something
 * }
 * }
 * </pre>
 * @author Andrew Seybold
 */
public class Switch {
    private final static double BUTTON_THRESHOLD = 300; //in millis - time between presses
    private long lastPress;

    public Switch() {
        lastPress = FTCUtilities.getCurrentTimeMillis();
    }

    /**
     * @return Whether or not enough time has elapsed to register a button press again
     */
    public boolean canFlip() {
        if(FTCUtilities.getCurrentTimeMillis() - lastPress > BUTTON_THRESHOLD) {
            lastPress = FTCUtilities.getCurrentTimeMillis();
            return true;
        }
        return false;
    }
}
