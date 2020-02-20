package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.ContinuosServo;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * Command to synchronously run the Ardennes Tape Measurer
 * @author Alex Appleby and Andrew Seybold
 */
public class TapeMeasureCommand implements OBMCommand {

    private State state = State.INITIAL;
    private long startTime;
    private ContinuosServo tapeMeasure;

    private static final long DURATION = 2000;

    private enum State {
        INITIAL,
        WAITING,
        FINISHED
    }

    public TapeMeasureCommand(ContinuosServo tapeMeasure) {
        this.tapeMeasure = tapeMeasure;
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        switch (state){
            case FINISHED:
                break;
            case INITIAL:
                if(robotState.position.y > 0.0){
                    state = State.WAITING;
                    startTime = FTCUtilities.getCurrentTimeMillis();
                    tapeMeasure.setPower(-0.85);
                }
                break;
            case WAITING:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > DURATION){
                    state = State.FINISHED;
                    tapeMeasure.setPower(0);
                }
                break;
        }
        return false;
    }

    @Override
    public void reset() {
        state = State.INITIAL;
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
