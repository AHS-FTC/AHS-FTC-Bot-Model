package edu.ahs.robotics.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.util.ErrorStealer;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.loggers.TextLogger;


/**
 * 16896's own LinearOpMode class that automatically steals errors and sets the OpMode in FTCUtilites.
 * Also establishes mainLogger, an instance of TextLogger for general use and stolen error output.
 * To use, create a subclass and override runProgram with your own code
 */
public abstract class LinearOpMode16896 extends LinearOpMode {

    protected TextLogger mainLogger;

    @Override
    public final void runOpMode() throws InterruptedException {
        try {
            mainLogger = new TextLogger("linearOpModeMain", "main");
            mainLogger.startWriting();
            ErrorStealer.setErrorLogger(mainLogger);

            FTCUtilities.setOpMode(this);

            runProgram();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
    }

    //Override this!
    protected abstract void runProgram();
}
