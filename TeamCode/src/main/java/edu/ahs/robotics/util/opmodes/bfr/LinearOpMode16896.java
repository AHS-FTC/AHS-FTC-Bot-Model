package edu.ahs.robotics.util.opmodes.bfr;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.util.ftc.ErrorStealer;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.TextLogger;


/**
 * 16896's own LinearOpMode class that automatically steals errors and sets the OpMode in FTCUtilites.
 * Also establishes mainLogger, an instance of TextLogger for general use and stolen error output.
 * To use, create a subclass and override initialize() and runProgram() with your own code
 * @see IterativeOpMode16896
 */
public abstract class LinearOpMode16896 extends LinearOpMode {

    protected TextLogger mainLogger;

    @Override
    public final void runOpMode() {
        try {
            mainLogger = new TextLogger("linearOpModeMain", "main");
            mainLogger.startWriting();
            ErrorStealer.setErrorLogger(mainLogger);

            FTCUtilities.setOpMode(this);

            initialize();
            mainLogger.logLine("Initialization Finished");

            super.waitForStart();

            runProgram();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
    }

    //Override this!
    protected abstract void initialize();

    //Override this too!
    protected abstract void runProgram();

    @Override
    public final void waitForStart() { //Don't override or call this!
        throw new UnsupportedOperationException("Don't use waitForStart() in the 16896 LinearOpMode! The OpMode is separated into start() and runProgram");
    }
}
