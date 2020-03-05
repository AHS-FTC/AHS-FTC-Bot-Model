package edu.ahs.robotics.util.opmodes.bfr;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.ahs.robotics.util.ftc.ErrorStealer;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.TextLogger;

/**
 * 16896's own OpMode class that automatically steals loose errors and sets the FTCUtilities OpMode.
 * Also establishes mainLogger, an instance of TextLogger for general use and stolen error output.
 * To use, create a subclass and override methods with your own code.
 * @see LinearOpMode16896
 */
public abstract class IterativeOpMode16896 extends OpMode {
    protected TextLogger mainLogger;

    @Override
    public final void init() {
        mainLogger = new TextLogger("iterativeOpModeMain", "main");
        mainLogger.startWriting();
        ErrorStealer.setErrorLogger(mainLogger);
        FTCUtilities.setOpMode(this);

        try {
            initialize();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }

        mainLogger.logLine("Initialization Finished");
    }

    @Override
    public final void init_loop() {
        try {
            repeatAfterInit();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
    }

    @Override
    public final void start() {
        mainLogger.logLine("Start Was Pressed");
        try {
            begin();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
        mainLogger.logLine("Start Code Finished");
    }

    @Override
    public final void loop() {
        try {
            iterate();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
    }

    @Override
    public final void stop() {
        try {
            teardown();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }
        mainLogger.logLine("Teardown Finished");
        mainLogger.stopWriting();
    }

    /**
     * Override in place of init();
     */
    protected abstract void initialize(); //Override this!

    /**
     * Override in place of init_loop();
     */
    protected abstract void repeatAfterInit(); //Override this!

    /**
     * Override in place of start();
     */
    protected abstract void begin(); //Override this!

    /**
     * Override in place of loop();
     */
    protected abstract void iterate(); //Override this!

    /**
     * Override in place of stop();
     */
    protected abstract void teardown(); //Override this!
}
