package org.firstinspires.ftc.teamcode.autos;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.control.obm.SlideCycle;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.Logger;

public class PartialPursuitAuto {
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private Intake intake;

    private SerialServo leftFoundation, rightFoundation, xSlide, gripper;

    private Path quarry, toFoundation, quarry2, foundation2;
    private Path gripFoundation, pullFoundation;

    private Logger logger;
    private SlideCycle slideCycle;
    private OBMCommand nullCommand = new NullCommand();

    private int turnSign;


    public PartialPursuitAuto(Path quarry, Path toFoundation, Path quarry2, Path foundation2, boolean flipToBlue) {
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();

        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        xSlide = ardennes.getySlide();
        gripper = ardennes.getGripper();

        xSlide.setPosition(0);
        gripper.setPosition(0);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        this.quarry = quarry;
        this.toFoundation = toFoundation;
        this.quarry2 = quarry2;
        this.foundation2 = foundation2;

        if(flipToBlue){
            turnSign = -1;
        } else {
            turnSign = 1;
        }

        slideCycle = new SlideCycle(ardennes);

        pullFoundation = new Path(GCodeReader.openFile("2-5-4_pullFoundation.csv"), 24, 24, 48, 28,2,10, flipToBlue);
        gripFoundation = new Path(GCodeReader.openFile("2-5-3_gripFoundation.csv"), 12,12,12, flipToBlue);

        chassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (flipToBlue) {
            chassis.setPosition(-63,-40, 0);
        } else {
            chassis.setPosition(63,-40, Math.PI);
        }


        logger = new Logger("partialPursuitAutoData", "partialPursuit");
        logger.startWriting();

        chassis.startOdometrySystem();
    }

    public void start(){

        //intake.startIntakeWaitForBlock(ardennes.getGripperTrigger());
        intake.runMotors(1);
        chassis.followPath(quarry, 12, (turnSign)*(-Math.PI/4), nullCommand);
        chassis.followPath(toFoundation, 12, Math.PI, nullCommand);
        intake.stopMotors();
        chassis.stopMotors();
        gripper.setPosition(1);
        //FTCUtilities.sleep(2000);

        double targetAngle = (2*Math.PI);
        if(turnSign == -1){
            targetAngle = -Math.PI;
        }

        chassis.globalPointTurn(targetAngle, 0.3, 2500);
        slideCycle.cycleSlides();

        chassis.followPath(gripFoundation, 12, Math.PI, nullCommand);
        chassis.stopMotors();

        leftFoundation.setPosition(1);
        rightFoundation.setPosition(1);

        FTCUtilities.sleep(400);

        chassis.followPath(pullFoundation, 12, 0, nullCommand);

        leftFoundation.setPosition(0);
        rightFoundation.setPosition(0);

        //intake.startIntakeWaitForBlock(ardennes.getGripperTrigger());
        intake.runMotors(1);
        chassis.followPath(quarry2, 12, 0, nullCommand);

        chassis.followPath(foundation2,12, Math.PI, nullCommand); //will have to reset slideCycle next use
        intake.stopMotors();
        chassis.stopMotors();


        FTCUtilities.sleep(2500);

        intake.killThread();
        chassis.stopOdometrySystem();
        logger.stopWriting();
    }
}
