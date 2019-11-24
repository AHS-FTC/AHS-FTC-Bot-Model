package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.autocommands.autopaths.IntakeMonitor;
import edu.ahs.robotics.autocommands.obmcommands.IntakeCommand;
import edu.ahs.robotics.autocommands.obmcommands.IntakeCommandWithTrigger;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;

public class Intake implements Executor{ //todo make a one or two motor alternate to intake class
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private IntakeMode intakeMode;
    private double motorPower;

    public enum IntakeMode {
        OFF,
        IN,
        OUT
    }

    public Intake (double motorPower) {
        leftMotor = FTCUtilities.getMotor("intakeL");
        rightMotor = FTCUtilities.getMotor("intakeR");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorPower = motorPower;
        intakeMode = IntakeMode.OFF;
    }

    public void execute(PlanElement planElement){
        if(planElement instanceof IntakeCommand){
            execute((IntakeCommand)planElement);
        } else if(planElement instanceof IntakeCommandWithTrigger){
            execute((IntakeCommandWithTrigger)planElement);
        } else{
            throw new Warning("Could not executePlan Planelement " + planElement.toString() + " in intake class");

        }
    }

    private void execute(IntakeCommand command){
        intakeMode = command.intakeMode;
        checkIntakeMode();
    }

    private void execute(IntakeCommandWithTrigger command){
        intakeMode = IntakeMode.IN;
        checkIntakeMode();
        IntakeMonitor intakeMonitor = new IntakeMonitor(command.trigger,this);
        Thread thread = new Thread(intakeMonitor);
        thread.run();
    }

    private void checkIntakeMode(){
        if(intakeMode == IntakeMode.IN){
            runMotors(motorPower);
        } else if(intakeMode == IntakeMode.OUT){
            runMotors(-motorPower);
        } else if (intakeMode == IntakeMode.OFF){
            stopMotors();
        }
    }

    public void runMotors(double motorPower){
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
    }
    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
