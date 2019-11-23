package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.hardware.Executor;
import edu.ahs.robotics.hardware.Intake;

public class IntakeCommand extends PlanElement {
    public Intake.IntakeMode intakeMode;

    public IntakeCommand(Executor executor, Intake.IntakeMode intakeMode) {
        super(executor);
        this.intakeMode = intakeMode;
    }
}
