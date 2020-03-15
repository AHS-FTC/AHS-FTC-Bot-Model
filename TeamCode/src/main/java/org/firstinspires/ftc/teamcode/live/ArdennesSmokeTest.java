package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.opmodes.bfr.IterativeOpMode16896;

@Autonomous
public class ArdennesSmokeTest extends IterativeOpMode16896 {

    private Ardennes ardennes;

    private enum State{

        TEST_LIMIT_SWITCHES("Limit Switches"),

        TEST_DISTANCE_SENSORS("Distance Sensors"),

        TEST_CAPSTONE_SERVO("Capstone Servo"),

        TEST_GRIPPER_SERVO("Gripper Servo"),

        TEST_Y_SERVO("Y Slide Servo"),

        TEST_FOUNDATION_SERVOS("Foundation Servos"),

        TEST_SLIDE_MOTORS("Slide Motors"),

        TEST_DRIVE_MOTORS("Drive Motors");

        String tag;

        State(String tag) {
            this.tag = "Testing " + tag;
        }

    }

    @Override
    protected void initialize() {
        ardennes = new Ardennes();
    }

    @Override
    protected void repeatAfterInit() {

    }

    @Override
    protected void begin() {

    }

    @Override
    protected void iterate() {

    }

    @Override
    protected void teardown() {

    }
}
