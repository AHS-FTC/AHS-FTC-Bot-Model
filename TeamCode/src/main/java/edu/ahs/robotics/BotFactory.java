package edu.ahs.robotics;

import android.view.SubMenu;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;
import java.util.Map;

public abstract class BotFactory {

    public abstract Robot createRobot();

}


