package org.firstinspires.ftc.teamcode.roadrunner.NextFTC.OpModes.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Elbows extends Subsystem {

    public static final Elbows INSTANCE = new Elbows();
    private Elbows() {}

    public Servo rightElbow;

    public String elbow_2 = "elbow_2";

    double rightElbowDownPosition = 0.675;
    double rightElbowUpPosition = 1.0;

    public Command elbowDown(){
        return new ServoToPosition(rightElbow,
                rightElbowUpPosition,
                this);
    }

    public Command elbowUp(){
        return new ServoToPosition(rightElbow,
                rightElbowDownPosition,
                this);
    }

    @Override
    public void initialize(){
        rightElbow = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, elbow_2);
    }
}
