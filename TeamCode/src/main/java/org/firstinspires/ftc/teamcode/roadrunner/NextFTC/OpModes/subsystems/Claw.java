package org.firstinspires.ftc.teamcode.roadrunner.NextFTC.OpModes.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Claw extends Subsystem {
    public static final Claw INSTANCE = new Claw();
    private Claw() {}

    public Servo clawServo;
    public String name = "claw";
    double clawOpenPosition = 0.0;
    double clawClosePosition = 1.0;

    public Command openClawCommand(){
        return new ServoToPosition(clawServo,
                clawOpenPosition,
                this);
    }

    public Command closeClawCommand(){
        return new ServoToPosition(clawServo,
                clawClosePosition,
                this);
    }

    @Override
    public void initialize(){
        clawServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class,name);
    }

}
