package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToSeperatePositions;

import java.util.HashMap;
import java.util.Map;

public class Elbows extends Subsystem {

    public static final Elbows INSTANCE = new Elbows();
    private Elbows(){}

    private Servo Elbow,Elbow2;

    public String name = "elbow";
    public String name2 = "elbow_2";

    public String state;

    public Command elbowUp(){
        return new InstantCommand(() ->{
            state = "elbowUp";
            Map<Servo, Double> servoPositions = new HashMap<>();
            servoPositions.put(Elbow, 0.45);
            servoPositions.put(Elbow2, 0.645);
            new MultipleServosToSeperatePositions(servoPositions, this);
            return null;
        });
    }

    public Command elbowDown(){
        return new InstantCommand(() -> {
            state = "elbowDown";
            Map<Servo, Double> servoPositions = new HashMap<>();
            servoPositions.put(Elbow, 0.188);
            servoPositions.put(Elbow2, 0.878);
            new MultipleServosToSeperatePositions(servoPositions, this);
            return null;
        });
    }

    public Command elbowStraight(){
        Map<Servo, Double> servoPositions = new HashMap<>();
        servoPositions.put(Elbow,0.5);
        servoPositions.put(Elbow2,0.45);
        return new MultipleServosToSeperatePositions(servoPositions, this);
    }
    public Command toggle(){
        if(state.equals("elbowUp")){
            return elbowDown();
        }else{
            return elbowUp();
        }
    }
    @Override
    public void periodic(){
        OpModeData.INSTANCE.telemetry.addData("Elbow", Elbow.getPosition());
        OpModeData.INSTANCE.telemetry.addData("Elbow2", Elbow2.getPosition());
    }



    @Override
    public void initialize(){
        Elbow = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        Elbow2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name2);
    }
}
