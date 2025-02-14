package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Claw extends Subsystem {

    public static final Claw INSTANCE = new Claw();
    private Claw(){}

    public Servo Claw;
    public String name = "claw";

    public String state;

    public Command open(){
        state = "open";
        return new ServoToPosition(Claw,
                1.0,
                this);
    }
    public Command close(){
        state = "closed";
        return new ServoToPosition(Claw,
                0.0,
                this);
    }

    public Command toggle(){
        if(state.equals("open")){
            return close();
        }else{
            return open();
        }
    }
    @Override
    public void periodic(){
        OpModeData.INSTANCE.telemetry.addData("Claw", Claw.getPosition());
    }



    @Override
    public void initialize(){
        Claw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        Claw.close();
    }
}
