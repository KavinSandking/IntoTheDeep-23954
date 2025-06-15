package org.firstinspires.ftc.teamcode.roadrunner.NextFTC.OpModes.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

@Config
public class Slides extends Subsystem {

    public static final Slides INSTANCE = new Slides();
    private Slides() {}

    public MotorEx leftSlide;
    public MotorEx rightSlide;
    public MotorGroup slides;
    public static double kp = 0.01 , ki, kd, kf=0.08;
    public static double target = 0;
    public PIDFController controller = new PIDFController(kp, 0, 0, (v) -> kf);

    public String leftSlideName = "Slides";
    public String rightSlideName = "Slides2";
    double slidesDownPosition = 0.0;
    double slidesUpPosition = 4100;

    public Command slidesDown(){
        return new RunToPosition(slides,
                slidesDownPosition,
                controller,
                this);
    }
    public Command slidesUp(){
        return new RunToPosition(slides,
                slidesUpPosition,
                controller,
                this);
    }
    public Command manualControl(float power){
        return new SetPower(slides, power, this);
    }
    @Override
    public void periodic(){
        controller.setKP(kp);

        OpModeData.telemetry.addData("currentPos", slides.getLeader().getCurrentPosition());
        OpModeData.telemetry.addData("target", target);
        OpModeData.telemetry.update();



    }

    @Override
    @NonNull
    public Command getDefaultCommand(){
        return new HoldPosition(slides,
                controller,
                this);
    }

    @Override
    public void initialize(){
        leftSlide = new MotorEx(leftSlideName)
                .reverse();
        rightSlide = new MotorEx(rightSlideName)
                .reverse();
        slides = new MotorGroup(leftSlide, rightSlide);
        leftSlide.resetEncoder();
    }

}
