package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import static com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleMotorsToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

@Config
public class Slides extends Subsystem {
    // BOILERPLATE
    public static final Slides INSTANCE = new Slides();
    private Slides() { }

    // USER CODE
    public MotorEx Slides;
    public MotorEx Slides2;
    public static double Kf;
    public static double kp = 0.0015, ki = 0, kd = 0;


    public PIDFController controller = new PIDFController(new PIDCoefficients(kp,ki,kd), v -> Kf);


    public String name = "Slides";
    public String name2 = "Slides2";
    public Command slidesUp() {
        return new ParallelGroup(
                new RunToPosition(Slides,
                        4200,
                        controller,
                        this),
                new RunToPosition(Slides2,
                        4200,
                        controller,
                        this)
        );
    }

    public Command slidesDown(){
        return new ParallelGroup(
                new RunToPosition(Slides,
                        100,
                        controller,
                        this),
                new RunToPosition(Slides2,
                        100,
                        controller,
                        this)
        );
    }

    public Command manualControl(float power){
        return new ParallelGroup(
                new SetPower(Slides, power, this),
                new SetPower(Slides2, power, this));
    }





    @Override
    @NonNull
    public Command getDefaultCommand() {
        return new ParallelGroup(
                new HoldPosition(Slides,controller,this),
                new HoldPosition(Slides2,controller,this)
        );
    }
    @Override
    public void periodic(){
        telemetry.addData("Slides", Slides.getCurrentPosition());
        telemetry.addData("Slides2", Slides2.getCurrentPosition());

    }
    @Override
    public void initialize() {
        Slides = new MotorEx(name);
        Slides2 = new MotorEx(name2);


    }
}
