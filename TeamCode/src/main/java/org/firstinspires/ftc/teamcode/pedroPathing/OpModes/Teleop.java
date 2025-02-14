package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;


import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Claw;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Elbows;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Slides;


@TeleOp(name = "NextFTC Program Java")
class Teleop extends NextFTCOpMode {
    public Teleop() {

        super(Slides.INSTANCE, Claw.INSTANCE, Elbows.INSTANCE);
    }

    public String leftFrontName = "leftFront";
    public String leftBackName = "leftBack";
    public String rightFrontName = "rightFront";
    public String rightBackName = "rightBack";

    public MotorEx frontLeft;
    public MotorEx backLeft;
    public MotorEx frontRight;
    public MotorEx backRight;

    public MotorEx[] driveTrain;
    public Command driverControlled;

    public Command dropSample() {
        return new SequentialGroup(
                Slides.INSTANCE.slidesUp(),
                Elbows.INSTANCE.elbowUp(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                Elbows.INSTANCE.elbowStraight()
        );
    }

    @Override
    public void onInit() {
        frontLeft = new MotorEx(leftFrontName);
        backLeft = new MotorEx(leftBackName);
        frontRight = new MotorEx(rightFrontName);
        backRight = new MotorEx(rightBackName);


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveTrain = new MotorEx[]{frontLeft, backLeft, frontRight, backRight};
    }
    @Override
    public void onUpdate(){
        Slides.INSTANCE.manualControl(gamepadManager.getGamepad2().getLeftStick().getY());
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(driveTrain, gamepadManager.getGamepad1());
        driverControlled.invoke();
        registerGamepad2Controls();
    }

    public void registerGamepad2Controls() {
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(Slides.INSTANCE::slidesUp);
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(Slides.INSTANCE::slidesDown);

        gamepadManager.getGamepad2().getX().setPressedCommand(Claw.INSTANCE::toggle);
        gamepadManager.getGamepad2().getB().setPressedCommand(Elbows.INSTANCE::toggle);

        gamepadManager.getGamepad2().getY().setPressedCommand(this::dropSample);


    }

}
