package org.firstinspires.ftc.teamcode.roadrunner.NextFTC.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.PassiveConditionalCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.DifferentialTankDriverControlled;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

import org.firstinspires.ftc.teamcode.roadrunner.NextFTC.OpModes.subsystems.*;

@TeleOp(name="teleop")
public class teleop extends NextFTCOpMode {

    public teleop() {
        super(Claw.INSTANCE, Elbows.INSTANCE, Slides.INSTANCE);
    }

    public String frontLeftName = "leftFront";
    public String frontRightName = "rightFront";
    public String backLeftName = "leftBack";
    public String backRightName = "rightBack";

    public MotorEx leftFront;
    public MotorEx rightFront;
    public MotorEx leftBack;
    public MotorEx rightBack;
    public Command driverControlled;
    public MotorEx[] motors;

    boolean isClawOpen = false;
    boolean isElbowUp = false;

    @Override
    public void onInit() {
        leftFront = new MotorEx(frontLeftName);
        leftBack = new MotorEx(backLeftName);
        rightBack = new MotorEx(backRightName);
        rightFront = new MotorEx(frontRightName);

        // Change the motor directions to suit your robot.
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = new MotorEx[] {leftFront, leftBack, rightFront, rightBack};


    }

    @Override
    public void onStartButtonPressed(){
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();

        setGamePad2Commands();
    }
    public void setGamePad2Commands(){
        gamepadManager.getGamepad2().getX().setPressedCommand(() ->
                new SequentialGroup(
                        new InstantCommand(() -> {
                            isClawOpen = !isClawOpen;
                        }),
                        new PassiveConditionalCommand(
                                () -> isClawOpen,
                                () -> Claw.INSTANCE.openClawCommand(),
                                () -> Claw.INSTANCE.closeClawCommand()
                        )
                )
        );

        gamepadManager.getGamepad2().getB().setPressedCommand(() ->
                new SequentialGroup(
                        new InstantCommand(() -> {
                            isElbowUp = !isElbowUp;
                        }),
                        new PassiveConditionalCommand(
                                () -> isElbowUp,
                                () -> Elbows.INSTANCE.elbowUp(),
                                () -> Elbows.INSTANCE.elbowDown()
                        )

                ));

        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(Slides.INSTANCE::slidesUp);
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(Slides.INSTANCE::slidesDown);

        gamepadManager.getGamepad2().getA().setPressedCommand(() ->
                new SequentialGroup(
                        Slides.INSTANCE.slidesUp(),
                        Elbows.INSTANCE.elbowUp(),
                        Claw.INSTANCE.openClawCommand()
                )
        );



    }
    @Override
    public void onUpdate(){}


}
