package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.slides;

@Mercurial.Attach
@slides.Attach
@Intake.Attach
@drivetrain.Attach
@FeatureRegistrar.LogDependencyResolutionExceptions
@TeleOp(name = "teleop_commands")
public class teleop extends OpMode {

    private static final int HIGHBASKET = 4200;
    private static final int HOME = 100;

    private slides Slides;
    private Intake Elbows;
    private Intake Claw;
    private drivetrain Drivetrain;
    private BoundGamepad gamepad2;
    private Lambda driveCommand;
    @Override
    public void init() {
        gamepad2 = Mercurial.gamepad2();
        Slides = slides.INSTANCE; // Get the instance!
        Elbows = Intake.INSTANCE; // Get the instance!
        Claw = Intake.INSTANCE; // Get the instance!
        Drivetrain = drivetrain.INSTANCE; // Get the instance!

        driveCommand = Drivetrain.controlDrive(gamepad1);

        // Move slides to target positions
        gamepad2.dpadUp().onTrue(new Lambda("set-high").addRequirements(Slides).setExecute(() -> Slides.setTargetPosition(slides.HIGHBASKET))); // Correct call!
        gamepad2.dpadDown().onTrue(new Lambda("set-home").addRequirements(Slides).setExecute(() -> Slides.setTargetPosition(slides.HOME))); // Correct call!

        gamepad2.b().toggleTrue(Elbows.setPositionCommand(0.45, 0.645));  // Elbow position 1 on button press
        gamepad2.b().toggleFalse(Elbows.setPositionCommand(0.188, 0.878));
        gamepad2.y().onTrue(dropHighBasket());
        gamepad2.a().onTrue(pickUpSample());
        gamepad2.x().toggleTrue(Claw.closeClaw());
        gamepad2.x().toggleFalse(Claw.openClaw());



    }

    @Override
    public void loop() {
        driveCommand.execute();
        telemetry.addData("SlidesPos", Slides.CurrentPosition());
        telemetry.addData("elbow1Pos", Elbows.getElbowPosition());
        telemetry.addData("elbow2Pos", Elbows.getElbow2Position());
        telemetry.update();
    }
}
