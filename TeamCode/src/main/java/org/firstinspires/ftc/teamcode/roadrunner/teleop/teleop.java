package org.firstinspires.ftc.teamcode.roadrunner.teleop;

import static org.firstinspires.ftc.teamcode.roadrunner.subsystems.CommandGroups.dropHighBasket;
import static org.firstinspires.ftc.teamcode.roadrunner.subsystems.CommandGroups.pickUpSample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;

import org.firstinspires.ftc.teamcode.roadrunner.subsystems.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.roadrunner.subsystems.slides;

@Mercurial.Attach
@slides.Attach
@Intake.Attach
@drivetrain.Attach
@FeatureRegistrar.LogDependencyResolutionExceptions
@Disabled
@TeleOp(name = "teleop_commands")
public class teleop extends OpMode {
    private slides Slides;
    private Intake Elbows;
    private Intake Claw;
    private drivetrain Drivetrain;
    private BoundGamepad gamepad1;
    private BoundGamepad gamepad2;

    @Override
    public void init() {
        gamepad2 = Mercurial.gamepad2();
        gamepad1 = Mercurial.gamepad1();

        Slides = slides.INSTANCE;
        Elbows = Intake.INSTANCE;
        Claw = Intake.INSTANCE;
        Drivetrain = drivetrain.INSTANCE;

        Drivetrain.INSTANCE.setDefaultCommand(Drivetrain.robotCentricDriveCommand());
        gamepad1.rightBumper().onTrue(Drivetrain.strafeRight());
        gamepad1.leftBumper().onTrue(Drivetrain.strafeLeft());

        gamepad2.dpadUp().onTrue(new Lambda("set-high")
                .addRequirements(Slides)
                .setExecute(() -> Slides.setTargetPosition(slides.HIGHBASKET))); // Correct call!
        gamepad2.dpadDown().onTrue(new Lambda("set-home")
                .addRequirements(Slides)
                .setExecute(() -> Slides.setTargetPosition(Slides.HOME))); // Correct call!

        gamepad2.b().toggleTrue(Elbows.setPositionCommand(0.45, 0.645));  // Elbow position 1 on button press
        gamepad2.b().toggleFalse(Elbows.setPositionCommand(0.188, 0.878));

        gamepad2.x().toggleTrue(Claw.closeClaw());
        gamepad2.x().toggleFalse(Claw.openClaw());

        gamepad2.y().onTrue(dropHighBasket());
        gamepad2.a().onTrue(pickUpSample());



    }

    @Override
    public void loop() {
        Slides.INSTANCE.setManualPowerCommand().schedule();
        telemetry.addData("SlidesPos", Slides.CurrentPosition());
        telemetry.addData("elbow1Pos", Elbows.getElbowPosition());
        telemetry.addData("elbow2Pos", Elbows.getElbow2Position());
        telemetry.update();
    }
}
