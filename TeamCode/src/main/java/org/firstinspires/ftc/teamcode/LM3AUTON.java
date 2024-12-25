package org.firstinspires.ftc.teamcode;

//roadrunner necessary imports
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ParallelAction;

//regular autonomous imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous (name = "LM3AUTON", group = "Autonomous")
public class LM3AUTON extends LinearOpMode {
    DcMotorEx Intake;
    Servo Claw;

    //Initiating Slides
    public class Intake {
        private DcMotorEx Intake;

        public Intake(HardwareMap hardwareMap) {
            Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    //Initiating Claw
    public class Claw {
        private Servo Claw;

        public Claw(HardwareMap hardwareMap) {
            Claw = hardwareMap.get(Servo.class, "claw");
        }
    }

    public class slidesUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Intake.setPower(0.8);
                initialized = true;
            }
            //encoder read position for the slides
            double pos = Intake.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 3000.0) {
                return true;
            } else {
                Intake.setPower(0);
                return false;
            }
        }
    }

    public Action slidesUp() {
        return new slidesUp();
    }

    public class openClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(0.0);
            return false;
        }
    }

    public Action openClaw() {
        return new openClaw();
    }

    public class closeClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition((1.0));
            return false;
        }
    }

    public Action closeClaw() {
        return new closeClaw();
    }

    public class slidesDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Intake.setPower(-0.8);
                initialized = true;
            }

            double pos = Intake.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                Intake.setPower(0);
                return false;
            }
        }
    }

    public Action slidesDown() {
        return new slidesDown();
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-36.0, -72.0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw Claw = new Claw(hardwareMap);
        Intake Intake = new Intake(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-48.0,-36.0), Math.toRadians(0))
                .turn(175.0)
                .lineToY(-60.0);

        Actions.runBlocking(closeClaw());

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(new SequentialAction(
                trajectoryActionChosen,
                slidesUp()
        ));


    }
    }
