package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ParallelAction;

// Regular autonomous imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



@Config
@Autonomous(name = "LM3AUTONBLUE", group = "Autonomous")
public class Auton extends LinearOpMode {

    public class Slides {
        private DcMotorEx Slides;
        private DcMotorEx Slides2;

        public Slides(HardwareMap hardwareMap) {
            Slides = hardwareMap.get(DcMotorEx.class, "Slides");
            Slides2 = hardwareMap.get(DcMotorEx.class, "Slides2");

            Slides.setDirection(DcMotorSimple.Direction.REVERSE);
            Slides2.setDirection(DcMotorSimple.Direction.REVERSE);
            Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class slidesUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    Slides.setTargetPosition(4200);
                    Slides2.setTargetPosition(4200);

                    Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Slides.setPower(1.0);
                    Slides2.setPower(1.0);

                    initialized = true;
                }

                packet.put("Slide Encoder 1", Slides.getCurrentPosition());
                packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.addData("Slide Encoder 1", Slides.getCurrentPosition());
                telemetry.addData("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.update();

                if (!Slides.isBusy() && !Slides2.isBusy()) {
                    Slides.setPower(0);
                    Slides2.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public Action slidesUp() {
            return new slidesUp();
        }

        public class slidesDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Slides.setTargetPosition(10);
                    Slides2.setTargetPosition(10);

                    Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Slides.setPower(1.0);
                    Slides2.setPower(1.0);

                    initialized = true;
                }

                packet.put("Slide Encoder 1", Slides.getCurrentPosition());
                packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.addData("Slide Encoder 1", Slides.getCurrentPosition());
                telemetry.addData("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.update();

                if (!Slides.isBusy() && !Slides2.isBusy()) {
                    Slides.setPower(0);
                    Slides2.setPower(0);
                    return false;
                }
                return true;
            }
        }

    public class holdSlides implements Action {
        private boolean initialized = false;
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Slides.setPower(0.1); // Small power to hold position
                Slides2.setPower(0.1);
                startTime = System.currentTimeMillis(); // Record start time
                initialized = true;
            }

            long elapsedTime = System.currentTimeMillis() - startTime;

            packet.put("Slide Encoder 1", Slides.getCurrentPosition());
            packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
            packet.put("Hold Time (ms)", elapsedTime);

            // Run for 1 second (or adjust as needed for claw to open)
            return elapsedTime < 500;
        }
    }

    public Action holdSlides() {
        return new holdSlides();
    }


    public Action slidesDown() {
            return new slidesDown();
        }
    }

    public class elbow {
        private Servo elbow;
        private Servo elbow2;

        public elbow(HardwareMap hardwareMap) {
            elbow = hardwareMap.get(Servo.class, "elbow");
            elbow2 = hardwareMap.get(Servo.class, "elbow_2");
        }

        public class lowerElbow implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    elbow.setPosition(0.188);
                    elbow2.setPosition(0.878);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 500) {
                    return false;
                }
                return true;
            }
        }

        public Action lowerElbow() {
            return new lowerElbow();
        }

        public class elbowUp implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    elbow.setPosition(0.45);
                    elbow2.setPosition(0.645);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 1000) {
                    return false;
                }
                return true;
            }
        }

        public Action elbowUp() {
            return new elbowUp();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPosition(1.0);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 500) {
                    return false;
                }
                return true;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPosition(0.0);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 500) {
                    return false;
                }
                return true;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Slides Slides = new Slides(hardwareMap);
        elbow elbow = new elbow(hardwareMap);

        TrajectoryActionBuilder Sample1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(6.42, 23.72), Math.toRadians(134.13));

        TrajectoryActionBuilder GOBACK = Sample1.endTrajectory().fresh()
                .lineToX(11.541);
        TrajectoryActionBuilder Sample2 = GOBACK.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(24.14, 19.82), Math.toRadians(-5.61));
        TrajectoryActionBuilder Wait = GOBACK.endTrajectory().fresh()
                .waitSeconds(2.0);
        TrajectoryActionBuilder DROP2 = Sample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10.01, 18.55), Math.toRadians(129.53));
        TrajectoryActionBuilder MOVEFORWARD = DROP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(8.17, 24.46), Math.toRadians(130.17));
        TrajectoryActionBuilder GOBACK2 = DROP2.endTrajectory().fresh()
                .lineToX(11.541);
        TrajectoryActionBuilder Sample3 = GOBACK2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(24.49, 28.81), Math.toRadians(-7.09));
        TrajectoryActionBuilder DROP3 = Sample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10.01, 18.55), Math.toRadians(129.53));
        TrajectoryActionBuilder MOVEFORWARD3 = DROP3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(8.17, 24.46), Math.toRadians(130.17));
        TrajectoryActionBuilder GOBACK3 = MOVEFORWARD3.endTrajectory().fresh()
                .lineToX(11.541);
        TrajectoryActionBuilder Sample4 = GOBACK3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(26.46, 27.38), Math.toRadians(34.31));
        TrajectoryActionBuilder DROP4 = Sample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10.01, 18.55), Math.toRadians(129.53));
        TrajectoryActionBuilder MOVEFORWARD4 = DROP4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(8.17, 24.46), Math.toRadians(130.17));
        TrajectoryActionBuilder GOBACK4 = MOVEFORWARD3.endTrajectory().fresh()
                .lineToX(11.541);



        //TODO Set up Velocity Constraints for certain trajectories
        //TrajectoryActionBuilder Velocity Constraints = Poop.endTrajectory().fresh()
                //.splineTo(new Vector2d(69.0,69.0),Math.toRadians(69.0), new TranslationalVelConstraint(20.0));


        Action Sample_1 = Sample1.build();
        Action goBack = GOBACK.build();
        Action Sample_2 = Sample2.build();
        Action WAIT = Wait.build();
        Action Drop2 = DROP2.build();
        Action MoveForward = MOVEFORWARD.build();
        Action goBack2 = GOBACK2.build();
        Action Sample_3 = Sample3.build();
        Action Drop3 = DROP3.build();
        Action MoveForward3 = MOVEFORWARD3.build();
        Action goBack3 = GOBACK3.build();
        Action Sample_4  = Sample4.build();
        Action Drop4 = DROP4.build();
        Action goBack4 = GOBACK4.build();
        Action MoveForward4 = MOVEFORWARD4.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                Sample_1,
                                Slides.slidesUp()

                        ),

                        elbow.elbowUp(),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp()
                        ),

                        claw.openClaw(),
                        goBack,
                        new ParallelAction(
                                Slides.slidesDown(),
                                Sample_2
                        ),
                        WAIT,
                        new ParallelAction(
                                elbow.lowerElbow(),
                                claw.openClaw()
                        ),

                        claw.closeClaw(),
                        elbow.elbowUp(),
                        new SequentialAction(
                                Drop2,
                                Slides.slidesUp(),
                                MoveForward,
                                claw.closeClaw()
                        ),

                        //elbow.elbowUp(),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp(),
                                claw.openClaw()
                        ),
                        goBack2,
                        new ParallelAction(
                                Slides.slidesDown(),
                                Sample_3
                        ),
                        new SequentialAction(
                                elbow.lowerElbow(),
                                claw.openClaw(),
                                claw.closeClaw(),
                                elbow.elbowUp(),
                                Drop3,
                                Slides.slidesUp(),
                                MoveForward3,
                                claw.closeClaw()
                        ),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp(),
                                claw.openClaw()
                        ),
                        goBack3,
                        new ParallelAction(
                                Slides.slidesDown(),
                                Sample_4
                        ),
                        new SequentialAction(
                                elbow.lowerElbow(),
                                claw.openClaw(),
                                claw.closeClaw(),
                                elbow.elbowUp(),
                                Drop4,
                                Slides.slidesUp(),
                                MoveForward4,
                                claw.closeClaw()
                        ),
                        new ParallelAction(
                            Slides.holdSlides(),
                            elbow.elbowUp(),
                            claw.openClaw()
                        ),
                        goBack4




                ));

    }
}
