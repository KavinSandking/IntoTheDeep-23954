package org.firstinspires.ftc.teamcode.roadrunner.roadrunner.Actions.Autonomous;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.roadrunner.Actions.MecanumDrive;


@Config
@Disabled
@Autonomous(name = "4SampleSecondary", group = "Autonomous")
public class AutonSecondary extends LinearOpMode {

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

                    Slides.setTargetPosition(4050);
                    Slides2.setTargetPosition(4050);

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
                    Slides.setTargetPosition(80);
                    Slides2.setTargetPosition(80);

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
        public class elbowStraight implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    elbow.setPosition(0.5);
                    elbow2.setPosition(0.45);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 150){
                    return false;
                }
                return true;
            }
        }

        public Action elbowStraight() {
            return new elbowStraight();
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

                if (elapsedTime >= 550) {
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
                    elbow.setPosition(0.5);
                    elbow2.setPosition(0.6);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >=750 ) {
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
                    claw.setPosition(0.0);
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime >= 350) {
                    return false;
                }
                return true;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            private long startTime;
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    claw.setPosition(1.0);
                    initialized = true;
                }
                return false;
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
                .splineTo(new Vector2d(5.28, 21.87), Math.toRadians(136.04),new TranslationalVelConstraint(70));
        TrajectoryActionBuilder Sample2 = Sample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(21.76, 16.12), Math.toRadians(-0.28));
        TrajectoryActionBuilder DROP2 = Sample2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(5.89, 23.78), Math.toRadians(130.64),new TranslationalVelConstraint(70));
        TrajectoryActionBuilder Sample3 = DROP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(22.42, 24.76), Math.toRadians(-0.8));
        TrajectoryActionBuilder DROP3 = Sample3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(5.89, 23.78), Math.toRadians(130.64),new TranslationalVelConstraint(70));
        TrajectoryActionBuilder Sample4 = DROP3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(24.31, 25.82), Math.toRadians(27.46));
        TrajectoryActionBuilder DROP4 = Sample4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(5.89, 23.78), Math.toRadians(130.64),new TranslationalVelConstraint(70));
        TrajectoryActionBuilder goBack = DROP4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(17.69, 17.68), Math.toRadians(143.1));





        //TODO Set up Velocity Constraints for certain trajectories
        //TrajectoryActionBuilder Velocity Constraints = Poop.endTrajectory().fresh()
        //.splineTo(new Vector2d(69.0,69.0),Math.toRadians(69.0), new TranslationalVelConstraint(20.0));


        Action Sample_1 = Sample1.build();
        Action Sample_2 = Sample2.build();
        Action Drop2 = DROP2.build();
        Action Sample_3 = Sample3.build();
        Action Drop3 = DROP3.build();
        Action Sample_4  = Sample4.build();
        Action Drop4 = DROP4.build();
        Action goBack4 = goBack.build();


        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                elbow.elbowStraight(),// go to bucket and lift slides up
                                Sample_1,
                                Slides.slidesUp()

                        ),
                        // put the elbow up
                        new ParallelAction(
                                // hold the slides
                                Slides.holdSlides(),
                                elbow.elbowUp()

                        ),
                        // open claw and go back
                        claw.openClaw(),
                        new SleepAction(0.5),

                        elbow.elbowStraight(),
                        new ParallelAction(
                                // move the slides down and go to pick up sample 2
                                Slides.slidesDown(),
                                Sample_2
                        ),
                        elbow.lowerElbow(),
                        claw.closeClaw(),
                        elbow.elbowStraight(),
                        new ParallelAction(
                                Drop2,
                                Slides.slidesUp()
                        ),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp()


                        ),

                        claw.openClaw(),
                        new SleepAction(0.5),
                        elbow.elbowStraight(),
                        new ParallelAction(
                                Slides.slidesDown(),
                                Sample_3
                        ),
                        new SequentialAction(
                                elbow.lowerElbow(),
                                claw.closeClaw(),
                                elbow.elbowStraight()
                        ),
                        new ParallelAction(
                                Drop3,
                                Slides.slidesUp()
                        ),

                        claw.closeClaw(),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp()


                        ),

                        claw.openClaw(),
                        new SleepAction(0.5),

                        elbow.elbowStraight(),
                        new ParallelAction(
                                Slides.slidesDown(),
                                Sample_4
                        ),
                        new SequentialAction(
                                elbow.lowerElbow(),
                                claw.closeClaw(),
                                elbow.elbowStraight()
                        ),
                        new ParallelAction(
                                Drop4,
                                Slides.slidesUp()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                Slides.holdSlides(),
                                elbow.elbowUp()


                        ),

                        claw.openClaw(),
                        new SleepAction(0.5),

                        elbow.elbowStraight(),
                        goBack4

                ));

    }
}