package org.firstinspires.ftc.teamcode.roadrunner.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.Actions.Claw;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Actions.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.Actions.elbow;


@Config
@Autonomous(name = "ActualAuton", group = "Autonomous")
public class ActualAuton extends LinearOpMode {

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
                .strafeToLinearHeading(new Vector2d(22.35, 25.16), Math.toRadians(0.82));
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