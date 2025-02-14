package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Claw;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Elbows;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Slides;

@Autonomous(name = "Efcgvhj", group = "Examples")
public class testAuto extends PedroOpMode {

    public testAuto(){
        super(Slides.INSTANCE, Claw.INSTANCE, Elbows.INSTANCE);
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(8, 88, Math.toRadians(0));

    private final Pose scorePose = new Pose(13, 130, Math.toRadians(135));
    private final Pose pickUpSample2 = new Pose(130, 130, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(13,130, Math.toRadians(135));

    private PathBuilder scorePreload, sample2, scoreSample2;
    

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(
                        // go to highBasket
                        new BezierLine(
                                new Point(startPose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        sample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickUpSample2)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpSample2.getHeading());
        scoreSample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickUpSample2),
                                new Point(scorePose2)
                        )
                )
                .setLinearHeadingInterpolation(pickUpSample2.getHeading(), scorePose2.getHeading());


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void onUpdate() {

        // These loop the movements of the robot
        follower.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void waitForStart(){}

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        CommandManager.INSTANCE.scheduleCommand(new SequentialGroup(
                new ParallelGroup(
                        Slides.INSTANCE.slidesUp(),
                        new FollowPath(scorePreload.build(),true)
                ),
                Elbows.INSTANCE.elbowUp(),
                Claw.INSTANCE.open(),
                new Delay(0.5),
                Elbows.INSTANCE.elbowStraight(),
                new ParallelGroup(
                        new FollowPath(sample2.build(),true),
                        Slides.INSTANCE.slidesDown()
                ),
                Elbows.INSTANCE.elbowDown(),
                Claw.INSTANCE.close(),
                Elbows.INSTANCE.elbowUp(),
                new ParallelGroup(
                        new FollowPath(scoreSample2.build(),true),
                        Slides.INSTANCE.slidesUp()
                )


        ));
    }

}

