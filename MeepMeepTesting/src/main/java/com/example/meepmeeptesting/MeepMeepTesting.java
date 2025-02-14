package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Create a bot with constraints (maxVel, maxAccel, maxAngVel, maxAngAccel, track width)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        // Define the action for the bot: moving from (-36, -72) to (-48, -36) using a spline
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -66, Math.toRadians(270)))
                        //drop preloaded
                        .strafeToLinearHeading(new Vector2d(0, -31), Math.toRadians(270))
                        // get first sample
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(33, -38, Math.toRadians(40)), Math.toRadians(45))
                        // drop first sample
                        .strafeToLinearHeading(new Vector2d(38, -40), Math.toRadians(-45))
                        // get second sample
                        .turnTo(Math.toRadians(35))
                        // drop second sample
                        .strafeToLinearHeading(new Vector2d(47, -40), Math.toRadians(270))
                        // pick specimen 1
                        .strafeToConstantHeading(new Vector2d(47, -47.5))
                        //drop specimen 1
                        .strafeToLinearHeading(new Vector2d(-5, -29), Math.toRadians(270))
                        //pick specimen 2
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(47, -42, Math.toRadians(270)), Math.PI/9)
                        .strafeToConstantHeading(new Vector2d(47, -47.5))
                        //drop specimen 2
                        .strafeToLinearHeading(new Vector2d(-9, -29), Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(5, -28), new TranslationalVelConstraint(20.0))
                        .build());
        // Set the background and start the simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
