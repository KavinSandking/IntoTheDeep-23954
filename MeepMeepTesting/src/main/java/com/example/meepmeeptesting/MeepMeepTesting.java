package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -72, 0))
                .splineTo(new Vector2d(-48, -34), Math.toRadians(90)) // Initial movement
                .strafeToLinearHeading(new Vector2d(-56,-45),Math.toRadians(230))
                .waitSeconds(2)
                .turn(Math.toRadians(-130))  // Second turn
                .lineToY(-36)  // Move to Y = -36
                .strafeToLinearHeading(new Vector2d(-56,-45),Math.toRadians(225))// Third turn// Another move to Y = -36
                .strafeToLinearHeading(new Vector2d(-54, -25), Math.PI)  // Strafe to a new position
                .strafeToLinearHeading(new Vector2d(-60, -25), Math.PI)
                .waitSeconds(1)// Strafe to new position
                .strafeToLinearHeading(new Vector2d(-54, -25), Math.PI)  // Strafe back
                .strafeToLinearHeading(new Vector2d(-55, -45), Math.toRadians(230)) // Strafe to next position
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30, 0),0)  // Final position move
                .build());
        // Set the background and start the simulation
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
