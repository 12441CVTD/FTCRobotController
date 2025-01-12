package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-20, -60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225)) // Place
                //pause?
                        .strafeToLinearHeading(new Vector2d(-48, -38), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(-57, -38), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(-58, -34), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(-54,-54), Math.toRadians(225))
                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}