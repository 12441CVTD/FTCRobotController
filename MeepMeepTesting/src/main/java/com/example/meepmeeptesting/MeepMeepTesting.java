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
                .strafeTo(new Vector2d(0, -33.5))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(0, -37))
                .strafeTo(new Vector2d(48.5, -36))
                .waitSeconds(0.125)
                .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270.0000000001))
                .strafeTo(new Vector2d(48, -10))
                .strafeTo(new Vector2d(60, -10))
                .strafeTo(new Vector2d(58, -48.2))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(3, -38), Math.toRadians(89.9))
                .strafeTo(new Vector2d(3, -33.5))
                .strafeTo(new Vector2d(3, -38))
                .waitSeconds(5)
                //pause?
                .strafeTo(new Vector2d(0, -40))
                .strafeToLinearHeading(new Vector2d(53, -24), Math.toRadians(-0))
                .strafeToLinearHeading(new Vector2d(50, -48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0, -38), Math.toRadians(89.7))
                .strafeTo(new Vector2d(0, -33))
                //pause?
                .strafeToLinearHeading(new Vector2d(50, -55), Math.toRadians(270))
                .strafeTo(new Vector2d(50, -58))
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90))
                //pause?
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}