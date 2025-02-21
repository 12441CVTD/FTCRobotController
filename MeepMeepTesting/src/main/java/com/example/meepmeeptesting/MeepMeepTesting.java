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
                .strafeTo(new Vector2d(0, -32.6))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(0, -34.3))
                .strafeTo(new Vector2d(50, -38))
                .strafeTo(new Vector2d(50, -36.5))
                .waitSeconds(0.6)
                .strafeToLinearHeading(new Vector2d(48, -47), Math.toRadians(270.0000000001))
                .waitSeconds(1)
                .strafeTo(new Vector2d(48, -10))
                .strafeTo(new Vector2d(60, -10))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(58, -50.2))
                .strafeTo(new Vector2d(58, -49))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(3, -38.5), Math.toRadians(89.9))
                .waitSeconds(1.7)
                .strafeTo(new Vector2d(3, -33))
                .waitSeconds(5)
                .strafeTo(new Vector2d(3, -36))
                //pause?
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}