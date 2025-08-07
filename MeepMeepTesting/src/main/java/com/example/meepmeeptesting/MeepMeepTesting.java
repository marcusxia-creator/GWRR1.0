package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double x_pick = 18;
        double y_pick = -48;
        double y_hook = -35;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(72, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -60, -Math.PI/2))
                .strafeToLinearHeading(new Vector2d(0,-33),-Math.PI/2)
                .strafeToLinearHeading(new Vector2d(20,-40),0)
                //.splineToLinearHeading(new Pose2d(20,-60,Math.toRadians(0)),0)
                .splineTo(new Vector2d(40,-40),Math.PI/4)
                //pick 1
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.25)
                //pick 2
                .strafeToLinearHeading(new Vector2d(48,-40),Math.PI/3)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                //pick 2hook
                .splineToLinearHeading(new Pose2d(x_pick,y_pick,Math.toRadians(-30)),90)
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(0,y_hook,Math.toRadians(-90)),-90)
                .waitSeconds(0.5)
                //pick 3hook
                .splineToLinearHeading(new Pose2d(x_pick,y_pick,Math.toRadians(-30)),90)
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(0,y_hook,Math.toRadians(-90)),-90)
                .waitSeconds(0.5)
                //pick 4hook
                .splineToLinearHeading(new Pose2d(x_pick,y_pick,Math.toRadians(-30)),90)
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(0,y_hook,Math.toRadians(-90)),-90)
                .waitSeconds(0.5)
                //park
                .splineToLinearHeading(new Pose2d(50,-55,Math.toRadians(0)),0)
                //.strafeToLinearHeading(new Vector2d(60,-60),Math.PI/2)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}