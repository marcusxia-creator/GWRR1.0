package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34, -66, 0))
                        .setTangent(90)
                        .splineToConstantHeading(new Vector2d(-52,-52),Math.toRadians(-90))
                        .waitSeconds(0.25)
                        .turn(Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(-48,-24),Math.toRadians(90))
                        .waitSeconds(0.25)
                        .splineToConstantHeading(new Vector2d(-52,-52),Math.toRadians(45))
                        .waitSeconds(0.25)
                        .splineToConstantHeading(new Vector2d(-59,-24),Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}