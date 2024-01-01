package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(270)))
                                .strafeRight(10)
                                .back(5)
                                .turn(Math.toRadians(180))
                                .forward(23.5)
                            .strafeRight(10)
                                .back(12)
                                .lineToLinearHeading(new Pose2d(-38, -55.25, Math.toRadians(180)))
                                .back(50)

                                .lineToLinearHeading(new Pose2d(50, -31.5
                                        , Math.toRadians(180)))
                                .forward(1.5)
                                .strafeLeft(7.5)
                                .strafeRight(9)
                                .forward(5)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
