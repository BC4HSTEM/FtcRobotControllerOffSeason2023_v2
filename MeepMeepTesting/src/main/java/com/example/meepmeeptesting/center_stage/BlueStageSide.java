package com.example.meepmeeptesting.center_stage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueStageSide {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(37.105101137333385, 30, 16.897379840537297, Math.toRadians(180), 13.35)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                // place purple pixel and park 
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12,37, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(0)))
                                //.turn(Math.toRadians(90))
                                //.forward(30)
                                .addDisplacementMarker(() -> {
                                    /* Everything in the marker callback should be commented out */

                                    // bot.shooter.shoot()
                                    // bot.wobbleArm.lower()
                                })
                                /*.turn(Math.toRadians(90))
                                .splineTo(new Vector2d(10, 15), 0)
                                .turn(Math.toRadians(90))*/
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}