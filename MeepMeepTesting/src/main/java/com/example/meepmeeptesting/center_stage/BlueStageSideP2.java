package com.example.meepmeeptesting.center_stage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueStageSideP2 {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40.06441304308421, 30, 18.580384897314797, Math.toRadians(180), 13.81)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                // place purple pixel and park
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(12,37, Math.toRadians(270)))
                                // Drop Purple Pixel on the spike mark here
                                .addDisplacementMarker(() -> {
                                    // Drop Pixel based on object
                                    // Collapse the grabber wrist to arm
                                })
                                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(0)))
                                // Park in the backstage
                                .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(0)))

                                // Park in the backstage
                                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)))
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