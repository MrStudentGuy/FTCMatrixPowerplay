package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


        final double MAX_SPEED_AUTO = 53.9;
        final double MAX_ANG_VEL = Math.toRadians(307), MAX_ANG_ACCEL = Math.toRadians(180), TRACK_WIDTH = 10.5, MAX_ACCEL = 2;

        final Pose2d droppingPosition0 = new Pose2d(39.2, -12.8, Math.toRadians(0));
        final Pose2d droppingPosition = new Pose2d(39.2, -12.00, Math.toRadians(0));
        final Pose2d pickingPosition = new Pose2d(49, -12, Math.toRadians(0));

        Pose2d startPose = new Pose2d(32, -63.3, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53, 40, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(13.5, 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(droppingPosition0)
                                        .build()
//                        drive.trajectorySequenceBuilder(new Pose2d(-32.6, -64.5, Math.toRadians(180.00)))
//                                .splineTo(new Vector2d(-35.77, -47.41), Math.toRadians(74.26))
//                                .splineTo(new Vector2d(-35, -12), Math.toRadians(90))
//                                .lineToConstantHeading(new Vector2d(-34.7, -62.05))
//                                .lineToLinearHeading(new Pose2d(-35, -12,Math.toRadians(44)))
//                                .waitSeconds(1)
//
//                                .lineToSplineHeading(new Pose2d(-50, -12.11, Math.toRadians(180)))
////                                .splineTo(new Vector2d(-55.80, -12.11), Math.toRadians(180))
//                                .waitSeconds(1.5)
//                                .lineToLinearHeading(new Pose2d(-35, -12.7 ,Math.toRadians(45)))
//
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-50, -12.11, Math.toRadians(180)))
////                                .splineTo(new Vector2d(-55.80, -12.11), Math.toRadians(180))
//                                .waitSeconds(1.5)
//                                .lineToLinearHeading(new Pose2d(-35, -12.7 ,Math.toRadians(45)))
//
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-50, -12.11, Math.toRadians(180)))
////                                .splineTo(new Vector2d(-55.80, -12.11), Math.toRadians(180))
//                                .waitSeconds(1.5)
//                                .lineToLinearHeading(new Pose2d(-35, -12.7 ,Math.toRadians(45)))
//
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-50, -12.11, Math.toRadians(180)))
////                                .splineTo(new Vector2d(-55.80, -12.11), Math.toRadians(180))
//                                .waitSeconds(1.5)
//                                .lineToLinearHeading(new Pose2d(-35, -12.7 ,Math.toRadians(45)))
//
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(-50, -12.11, Math.toRadians(180)))
////                                .splineTo(new Vector2d(-55.80, -12.11), Math.toRadians(180))
//                                .waitSeconds(1.5)//                                .lineToLinearHeading(new Pose2d(-35, -12.7 ,Math.toRadians(45)))
//                                .splineToConstantHeading(new Vector2d(-34.85, -54), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-36, -12), Math.toRadians(90))
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}