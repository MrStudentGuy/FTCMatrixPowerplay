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

        Pose2d startPose = new Pose2d(-41.5, -63, Math.toRadians(90));
        final double MAX_SPEED_AUTO = 53.9;
        final double MAX_ANG_VEL = Math.toRadians(307), MAX_ANG_ACCEL = Math.toRadians(180), TRACK_WIDTH = 10.5, MAX_ACCEL = 50.46434527240892;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50.46434527240892, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToConstantHeading(new Vector2d(-38, -63))
                                .splineTo(new Vector2d(-36, -60), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(-36, -20))
                                .splineTo(new Vector2d(-32, -10), Math.toRadians(45))
                                .waitSeconds(1)
                                .turn(Math.toRadians(135))

                                .splineTo(new Vector2d(-56, -12), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}