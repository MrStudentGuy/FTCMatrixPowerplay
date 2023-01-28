package org.firstinspires.ftc.teamcode.Autonomous.TrajectoriySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequences {
    SampleMecanumDrive drive;
    Pose2d startPose;

    public TrajectorySequences(SampleMecanumDrive LocalDrive, Pose2d start){
        drive = LocalDrive;
        startPose = start;
    }

}
