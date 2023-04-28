package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
@Disabled
public class LineTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence gooneInch = drive.trajectorySequenceBuilder(startPose).splineToConstantHeading(new Vector2d(startPose.getX(), startPose.getY() + 1), Math.toRadians(90)).build();



        waitForStart();


        drive.followTrajectorySequence(gooneInch);
        while(opModeIsActive()){

        }
    }
}
