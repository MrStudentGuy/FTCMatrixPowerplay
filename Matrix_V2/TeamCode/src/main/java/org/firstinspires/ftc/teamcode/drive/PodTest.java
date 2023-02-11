package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class PodTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();

        DcMotorEx encoderLeft = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx encoderRight = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx encoderCenter = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.setMsTransmissionInterval(20);


        waitForStart();



        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("Left: ", encoderLeft.getCurrentPosition());
            telemetry.addData("Right: ", encoderRight.getCurrentPosition());
            telemetry.addData("Center: ", encoderCenter.getCurrentPosition());
            telemetry.update();

        }
    }
}
