package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;


@TeleOp
@Config
public class drivePIDTest extends LinearOpMode {

    double previousHeading = 0;
    double integratedHeading = 0;
    PIDController xController;
    PIDController yController;
    PIDController hController;


    public static double Kp_x = 0.11;
    public static double Ki_x = 0.01;
    public static double Kd_x = 0.01;

    public static double Kp_y = 0.13;
    public static double Ki_y = 0.01;
    public static double Kd_y = 0.01;

    public static double Kp_h = 0.04;
    public static double Ki_h = 0.1;
    public static double Kd_h = 0.001;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double hTarget = 0;


    Pose2d pose;
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        xController = new PIDController(Kp_x, Ki_x, Kd_x);
        yController = new PIDController(Kp_y, Ki_y, Kd_y);
        hController = new PIDController(Kp_h, Ki_h, Kd_h);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
//
//        Pose2d startPose = new Pose2d(-32, -63.3, Math.toRadians(180));
//        drive.setPoseEstimate(startPose);




        waitForStart();




        while(opModeIsActive()){


            xController.setPID(Kp_x, Ki_x, Kd_x);
            yController.setPID(Kp_y, Ki_y, Kd_y);
            hController.setPID(Kp_h, Ki_h, Kd_h);

            pose = drive.getPoseEstimate();
            double heading =  getIntegratedHeading(pose);

            double xPID = xController.calculate(pose.getX(), xTarget);
            double yPID = yController.calculate(pose.getY(), yTarget);
            double hPID = hController.calculate(heading, hTarget);

            Pose2d power = new Pose2d(xPID, yPID, hPID);
            drive.setWeightedDrivePower(power);
            drive.update();


            telemetry.addData("Heading Power: ", hPID);
            telemetry.addData("Integ Heading: ", heading);
            telemetry.update();
        }
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public double getIntegratedHeading(Pose2d poseEstimate){
        double currentHeading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
        double deltaHeading = currentHeading - previousHeading;

        if(deltaHeading < -180){
            deltaHeading+=360;
        }
        else if(deltaHeading >= 180){
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;

        previousHeading = currentHeading;

        return integratedHeading;
    }


}
