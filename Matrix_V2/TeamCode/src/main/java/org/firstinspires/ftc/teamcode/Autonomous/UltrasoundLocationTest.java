package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Autonomous
public class UltrasoundLocationTest extends LinearOpMode {

    double kp_wall = 0.04;
    double kd_wall = 0.01;
    double ki_wall = 0;
    double prevError_wall = 0;

    double kp_gyro = 0.04;
    double kd_gyro = 0.01;
    double ki_gyro = 0;
    double prevError_gyro = 0;

    double kp_pole = 0.04;
    double kd_pole = 0.01;
    double ki_pole = 0;
    double prevError_pole = 0;




    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Sensors sensors = new Sensors(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();


        while(opModeIsActive()){
            double power = wallSensorPID(Sensors.WallSensor.getDistanceCM(), 19);
            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
            double headingPower = gyroPID(heading, 0);

            double distance = Sensors.PoleSensor.getDistanceCM();
            double polePower = 0;

            if(Math.abs(power) < 0.09){
                power = 0;
            }

            if(Math.abs(headingPower) < 0.09){
                headingPower = 0;
            }

            if(Math.abs(headingPower) == 0 && Math.abs(power) == 0) {
                polePower = polePID(distance, 12.5);
            }
            else{
                polePower = 0;
            }
            drive.setWeightedDrivePower(new Pose2d(power, polePower, -headingPower));

            drive.update();

            telemetry.addData("Pole is at: ", distance + "CM");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading normalized", heading);
//            telemetry.update();

            Sensors.WallSensor.printDistance();
//            telemetry.addData("Distance: ", Sensors.WallSensor.getDistanceCM());
            telemetry.update();
        }
    }

    public double wallSensorPID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_wall;
        double Ierror = error + prevError_wall;

        prevError_wall = error;
        return pError * kp_wall + dError * kd_wall + Ierror * ki_wall;
    }


    public double gyroPID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_gyro;
        double Ierror = error + prevError_gyro;

        prevError_gyro = error;

        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
    }

    public double polePID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_pole;
        double Ierror = error + prevError_pole;

        prevError_pole = error;

        return pError * kp_pole + dError * kd_pole + Ierror * ki_pole;
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
}
