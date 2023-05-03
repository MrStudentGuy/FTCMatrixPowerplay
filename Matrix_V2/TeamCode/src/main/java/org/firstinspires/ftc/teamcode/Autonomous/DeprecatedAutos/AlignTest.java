package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
@Disabled
public class AlignTest extends LinearOpMode {

    RevColorSensorV3 sensor = null;
    SampleMecanumDrive drive = null;

    double currentDist = 0, prevDist = 0;

    boolean flag = false;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensorAlign");
//        telemetry.setMsTransmissionInterval(10);

        waitForStart();



        while(opModeIsActive()){
            currentDist = sensor.getDistance(DistanceUnit.MM);
            if(currentDist-prevDist < -20){
                flag = true;
                telemetry.addLine("Pole detected");
//                telemetry.setMsTransmissionInterval(500);
            }
//            if(!flag){
//                drive.setWeightedDrivePower(new Pose2d(0.3, 0, 0));
//            }
//            else if(flag){
//                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//            }
            telemetry.addData("flag", flag);
            telemetry.addData("Dist: ", currentDist);
            telemetry.update();
            prevDist = currentDist;
        }
    }
}
