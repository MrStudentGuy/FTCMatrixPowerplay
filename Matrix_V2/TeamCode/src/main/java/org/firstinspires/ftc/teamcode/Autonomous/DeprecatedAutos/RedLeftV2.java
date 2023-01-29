package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Vision.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
@Disabled
public class RedLeftV2 extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    RevColorSensorV3 gripperSensor = null;

    final double MAX_SPEED_AUTO = 40;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;

    int[] MATRIX_IDS = {14,3,17};

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.setMsTransmissionInterval(50);


        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");
        lift.reset();
        turret.reset();
        setInitialPositions();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-43.5, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence startToDropPreload =drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegree(-90))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-38, 0), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.openGripper())

                .build();

        TrajectorySequence pickCone5 = drive.trajectorySequenceBuilder(startToDropPreload.end())
                .lineToConstantHeading(new Vector2d(-36, -12), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 0.8))
                .lineToConstantHeading(new Vector2d(-54.5, -11), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.25))
                .build();


        TrajectorySequence dropCone5 = drive.trajectorySequenceBuilder(pickCone5.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone4 = drive.trajectorySequenceBuilder(dropCone5.end())
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 0.8))
                .lineToConstantHeading(new Vector2d(-55, -12), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.25))
                .build();


        TrajectorySequence dropCone4 = drive.trajectorySequenceBuilder(pickCone4.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone3 = drive.trajectorySequenceBuilder(dropCone4.end())
                .addTemporalMarker(()->turret.setDegree(90))

                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 0.8))
                .lineToConstantHeading(new Vector2d(-55, -12), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.15))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();


        TrajectorySequence dropCone3 = drive.trajectorySequenceBuilder(pickCone3.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone2 = drive.trajectorySequenceBuilder(dropCone3.end())
                .addTemporalMarker(()->turret.setDegree(90))

                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 0.8))
                .lineToConstantHeading(new Vector2d(-55, -12), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.2))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();


        TrajectorySequence dropCone2 = drive.trajectorySequenceBuilder(pickCone2.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone1 = drive.trajectorySequenceBuilder(dropCone2.end())
                .addTemporalMarker(()->turret.setDegree(90))

                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 0.8))
                .lineToConstantHeading(new Vector2d(-55, -12), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.2))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();


        TrajectorySequence dropCone1 = drive.trajectorySequenceBuilder(pickCone1.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31), SampleMecanumDrive.getVelocityConstraint(MAX_SPEED_AUTO, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();



        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0) {
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == MATRIX_IDS[PARKING_ZONE1] || tag.id == MATRIX_IDS[PARKING_ZONE2] || tag.id == MATRIX_IDS[PARKING_ZONE3]) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else{
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }


        lift.reset();
        turret.reset();
        drive.followTrajectorySequence(startToDropPreload);
        drive.followTrajectorySequence(pickCone5);
        while(gripperSensor.getDistance(DistanceUnit.INCH)>1){
            telemetry.addData("dist: ", gripperSensor.getDistance(DistanceUnit.MM));
            drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
        }
        drive.followTrajectorySequence(dropCone5);
        drive.followTrajectorySequence(pickCone4);
        align(drive);
//        while(gripperSensor.getDistance(DistanceUnit.INCH)>1) drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
        drive.followTrajectorySequence(dropCone4);
        drive.followTrajectorySequence(pickCone3);
        align(drive);
//        while(gripperSensor.getDistance(DistanceUnit.INCH)>1) drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
        drive.followTrajectorySequence(dropCone3);
        drive.followTrajectorySequence(pickCone2);
        align(drive);
        drive.followTrajectorySequence(dropCone2);
        drive.followTrajectorySequence(pickCone1);
        align(drive);
        drive.followTrajectorySequence(dropCone1);

        while(opModeIsActive()){

        }
    }

    private void align(SampleMecanumDrive drive1){
        while(Sensors.GripperSensor.getDistanceINCH() >1.5) drive1.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
    }

    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        turret.setDegree(0);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
