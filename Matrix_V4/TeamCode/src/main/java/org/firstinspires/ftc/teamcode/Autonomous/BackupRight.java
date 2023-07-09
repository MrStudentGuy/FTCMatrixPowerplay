package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
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
public class BackupRight extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    Pose2d PARKING1 = new Pose2d(12, -62+24, Math.toRadians(0));
    Pose2d PARKING1_INSIDE = new Pose2d(-58, -24, Math.toRadians(90));

    Pose2d PARKING2 = new Pose2d(36, -62+24, Math.toRadians(0));
    Pose2d PARKING2_INSIDE = new Pose2d(-34, -24, Math.toRadians(90));


    Pose2d PARKING3 = new Pose2d(60, -62+24, Math.toRadians(0));
    Pose2d PARKING3_INSIDE = new Pose2d(-10, -24, Math.toRadians(90));

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;

    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;

    int[] MATRIX_IDS = {3, 7, 9};

    AprilTagDetection tagOfInterest = null;
    Pose2d startPose = new Pose2d(31.8, -63.3, Math.toRadians(0));

    final Pose2d leftPoint2 = new Pose2d(31.8-20, -62+24, Math.toRadians(0));
    final Pose2d leftPoint1 = new Pose2d(31.8-20, -62, Math.toRadians(0));

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Robot robot = null;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        turret.reset();
        lift.reset();

        Servos.Slider.moveInside();
        Servos.Gripper.openGripperAutoStart();
        Servos.Wrist.goGripping();
        Servos.AlignBar_2.goInside();
        Servos.AlignBar.inside();
        Servos.SliderServo.setPosition(0);
        Robot.targetDegree = 0;
        robot.setPoseEstimate(startPose);


        TrajectorySequence startToLeft = robot.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(leftPoint1, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence leftToMiddleParking = robot.trajectorySequenceBuilder(startToLeft.end())
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4],1))
                .lineToLinearHeading(leftPoint2, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .lineToLinearHeading(new Pose2d(-44.01, -11.8, Math.toRadians(180)))
                .build();

        TrajectorySequence parking1Traj = robot.trajectorySequenceBuilder(leftToMiddleParking.end())
                .lineToLinearHeading(PARKING1, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .waitSeconds(1)
//                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
//                .waitSeconds(1)
//                .addTemporalMarker(()->Robot.targetDegree = 90)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .build();

        TrajectorySequence parking2Traj = robot.trajectorySequenceBuilder(leftToMiddleParking.end())
                .lineToLinearHeading(PARKING2, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .waitSeconds(1)
//                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
//                .waitSeconds(1)
//                .addTemporalMarker(()->Robot.targetDegree = 90)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
//                .lineToLinearHeading(PARKING2_INSIDE)
                .build();

        TrajectorySequence parking3Traj = robot.trajectorySequenceBuilder(leftToMiddleParking.end())
                .lineToLinearHeading(PARKING3, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .waitSeconds(1)
//                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
//                .waitSeconds(1)
//                .addTemporalMarker(()->Robot.targetDegree = 90)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
//                .splineTo(new Vector2d(-11.27, -32.32), Math.toRadians(-90))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            /**
             * {@inheritDoc}
             */
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            /**
             * {@inheritDoc}
             * @param errorCode
             */
            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        while(opModeInInit()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == MATRIX_IDS[PARKING_ZONE1] || tag.id == MATRIX_IDS[PARKING_ZONE2] || tag.id == MATRIX_IDS[PARKING_ZONE3]) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        robot.followTrajectorySequence(startToLeft);
        robot.followTrajectorySequence(leftToMiddleParking);

        if(tagOfInterest != null){
            if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]){
                robot.followTrajectorySequence(parking1Traj);
                Robot.heading = Math.toRadians(90);
            }
            else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]){
                robot.followTrajectorySequence(parking2Traj);
                Robot.heading = Math.toRadians(90);
            }
            else  if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]){
                robot.followTrajectorySequence(parking3Traj);
                Robot.heading = Math.toRadians(90);
            }
        }
        else{
            robot.followTrajectorySequence(parking2Traj);
            Robot.heading = Math.toRadians(90);
        }

        while(opModeIsActive()){

            robot.update();
        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
