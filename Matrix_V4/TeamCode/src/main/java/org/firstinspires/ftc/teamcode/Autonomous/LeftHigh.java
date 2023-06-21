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
public class LeftHigh extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    Pose2d PARKING1 = new Pose2d(-58, -12, Math.toRadians(90));
    Pose2d PARKING1_INSIDE = new Pose2d(-58, -24, Math.toRadians(90));

    Pose2d PARKING2 = new Pose2d(-34, -13, Math.toRadians(90));
    Pose2d PARKING2_INSIDE = new Pose2d(-34, -24, Math.toRadians(90));


    Pose2d PARKING3 = new Pose2d(-10, -12, Math.toRadians(90));
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
    Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
    final Pose2d dropPosition = new Pose2d(-40, -12, Math.toRadians(180));
    final Pose2d pickingPosition1 = new Pose2d(-45.01, -13, Math.toRadians(180));
    final Pose2d midDropPosition = new Pose2d(-36, -10, Math.toRadians(180));

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
        Servos.Gripper.openGripper();
        Servos.Wrist.goGripping();
        Servos.AlignBar_2.goInside();
        Servos.SliderServo.setPosition(0);
        Robot.targetDegree = 0;
        robot.setPoseEstimate(startPose);


        TrajectorySequence startToMid = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.2))
                .lineToLinearHeading(midDropPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-1.3, ()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE],1))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, ()->Robot.targetDegree = -(AutoPositions.preloadTurretPosition+2))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> Servos.Wrist.setPosition(AutoPositions.preloadWristPosition))
//                .waitSeconds(0.01)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> Servos.AlignBar_2.setPosition(AutoPositions.preloadAlignPosition))
                .addTemporalMarker(()-> robot.setTargetForSlider(AutoPositions.preloadSliderPosition+1))
                .waitSeconds(0.0001)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.openGripperFull())
                .addTemporalMarker(()-> Servos.AlignBar_2.goInside())
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setMaxPower(0.6))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .build();

        TrajectorySequence preloadToPick = robot.trajectorySequenceBuilder(startToMid.end())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.4))
                .waitSeconds(0.25)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();

        TrajectorySequence pickToDrop = robot.trajectorySequenceBuilder(pickingPosition1)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE],1))
                .waitSeconds(0.1)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setMaxPower(0.5))
                .addTemporalMarker(()->Robot.targetDegree = -(AutoPositions.highTurretPosition+3))
                .addTemporalMarker(()-> Servos.Wrist.setPosition(AutoPositions.highWristPosition))
                .lineToLinearHeading(dropPosition)
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(AutoPositions.highAlignPosition))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.7)
                .addTemporalMarker(()->robot.setTargetForSlider(AutoPositions.highSliderPosition))
                .waitSeconds(0.55)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.15)
                .addTemporalMarker(()-> Servos.Gripper.openGripperFull())
                .addTemporalMarker(()-> Servos.AlignBar_2.goInside())
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->turret.setMaxPower(0.7))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .build();

        TrajectorySequence dropToPick3 = robot.trajectorySequenceBuilder(dropPosition)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->lift.extendTo(lift.AUTO_POSITION[3],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.4))
                .waitSeconds(0.25)
                .addTemporalMarker(()->turret.setMaxPower(0))
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();


        TrajectorySequence dropToPick2 = robot.trajectorySequenceBuilder(dropPosition)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->lift.extendTo(lift.AUTO_POSITION[2],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.4))
                .waitSeconds(0.25)
                .addTemporalMarker(()->turret.setMaxPower(0.4))
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();


        TrajectorySequence dropToPick1 = robot.trajectorySequenceBuilder(dropPosition)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->lift.extendTo(lift.AUTO_POSITION[1],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.4))
                .waitSeconds(0.25)
                .addTemporalMarker(()->turret.setMaxPower(0.4))
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();


        TrajectorySequence dropToPick0 = robot.trajectorySequenceBuilder(dropPosition)
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->lift.extendTo(lift.AUTO_POSITION[0],1))
                .lineToLinearHeading(pickingPosition1)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->robot.setTargetForSlider(0.4))
                .waitSeconds(0.25)
                .addTemporalMarker(()->turret.setMaxPower(0.4))
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();

        TrajectorySequence parking1Traj = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
                .splineTo(new Vector2d(-56.42, -20.56), Math.toRadians(246.41))
                .splineTo(new Vector2d(-58.48, -30.50), Math.toRadians(-90))
                .build();

        TrajectorySequence parking2Traj = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
//                .lineToLinearHeading(PARKING2)
                .lineToLinearHeading(PARKING2_INSIDE)
                .build();

        TrajectorySequence parking3Traj = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()-> lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
                .setReversed(true)
                .splineTo(new Vector2d(-11.27, -32.32), Math.toRadians(-90))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        robot.followTrajectorySequence(startToMid);
        robot.followTrajectorySequence(preloadToPick);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick3);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick2);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick1);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick0);
        robot.followTrajectorySequence(pickToDrop);

        if(tagOfInterest != null){
            if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]){
                robot.followTrajectorySequence(parking1Traj);
                Robot.heading = Math.toRadians(180);
            }
            else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]){
                robot.followTrajectorySequence(parking2Traj);
                Robot.heading = Math.toRadians(0);
            }
            else  if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]){
                robot.followTrajectorySequence(parking3Traj);
                Robot.heading = Math.toRadians(0);
            }
        }
        else{
            robot.followTrajectorySequence(parking2Traj);
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
