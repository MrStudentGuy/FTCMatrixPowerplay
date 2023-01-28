package org.firstinspires.ftc.teamcode.Autonomous;

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
public class RedLeftV4 extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;
    final double MAX_SPEED_AUTO = DriveConstants.MAX_VEL;


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
        sensors = new Sensors(hardwareMap, telemetry);

        lift.reset();
        turret.reset();
        setInitialPositions();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-42.75, -63, Math.toRadians(90));
        Pose2d startPoseOverall = new Pose2d(-41.5, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPoseOverall);

        TrajectorySequence startToCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()->lift.extendToLowPole())
                .lineToConstantHeading(new Vector2d(-36, -63))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->turret.setDegree(-45))
                .lineToConstantHeading(new Vector2d(-36, -12))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->lift.extendToHighPole())
                .addTemporalMarker(()->Servos.Slider.moveOutside())
                .waitSeconds(1)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                        .build();

        TrajectorySequence pick5 = drive.trajectorySequenceBuilder(startToCenter.end())
                        .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->lift.extendTo(lift.AUTO_POSITION[4], 1))
                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()->lift.extendToMidPole())
                .waitSeconds(0.1)
//                                .turn(Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-55, -12))
                                        .build();

        TrajectorySequence drop5 = drive.trajectorySequenceBuilder(pick5.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{turret.setDegree(-90);lift.extendToHighPole();})
                .lineToConstantHeading(new Vector2d(-24, -12))

                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();

        TrajectorySequence pick4 = drive.trajectorySequenceBuilder(drop5.end())
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .lineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()->lift.extendToMidPole())
                .waitSeconds(0.1)
//                                .turn(Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-55, -12))
                .build();

        TrajectorySequence drop4 = drive.trajectorySequenceBuilder(pick4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{turret.setDegree(-90);lift.extendToHighPole();})
                .lineToConstantHeading(new Vector2d(-24, -11))

                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();

        TrajectorySequence pick3 = drive.trajectorySequenceBuilder(drop4.end())
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .lineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()->lift.extendToMidPole())
                .waitSeconds(0.1)
//                                .turn(Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-55, -12))
                .build();

        TrajectorySequence drop3 = drive.trajectorySequenceBuilder(pick3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{turret.setDegree(-90);lift.extendToHighPole();})
                .lineToConstantHeading(new Vector2d(-24, -11))

                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();

        waitForStart();
        lift.reset();
        turret.reset();
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if(currentDetections.size() != 0) {
//                boolean tagFound = false;
//                for(AprilTagDetection tag : currentDetections) {
//                    if(tag.id == MATRIX_IDS[PARKING_ZONE1] || tag.id == MATRIX_IDS[PARKING_ZONE2] || tag.id == MATRIX_IDS[PARKING_ZONE3]) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if(tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else{
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if(tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//                if(tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            telemetry.update();
//            sleep(20);
//        }


//        drive.followTrajectorySequence(startToDropPreload);
//        drive.followTrajectorySequence(pickCone5);
//        align(drive);
//        align(drive);
//        drive.followTrajectorySequence(dropCone5);
//        drive.followTrajectorySequence(pickCone4);
//        align(drive);
////        while(gripperSensor.getDistance(DistanceUnit.INCH)>1) drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
//        drive.followTrajectorySequence(dropCone4);
//        drive.followTrajectorySequence(pickCone3);
//        align(drive);
////        while(gripperSensor.getDistance(DistanceUnit.INCH)>1) drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
//        drive.followTrajectorySequence(dropCone3);
//        drive.followTrajectorySequence(pickCone2);
//        align(drive);
//        drive.followTrajectorySequence(dropCone2);
//        drive.followTrajectorySequence(pickCone1);
//        align(drive);
//        drive.followTrajectorySequence(dropCone1);
        drive.followTrajectorySequence(startToCenter);
        drive.followTrajectorySequence(pick5);
        drive.followTrajectorySequence(drop5);
        drive.followTrajectorySequence(pick4);
        drive.followTrajectorySequence(drop4);
        drive.followTrajectorySequence(pick3);
        drive.followTrajectorySequence(drop3);
        while(opModeIsActive()){
//            align(drive);
            Sensors.WallSensor.printDistance();
            telemetry.update();
        }
    }

    private void align(SampleMecanumDrive drive1){
        while(Sensors.WallSensor.lastDistance>22.00 || Sensors.WallSensor.getDistanceCM()>21.00){
            Sensors.WallSensor.printDistance();
            drive1.setWeightedDrivePower(new Pose2d(0, 0.2, 0));
        }
        drive1.setWeightedDrivePower(new Pose2d(0,0,0));
//        while(Sensors.GripperSensor.getDistanceINCH() >1.5 && opModeIsActive()) drive1.setWeightedDrivePower(new Pose2d(0.2, 0, 0));

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
