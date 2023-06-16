//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import static android.os.SystemClock.sleep;
//
//import android.transition.Slide;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Servos;
//import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.TransferClass;
//import org.firstinspires.ftc.teamcode.Vision.Pipelines.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//
//@Autonomous
//public class ZipZapZoom extends LinearOpMode {
//
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    Pose2d PARKING1 = new Pose2d(-52+72, -12, Math.toRadians(90));
//    Pose2d PARKING2 = new Pose2d(-34+72, -13, Math.toRadians(90));
//    Pose2d PARKING3 = new Pose2d(-10+72, -12, Math.toRadians(90));
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//
//
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//
//    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;
//
//    int[] MATRIX_IDS = {3, 7, 9};
//
//    AprilTagDetection tagOfInterest = null;
//
//    final double dropAngle1 = -143, grippingTime = 0.2, turret0Time = 0.3;
//    final Pose2d preloadDropPosition = new Pose2d(-40.5, -12, Math.toRadians(180));
//
//    final Pose2d coneDropPosition = new Pose2d(-46, -12, Math.toRadians(180));
//
//    final Pose2d otherpreloadDropPosition = new Pose2d(40.5, -12, Math.toRadians(180));
//
//    final Pose2d pickPosition = new Pose2d(-46.2, -12, Math.toRadians(180));
//
//    final Pose2d otherPickPosition = new Pose2d(46.2, -12, Math.toRadians(180));
//
//    final Pose2d farHighPosition = new Pose2d(-16.5, -12, Math.toRadians(180));
//
//    Lift lift = null;
//    Servos servos = null;
//    Turret turret = null;
//    Robot robot = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        PhotonCore.enable();
//        lift = new Lift(hardwareMap, telemetry);
//        servos = new Servos(hardwareMap, telemetry);
//        turret = new Turret(hardwareMap, "turret", telemetry);
//
//        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//        turret.reset();
//        lift.reset();
//
//        Servos.Slider.moveInside();
//        Servos.Gripper.openGripper();
//        Servos.Wrist.goGripping();
//
//        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
//        robot.setPoseEstimate(startPose);
//
//        TransferClass.offsetpose = 90;
//
//        TrajectorySequence autonomousTrajectory = robot.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
//                .addTemporalMarker(0.25,()->lift.extendTo(lift.POSITIONS[lift.MID_POLE], 1))
//                .addTemporalMarker(0.4, ()->{turret.setTargetDegree(145);
//                    Servos.Wrist.goAutoTop();
//                    Servos.AlignBar.outside();
//                })
//                .lineToLinearHeading(preloadDropPosition)
////                .waitSeconds(0.05)
//                .UNSTABLE_addTemporalMarkerOffset(-0.15,()-> Servos.Slider.moveSlider(0.5))
////                .waitSeconds(0.1)
////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.03)
//                .addTemporalMarker(()->Servos.Wrist.goGripping())
//                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()-> {dropCone();})
//                .waitSeconds(0.01)
//                .addTemporalMarker(()-> Servos.AlignBar.outside())
//                .addTemporalMarker(()->Servos.Slider.moveInside())
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()->{
//                    Servos.AlignBar.inside();
//                    Servos.Gripper.closeGripper();})
//                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
//
//
//
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .lineToLinearHeading(coneDropPosition)
//                .UNSTABLE_addTemporalMarkerOffset(-0.15, ()-> Servos.Gripper.openGripper())
//                .waitSeconds(0.15)
//                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//                .waitSeconds(0.25)
//                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
//                .waitSeconds(grippingTime)
//                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
//                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
//                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
//                .waitSeconds(0.1)
//                .addTemporalMarker(()-> Servos.Slider.moveInside())
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->turret.setTargetDegree(dropAngle1))
//                .waitSeconds(0.1)
//                .lineToLinearHeading(coneDropPosition)
//                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
//                .waitSeconds(0.2)
//                .addTemporalMarker(()-> Servos.Slider.moveOutside())
////                .waitSeconds(0.3)
//                .waitSeconds(0.6)
////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
////                .waitSeconds(0.1)
//                .addTemporalMarker(()->Servos.Wrist.goGripping())
//                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
//                .waitSeconds(0.15 )
//                .addTemporalMarker( ()-> {dropCone();})
//                .waitSeconds(0.01)
//                .addTemporalMarker(()-> Servos.AlignBar.outside())
//                .addTemporalMarker(()->Servos.Slider.moveInside())
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()->{
//                    Servos.AlignBar.inside();
//                    Servos.Gripper.closeGripper();})
//                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
//                .waitSeconds(turret0Time)
//
//
//
////                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
////                .lineToLinearHeading(pickPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.15, ()-> Servos.Gripper.openGripper())
////                .waitSeconds(0.15)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
////                .waitSeconds(0.25)
////                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
////                .waitSeconds(grippingTime)
////                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
////                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
////                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
////                .waitSeconds(0.1)
////                .addTemporalMarker(()-> Servos.Slider.moveInside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()->turret.setTargetDegree(dropAngle1))
////                .waitSeconds(0.1)
////                .lineToLinearHeading(preloadDropPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
//////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//////                .waitSeconds(0.3)
////                .waitSeconds(0.6)
//////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//////                .waitSeconds(0.1)
////                .addTemporalMarker(()->Servos.Wrist.goGripping())
////                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
////                .waitSeconds(0.15 )
////                .addTemporalMarker( ()-> {dropCone();})
////                .waitSeconds(0.01)
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .addTemporalMarker(()->Servos.Slider.moveInside())
////                .waitSeconds(0.1)
////                .addTemporalMarker( ()->{
////                    Servos.AlignBar.inside();
////                    Servos.Gripper.closeGripper();})
////                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
////                .waitSeconds(turret0Time)
////
////
////
////                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
////                .lineToLinearHeading(pickPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.15, ()-> Servos.Gripper.openGripper())
////                .waitSeconds(0.15)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
////                .waitSeconds(0.25)
////                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
////                .waitSeconds(grippingTime)
////                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
////                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
////                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
////                .waitSeconds(0.1)
////                .addTemporalMarker(()-> Servos.Slider.moveInside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()->turret.setTargetDegree(dropAngle1))
////                .waitSeconds(0.1)
////                .lineToLinearHeading(preloadDropPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
//////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//////                .waitSeconds(0.3)
////                .waitSeconds(0.6)
//////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//////                .waitSeconds(0.1)
////                .addTemporalMarker(()->Servos.Wrist.goGripping())
////                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
////                .waitSeconds(0.15 )
////                .addTemporalMarker( ()-> {dropCone();})
////                .waitSeconds(0.01)
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .addTemporalMarker(()->Servos.Slider.moveInside())
////                .waitSeconds(0.1)
////                .addTemporalMarker( ()->{
////                    Servos.AlignBar.inside();
////                    Servos.Gripper.closeGripper();})
////                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
////                .waitSeconds(turret0Time)
////
////
////
////
////                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 1))
////                .lineToLinearHeading(pickPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.15, ()-> Servos.Gripper.openGripper())
////                .waitSeconds(0.15)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
////                .waitSeconds(0.25)
////                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
////                .waitSeconds(grippingTime)
////                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
////                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
////                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
////                .waitSeconds(0.1)
////                .addTemporalMarker(()-> Servos.Slider.moveInside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()->turret.setTargetDegree(dropAngle1))
////                .waitSeconds(0.1)
////                .lineToLinearHeading(preloadDropPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
//////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//////                .waitSeconds(0.3)
////                .waitSeconds(0.6)
//////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//////                .waitSeconds(0.1)
////                .addTemporalMarker(()->Servos.Wrist.goGripping())
////                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
////                .waitSeconds(0.15 )
////                .addTemporalMarker( ()-> {dropCone();})
////                .waitSeconds(0.01)
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .addTemporalMarker(()->Servos.Slider.moveInside())
////                .waitSeconds(0.1)
////                .addTemporalMarker( ()->{
////                    Servos.AlignBar.inside();
////                    Servos.Gripper.closeGripper();})
////                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
////                .waitSeconds(turret0Time)
////
////
////
////
////
////                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[0], 1))
////                .lineToLinearHeading(pickPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.15, ()-> Servos.Gripper.openGripper())
////                .waitSeconds(0.15)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
////                .waitSeconds(0.25)
////                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
////                .waitSeconds(grippingTime)
////                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
////                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
////                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
////                .waitSeconds(0.1)
////                .addTemporalMarker(()-> Servos.Slider.moveInside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()->turret.setTargetDegree(dropAngle1))
////                .waitSeconds(0.1)
////                .lineToLinearHeading(preloadDropPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
//////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//////                .waitSeconds(0.3)
////                .waitSeconds(0.6)
//////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//////                .waitSeconds(0.1)
////                .addTemporalMarker(()->Servos.Wrist.goGripping())
////                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
////                .waitSeconds(0.15 )
////                .addTemporalMarker( ()-> {dropCone();})
////                .waitSeconds(0.01)
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .addTemporalMarker(()->Servos.Slider.moveInside())
////                .waitSeconds(0.1)
////                .addTemporalMarker( ()->{
////                    Servos.AlignBar.inside();
////                    Servos.Gripper.closeGripper();})
////                .addTemporalMarker( ()->{turret.setTargetDegree(-180);lift.extendTo(lift.AUTO_POSITION[4], 1);})
////
////
////                .lineToLinearHeading(otherPickPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> Servos.Gripper.openGripper())
////                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> Servos.Slider.moveOutside())
////                .UNSTABLE_addTemporalMarkerOffset(-0.28, ()-> Servos.Gripper.closeGripper())
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->{
////                    lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1);
////                    Servos.Wrist.goAutoTop();
////                    Servos.AlignBar.interMediate();
////                })
////                .UNSTABLE_addTemporalMarkerOffset(-0.001, ()->{
////                    Servos.Slider.moveInside();
////
////                })
//////                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
//////                .waitSeconds(0.1)
//////                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
//////                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
//////                .addTemporalMarker(()-> Servos.AlignBar.interMediate())
//////                .waitSeconds(0.1)
//////                .addTemporalMarker(()-> Servos.Slider.moveInside())
//////                .waitSeconds(0.2)
////                .addTemporalMarker(()->turret.setTargetDegree(-41))
////                .lineToLinearHeading(otherpreloadDropPosition)
////                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->Servos.AlignBar.outside())
//////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveOutside())
//////                .waitSeconds(0.3)
////                .waitSeconds(0.6)
//////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//////                .waitSeconds(0.1)
////                .addTemporalMarker(()->Servos.Wrist.goGripping())
////                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
////                .waitSeconds(0.15)
////                .addTemporalMarker( ()-> {dropCone();})
////                .waitSeconds(0.01)
////                .addTemporalMarker(()-> Servos.AlignBar.outside())
////                .addTemporalMarker(()->Servos.Slider.moveInside())
////                .waitSeconds(0.1)
////                .addTemporalMarker( ()->{
////                    Servos.AlignBar.inside();
////                    Servos.Gripper.closeGripper();})
////                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////                .waitSeconds(10)
//
//
//                .build();
///**
// * Sequence for going and parking at parking zone 1
// */
//        TrajectorySequence goToP1 = robot.trajectorySequenceBuilder((autonomousTrajectory.end()))
//                .lineToLinearHeading(new Pose2d(PARKING1.getX(), PARKING1.getY(), Math.toRadians(90)))
////                .addTemporalMarker(() -> turret.setDegree(0))
//
////                .turn(Math.toRadians(-90))
////                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY()-24, Math.toRadians(90)))
//
//                .build();
//
//        /**
//         * Sequence for going and parking at parking zone 2
//         */
//        TrajectorySequence goToP2 = robot.trajectorySequenceBuilder((autonomousTrajectory.end()))
//                .lineToLinearHeading(new Pose2d(PARKING2.getX(), PARKING2.getY(), Math.toRadians(90)))
////                .addTemporalMarker(() -> turret.
//                .build();
//
//        /**
//         * Sequence for going and parking at parking zone 3
//         */
//        TrajectorySequence goToP3 = robot.trajectorySequenceBuilder((autonomousTrajectory.end()))
//                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY(), Math.toRadians(90)))
//                .build();
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            /**
//             * {@inheritDoc}
//             */
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            /**
//             * {@inheritDoc}
//             * @param errorCode
//             */
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//        telemetry.setMsTransmissionInterval(50);
//
//        while(opModeInInit()) {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == MATRIX_IDS[PARKING_ZONE1] || tag.id == MATRIX_IDS[PARKING_ZONE2] || tag.id == MATRIX_IDS[PARKING_ZONE3]) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            telemetry.update();
//            sleep(20);
//        }
////        waitForStart();
//        robot.followTrajectorySequence(autonomousTrajectory);
//        if(tagOfInterest != null) {
//
//            if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]) {
//                robot.followTrajectorySequence(goToP1);
//
//            } else if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]) {
//                robot.followTrajectorySequence(goToP2);
//            } else if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]) {
//                robot.followTrajectorySequence(goToP3);
//            }
//        } else {
//            robot.followTrajectorySequence(goToP3);
//        }
//
//        while(opModeIsActive()){
//            telemetry.addData("tagOfInterest is: ", tagOfInterest.id);
//            telemetry.update();
//        }
//    }
//
//
//
//
//
//    void dropCone(){
//        Servos.Gripper.openGripper();
//
//    }
//    /**
//     * Get the information about the tag being detected and display it
//     * @param detection The tag detection currently active
//     */
//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}
