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
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
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
//@TeleOp
//public class PleaseWorkIDontKnowWhatIWillDoIfYouDont extends LinearOpMode {
//
//    boolean lastUp, lastDown, lastA, lastB, lastRB, lastLB;
//
//    double sliderPos = 0;
//    double wristPos = 0;
//    double alignBarPos = 0;
//    ElapsedTime timer = null;
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    Pose2d PARKING1 = new Pose2d(-52, -12, Math.toRadians(90));
//    Pose2d PARKING2 = new Pose2d(-34, -12, Math.toRadians(90));
//    Pose2d PARKING3 = new Pose2d(-10, -12, Math.toRadians(90));
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
//    final double dropAngle1 = -151, grippingTime = 200, turret0Time = 1200;
//    final Pose2d preloadDropPosition = new Pose2d(-40.5, -12, Math.toRadians(180));
//
//    final Pose2d coneDropPosition = new Pose2d(-44, -12, Math.toRadians(180));
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
//        timer = new ElapsedTime();
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
//        alignBarPos = Servos.AlignBar.getPosition();
//        sliderPos = Servos.Slider.getPosition();
//        wristPos = Servos.Wrist.getPosition();
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
//                .addTemporalMarker(() -> Servos.Gripper.closeGripper())
//                .addTemporalMarker(0.25, () -> lift.extendTo(lift.POSITIONS[lift.MID_POLE], 1))
//                .addTemporalMarker(0.4, () -> {
//                    turret.setTargetDegree(145);
//                    Servos.Wrist.goAutoTop();
//                    Servos.AlignBar.outside();
//                })
//                .lineToLinearHeading(preloadDropPosition)
//                .addTemporalMarker(() -> Servos.Slider.moveSlider(0.5))
////                .waitSeconds(0.05)
////                .UNSTABLE_addTemporalMarkerOffset(-0.001,()-> Servos.Slider.moveSlider(0.5))
//                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.03)
//                .addTemporalMarker(() -> Servos.Wrist.goGripping())
//                .addTemporalMarker(() -> Servos.AlignBar.dropPosition())
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> {
//                    dropCone();
//                })
//                .addTemporalMarker(() -> Servos.AlignBar.outside())
//                .addTemporalMarker(() -> Servos.Slider.moveInside())
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> {
//                    Servos.AlignBar.inside();
//                    Servos.Gripper.closeGripper();
//                })
//                .addTemporalMarker(() -> {
//                    turret.setTargetDegree(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .lineToLinearHeading(coneDropPosition)
//                .addTemporalMarker(() -> lift.extendTo(lift.AUTO_POSITION[4], 1))
//
//                .build();
//
//        waitForStart();
//
//        robot.followTrajectorySequence(autonomousTrajectory);
//
//        for (int i = 4; i >= 0; i--) {
//            lift.extendTo(lift.AUTO_POSITION[i], 1);
//            Servos.Gripper.openGripper();
//            delay(1000);
//            Servos.Slider.moveOutside();
//            delay(500);
//            Servos.Gripper.closeGripper();
//            delay((int) grippingTime);
//            lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1);
//            Servos.Wrist.setPosition(0.346);
//            Servos.AlignBar.interMediate();
//            delay(100);
//            Servos.Slider.moveInside();
//            delay(200);
//            turret.setTargetDegree(dropAngle1);
//            delay(1000);
//            Servos.AlignBar.moveTo(0.242);
//            delay(1000);
//            Servos.Slider.moveSlider(0.45);
//                delay(1500);
//                Servos.Wrist.goGripping();
//                delay(400);
//                dropCone();
//                delay(10);
//            Servos.Slider.moveInside();
//            delay(100);
//            Servos.AlignBar.inside();
//            Servos.Gripper.closeGripper();
//            turret.setTargetDegree(0);
//            delay(turret0Time/2);
//            if(i>0){
//                lift.extendTo(lift.AUTO_POSITION[i-1], 1);
//            }
//            delay(turret0Time/2);
//
////            sliderPos = 0.45;
////            wristPos = 0.346;
////            alignBarPos = 0.242;
//
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
//
//    void delay(double ms){
//        while(timer.milliseconds() < ms){
//            robot.update();
//        }
//        timer.reset();
//    }
//}
