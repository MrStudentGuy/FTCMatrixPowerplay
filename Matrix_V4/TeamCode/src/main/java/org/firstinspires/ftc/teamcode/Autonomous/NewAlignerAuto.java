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
import org.firstinspires.ftc.teamcode.TransferClass;
import org.firstinspires.ftc.teamcode.Vision.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class NewAlignerAuto extends LinearOpMode {

    //----------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------APRIL TAG DETECTION-------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    Pose2d PARKING1 = new Pose2d(-58, -12, Math.toRadians(90));
    Pose2d PARKING2 = new Pose2d(-34, -13, Math.toRadians(90));
    Pose2d PARKING3 = new Pose2d(-10, -12, Math.toRadians(90));

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

    int[] MATRIX_IDS = {3, 7, 9};

    AprilTagDetection tagOfInterest = null;
    //----------------------------------------------------------------------------------------------------------------------------------
    //**********************************************************************************************************************************
    //----------------------------------------------------------------------------------------------------------------------------------



    //----------------------------------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------COORDINATES---------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------
    final Pose2d pickingPosition = new Pose2d(-44, -12, Math.toRadians(180));
    final Pose2d pickingPosition1 = new Pose2d(-44.01, -12, Math.toRadians(180));
    final Pose2d pickingPosition2 = new Pose2d(-45.5, -12, Math.toRadians(180));
    final Pose2d pickingPosition3 = new Pose2d(-46.5, -12, Math.toRadians(180));
    final Pose2d pickingPosition4 = new Pose2d(-47, -12, Math.toRadians(180));

    final Pose2d midDropPosition = new Pose2d(-36, -10, Math.toRadians(180));
    final Pose2d centerHighPosition = new Pose2d(midDropPosition.getX() + 24, -12, Math.toRadians(180));

    //----------------------------------------------------------------------------------------------------------------------------------
    //**********************************************************************************************************************************
    //----------------------------------------------------------------------------------------------------------------------------------


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
        Servos.AlignBar.inside();
        Servos.SliderServo.setPosition(0);

        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        TransferClass.offsetpose = 90;

        TrajectorySequence autonomousTrajectory = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addSpatialMarker(new Vector2d(startPose.getX(), startPose.getY()), ()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .addDisplacementMarker(1, ()->{
                    Robot.targetDegree = -144;
                    Servos.Slider.moveSlider(0.15);
                })
                .addDisplacementMarker(10, ()-> {;
                })
                .addDisplacementMarker(20,()-> Servos.Wrist.setPosition(0.3394))
                .lineToLinearHeading(midDropPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->Servos.AlignBar_2.setPosition(0.38))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->robot.setTargetForSlider(0.56))
                .waitSeconds(0.00000000001)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.01)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .waitSeconds(0.05)
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->turret.setMaxPower(0.8))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .lineToLinearHeading(pickingPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->lift.extendTo(lift.AUTO_POSITION[4], 1))
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.setTargetForSlider(0.76))
                .waitSeconds(0.35)



                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Robot.targetDegree = -152)
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.6)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(0.38))
                .addTemporalMarker(()->Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .lineToLinearHeading(pickingPosition1)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.setTargetForSlider(0.76))
                .waitSeconds(0.35)




                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Robot.targetDegree = -152)
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.6)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(0.38))
                .addTemporalMarker(()->Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .lineToLinearHeading(pickingPosition)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.setTargetForSlider(0.76))
                .waitSeconds(0.35)



                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Robot.targetDegree = -152)
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.6)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(0.38))
                .addTemporalMarker(()->Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .lineToLinearHeading(pickingPosition1)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 1))
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.setTargetForSlider(0.76))
                .waitSeconds(0.35)




                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Robot.targetDegree = -152)
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.6)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(0.38))
                .addTemporalMarker(()->Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                .lineToLinearHeading(pickingPosition)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[0], 1))
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.setTargetForSlider(0.76))
                .waitSeconds(0.35)



                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE], 1))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Robot.targetDegree = -152)
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.6)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(0.38))
                .addTemporalMarker(()->Servos.Wrist.setPosition(0.3394))
                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 10)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .addTemporalMarker(()->Robot.targetDegree = 0)



                .waitSeconds(20)



                .build();

        TrajectorySequence parking1Seq = robot.trajectorySequenceBuilder(autonomousTrajectory.end())
                .lineToLinearHeading(PARKING1)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .build();

        TrajectorySequence parking2Seq = robot.trajectorySequenceBuilder(autonomousTrajectory.end())
                .lineToLinearHeading(PARKING2)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .build();

        TrajectorySequence parking3Seq = robot.trajectorySequenceBuilder(autonomousTrajectory.end())
                .lineToLinearHeading(PARKING3)
                .waitSeconds(0.01)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
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
//        waitForStart();
        Robot.targetDegree = 0;
        robot.followTrajectorySequence(autonomousTrajectory);
        if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]){
            robot.followTrajectorySequence(parking1Seq);
        }
        else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]){
            robot.followTrajectorySequence(parking2Seq);
        }
        else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]){
            robot.followTrajectorySequence(parking3Seq);
        }


        Servos.AlignBar.inside();
        double turretAngle = turret.getPosition();
        double height = lift.getPosition()[0];

        while(opModeIsActive()){
            double alignBarPos = Servos.AlignBar.getPosition();
            double wristPos = Servos.Wrist.getPosition();
            double x = Servos.Slider.getPosition();
            robot.update();

            if(gamepad1.a){
                alignBarPos += 0.01;
            }
            else if(gamepad1.b){
                alignBarPos -= 0.01;
            }

            if(gamepad1.x){
                wristPos += 0.01;
            }
            else if(gamepad1.y){
                wristPos -= 0.01;
            }

            if(gamepad1.dpad_up){
                height += 5;
            }
            else if(gamepad1.dpad_down){
                height -= 5;
            }

            if(gamepad1.dpad_right){
                turretAngle -= 0.5;
            }
            else if(gamepad1.dpad_left){
                turretAngle += 0.5;
            }

            if(gamepad1.right_bumper){
                x += 0.01;
            }
            else if(gamepad1.left_bumper){
                x -= 0.01;
            }

            Servos.Slider.moveSlider(x);
            Servos.Wrist.setPosition(wristPos);
            Servos.AlignBar.moveTo(alignBarPos);
            Robot.targetDegree = turretAngle;
            lift.extendTo((int)height, 1);


            telemetry.addData("Wrist: ", wristPos);
            telemetry.addData("Align: ", alignBarPos);
            telemetry.addData("Height: ", height);
            telemetry.addData("Angle: ", turretAngle);
            telemetry.addData("Currents Lift: ", lift.getCurrent()[0] + ", " + lift.getCurrent()[1]);
            telemetry.addData("X: ", x);
//            telemetry.update();


//            telemetry.addData("tagOfInterest is: ", tagOfInterest.id);
//            telemetry.update();
        }
    }





    void dropCone(){
        Servos.Gripper.openGripper();

    }
    /**
     * Get the information about the tag being detected and display it
     * @param detection The tag detection currently active
     */
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
