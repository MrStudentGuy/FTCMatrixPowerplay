package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//@Disabled
public class KevalAuto extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;

    ElapsedTime AutoTime = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    final double MAX_SPEED_AUTO = DriveConstants.MAX_VEL;

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

    int[] MATRIX_IDS = {3,7,9};

    AprilTagDetection tagOfInterest = null;



    final Pose2d droppingPosition = new Pose2d(-39.3, -12.00, Math.toRadians(180));
    final Pose2d pickingPosition = new Pose2d(-47.6, -12, Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();

        Pose2d PARKING1 = new Pose2d(-60, -12, Math.toRadians(180));
        Pose2d PARKING2 = new Pose2d(-36, -13, Math.toRadians(180));
        Pose2d PARKING3 = new Pose2d(-12, -12, Math.toRadians(180));
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
        Pose2d startPose = new Pose2d(-32, -63.3, Math.toRadians(180));
//        Pose2d startPose = new Pose2d(-32.3, -64.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence startToCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()->turret.setDegree(-140))
                .addTemporalMarker(()->lift.extendToHighPole())
                .addTemporalMarker(()->Servos.Slider.moveSlider(1))
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .lineToLinearHeading(droppingPosition)
//                .waitSeconds(0.1)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.03)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .addTemporalMarker(()->Servos.AlignBar.inside())
                .waitSeconds(0.5)
                .build();

        TrajectorySequence pick1 = drive.trajectorySequenceBuilder(startToCenter.end())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
                .addTemporalMarker(()-> Servos.Slider.moveHalfway())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .waitSeconds(0.2)
                .lineToLinearHeading(pickingPosition)
//                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.9))
                .addTemporalMarker(()->turret.setDegree(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.5)
                .addTemporalMarker(()->lift.extendToLowPole())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence dropCone1 = drive.trajectorySequenceBuilder(pick1.end())
                .addTemporalMarker(()->turret.setDegree(-145))
//                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(0.4, ()-> Servos.AlignBar.outside())
                .lineToLinearHeading(droppingPosition)
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{lift.extendToHighPole();
                })
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.setPosition(1))
                .addTemporalMarker(()-> Servos.AlignBar.inside())
                .waitSeconds(0.5)
                .build();

        TrajectorySequence pick2 = drive.trajectorySequenceBuilder(dropCone1.end())
//                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .addTemporalMarker(()-> Servos.Slider.moveHalfway())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .waitSeconds(0.2)
                .lineToLinearHeading(pickingPosition)
//                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.9))
                .addTemporalMarker(()->turret.setDegree(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.5)
                .addTemporalMarker(()->lift.extendToLowPole())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence pick3 = drive.trajectorySequenceBuilder(dropCone1.end())
//                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .addTemporalMarker(()-> Servos.Slider.moveHalfway())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .waitSeconds(0.2)
                .lineToLinearHeading(pickingPosition)
//                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.9))
                .addTemporalMarker(()->turret.setDegree(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.5)
                .addTemporalMarker(()->lift.extendToLowPole())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence pick4 = drive.trajectorySequenceBuilder(dropCone1.end())
//                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 1))
                .addTemporalMarker(()-> Servos.Slider.moveHalfway())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .waitSeconds(0.2)
                .lineToLinearHeading(pickingPosition)
//                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(1))
                .addTemporalMarker(()->turret.setDegree(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.5)
                .addTemporalMarker(()->lift.extendToLowPole())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence pick5 = drive.trajectorySequenceBuilder(dropCone1.end())
//                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 1))
                .addTemporalMarker(()-> Servos.Slider.moveHalfway())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegreeHighPower(-10))
                .waitSeconds(0.2)
                .lineToLinearHeading(pickingPosition)
//                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.9))
                .addTemporalMarker(()->turret.setDegree(0))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->lift.extendToLowPole())
//                .waitSeconds(0.2)
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()->lift.extendTo(0, 1))
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goInit())
                .build();




        TrajectorySequence goToP1 = drive.trajectorySequenceBuilder((startToCenter.end()))
                .addTemporalMarker(()-> Servos.Wrist.goInit())
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING1.getX(), PARKING1.getY(), Math.toRadians(90)))
//                .turn(Math.toRadians(-90))
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()->lift.extendTo(0, 0.5))
//                .lineToLinearHeading(new Pose2d(PARKING1.getX(), PARKING1.getY()-24, Math.toRadians(90)))
                .build();
        TrajectorySequence goToP2 = drive.trajectorySequenceBuilder((startToCenter.end()))
                .addTemporalMarker(()-> Servos.Wrist.goInit())
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING2.getX(), PARKING2.getY(), Math.toRadians(90)))
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->lift.extendTo(0, 0.5))
//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(PARKING2.getX(), PARKING2.getY()-24, Math.toRadians(90)))
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .build();
        TrajectorySequence goToP3 = drive.trajectorySequenceBuilder((startToCenter.end()))
                .addTemporalMarker(()-> Servos.Wrist.goInit())
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING2.getX(), PARKING2.getY(), Math.toRadians(90)))
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->lift.extendTo(0, 0.5))
//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY()-24, Math.toRadians(90)))
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .build();




//        waitForStart();
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

        AutoTime.reset();
        lift.reset();
        turret.reset();
        turret.setDegree(0);

//        drive.followTrajectorySequence(rajStart);
        drive.followTrajectorySequence(startToCenter);
        drive.followTrajectorySequence(pick1);
        drive.followTrajectorySequence(dropCone1);
        drive.followTrajectorySequence(pick2);
        drive.followTrajectorySequence(dropCone1);
        drive.followTrajectorySequence(pick3);
        drive.followTrajectorySequence(dropCone1);
        drive.followTrajectorySequence(pick4);
        drive.followTrajectorySequence(dropCone1);
//        if(AutoTime.seconds() < 20) {
//            drive.followTrajectorySequence(pick5);
//        }
//        else{
            String ParkingZone = "3";

        if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]){
            ParkingZone = "1";
            drive.followTrajectorySequence(goToP1);

        }
        else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]){
            ParkingZone = "2";
            drive.followTrajectorySequence(goToP2);
        }
        else if(tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]){
            ParkingZone = "3";
            drive.followTrajectorySequence(goToP3);
        }
        else{
            drive.followTrajectorySequence(goToP3);
        }
//        }
//        drive.followTrajectorySequence(dropCone1);
//        timer1.reset();
//        while(timer1.milliseconds() < 2000){
//            double current = Sensors.WallSensor.getDistanceCM()
//        }
//


//        telemetry.addData("Parking Zone: ", ParkingZone);
        while(opModeIsActive()){
//            Servos.Slider.moveSlider(gamepad1.left_trigger);
//            telemetry.addData("Parking Zone: ", ParkingZone);
//            align(drive);
            Sensors.WallSensor.printDistance();
            telemetry.update();
        }
    }

//    private void align(SampleMecanumDrive drive1){
//        timer1.reset();
//        while(timer1.milliseconds() < 1000) {
//            double power = wallSensorPID(Sensors.WallSensor.getDistanceCM(), 19);
//            Pose2d poseEstimate = drive1.getPoseEstimate();
//            double heading = Math.toDegrees(poseEstimate.getHeading());
//            double headingPower = gyroPID(heading, 180);
//
//            double distance = Sensors.PoleSensor.getDistanceCM();
//            double polePower = 0;
//
//            if (Math.abs(power) < 0.09) {
//                power = 0;
//            }
//
//            if (Math.abs(headingPower) < 0.09) {
//                headingPower = 0;
//            }
//
//            if (Math.abs(headingPower) == 0 && Math.abs(power) == 0) {
//                polePower = polePID(distance, 12.5);
//            } else {
//                polePower = 0;
//            }
//            drive1.setWeightedDrivePower(new Pose2d(power, polePower, -headingPower));
//
//            drive1.update();
//            telemetry.addData("Pole is at: ", distance + "CM");
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("heading normalized", heading);
////            telemetry.update();
//
//            Sensors.WallSensor.printDistance();
////            telemetry.addData("Distance: ", Sensors.WallSensor.getDistanceCM());
//            telemetry.update();
//        }
//    }

    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        turret.setDegree(0);
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
