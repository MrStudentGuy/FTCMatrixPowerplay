package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.TransferClass;
import org.firstinspires.ftc.teamcode.Vision.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous
@Disabled
public class Right_Final extends OpMode {


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    Pose2d PARKING1 = new Pose2d(-58 + 72, -12, Math.toRadians(90));
    Pose2d PARKING2 = new Pose2d(-36 + 72, -13, Math.toRadians(90));
    Pose2d PARKING3 = new Pose2d(-10 + 72, -12, Math.toRadians(90));

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

    final Pose2d droppingPosition0 = new Pose2d(37.4, -11.5, Math.toRadians(0));       // Positions on the field to drop cone into pole
    final Pose2d droppingPosition = new Pose2d(37.4, -11.5001, Math.toRadians(0));
    final Pose2d pickingPosition = new Pose2d(49, -12, Math.toRadians(0));


    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Robot robot = null;

    enum State{
        STARTTOCENTER,
        PICK,
        DROP, PARK
    }


    State currentState = State.STARTTOCENTER;

    ElapsedTime timer = new ElapsedTime();


    int counter = 4;
    TrajectorySequence startToCenter;
    TrajectorySequence pick0, goToP1, goToP2, goToP3;
    Trajectory pick;
    Trajectory drop;
    @Override
    public void init() {
        PhotonCore.enable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);

        Servos.Gripper.closeGripper();
        Servos.Wrist.goInit();
        Servos.AlignBar.inside();



        Pose2d startPose = new Pose2d(31.8, -63.3, Math.toRadians(0));
        robot.setPoseEstimate(startPose);

        TransferClass.offsetpose = 90;

        startToCenter = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1))
                .addTemporalMarker(0.3, ()->{Robot.targetDegree = 138;})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->Servos.Wrist.goAutoTop())
                .lineToLinearHeading(droppingPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {Servos.Slider.moveSlider(0.6);})
                .waitSeconds(0.05)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.01)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->Servos.Wrist.goGripping())
//                .addTemporalMarker(()-> Servos.Wrist.goGripping())
//                .waitSeconds(0.05)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[4], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})

                .lineToLinearHeading(pickingPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .addTemporalMarker(()->)
//                .waitSeconds(10)
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1);})
                .lineToLinearHeading(droppingPosition0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->Robot.targetDegree = 138)
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.6))
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())


                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[3], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})

                .lineToLinearHeading(pickingPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .addTemporalMarker(()->)
//                .waitSeconds(10)
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1);})
                .lineToLinearHeading(droppingPosition0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->Robot.targetDegree = 138)
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.6))
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())


                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[2], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})

                .lineToLinearHeading(pickingPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .addTemporalMarker(()->)
//                .waitSeconds(10)
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1);})
                .lineToLinearHeading(droppingPosition0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->Robot.targetDegree = 138)
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.6))
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())



                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[1], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})

                .lineToLinearHeading(pickingPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .addTemporalMarker(()->)
//                .waitSeconds(10)
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendToLowPole())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
//                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1);})
                .lineToLinearHeading(droppingPosition0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->{Robot.targetDegree = 138;Servos.Slider.moveInside();})
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.6))
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())



                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[0], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})

                .lineToLinearHeading(pickingPosition, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .addTemporalMarker(()->)
//                .waitSeconds(10)
                .waitSeconds(0.3)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.Wrist.goAutoTop())
//                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendTo(lift.POSITIONS_AUTO[lift.HIGH_POLE], 1);})
                .lineToLinearHeading(droppingPosition0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()->Robot.targetDegree = 138)
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.6))
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())



                .addTemporalMarker(()->Servos.Slider.moveInside())
//                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{lift.extendTo(lift.AUTO_POSITION[0], 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Robot.targetDegree = 0;})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[0], 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;})
//                .addTemporalMarker(()-> )
//                .lineToLinearHeading(pickingPosition)
                .build();

        /**
         * Sequence for going and parking at parking zone 1
         */
        goToP1 = robot.trajectorySequenceBuilder((startToCenter.end()))
//                .addTemporalMarker(() -> turret.setDegree(0))
//                .addTemporalMarker(() -> lift.extendTo(0, 1))
                .addTemporalMarker(() -> Servos.Slider.moveInside())
                .addTemporalMarker(() -> Servos.Wrist.goInit())
                .addTemporalMarker(() -> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING1.getX(), PARKING1.getY(), Math.toRadians(-90)))
//                .addTemporalMarker(() -> turret.setDegree(0))

//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY()-24, Math.toRadians(90)))

                .build();

        /**
         * Sequence for going and parking at parking zone 2
         */
        goToP2 = robot.trajectorySequenceBuilder((startToCenter.end()))
//                .addTemporalMarker(() -> turret.setDegree(0))
//                .addTemporalMarker(() -> Servos.Wrist.goInit())
//                .addTemporalMarker(() -> lift.extendTo(0, 1))
                .addTemporalMarker(() -> Servos.Slider.moveInside())
                .addTemporalMarker(() -> Servos.Wrist.goInit())
                .addTemporalMarker(() -> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING2.getX(), PARKING2.getY(), Math.toRadians(-90)))
//                .addTemporalMarker(() -> turret.setDegree(0))

//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY()-24, Math.toRadians(90)))
                .addTemporalMarker(() -> Servos.Slider.moveInside())
                .addTemporalMarker(() -> Servos.Wrist.goInit())
                .build();

        /**
         * Sequence for going and parking at parking zone 3
         */
        goToP3 = robot.trajectorySequenceBuilder((startToCenter.end()))
//                .addTemporalMarker(() -> turret.setDegree(0))
//                .addTemporalMarker(() -> Servos.Wrist.goInit())
//                .addTemporalMarker(() -> lift.extendTo(0, 1))
                .addTemporalMarker(() -> Servos.Slider.moveInside())
                .addTemporalMarker(() -> Servos.Wrist.goInit())
                .addTemporalMarker(() -> Servos.Gripper.closeGripper())
                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY(), Math.toRadians(-90)))
//                .addTemporalMarker(() -> turret.setDegree(0))

//                .turn(Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(PARKING3.getX(), PARKING3.getY()-24, Math.toRadians(90)))
                .addTemporalMarker(() -> Servos.Slider.moveInside())
                .addTemporalMarker(() -> Servos.Wrist.goInit())
                .build();


//        pick0 = robot.trajectorySequenceBuilder(droppingPosition0)
//                .forward(10)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->lift.extendToHighPole())
//                .waitSeconds(5)
//                .
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4]))
//                .build();


        pick = robot.trajectoryBuilder(droppingPosition)
                .lineToLinearHeading(pickingPosition)
                .build();

        drop = robot.trajectoryBuilder(pickingPosition)
                .lineToLinearHeading(droppingPosition)
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

    }

    @Override
    public void init_loop() {
        super.init_loop();      // inits all bg processes such as timer
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

    @Override
    public void start() {
        super.start();
        turret.reset();
        lift.reset();



        timer.reset();
//        Robot.targetHeight = lift.POSITIONS[lift.LOW_POLE];
        robot.followTrajectorySequence(startToCenter);
        String ParkingZone = "3";                       //Defaults to Parking Zone 3

        if(tagOfInterest != null) {
            if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE1]) {
                ParkingZone = "1";
                robot.followTrajectorySequence(goToP1);

            } else if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE2]) {
                ParkingZone = "2";
                robot.followTrajectorySequence(goToP2);
            } else if (tagOfInterest.id == MATRIX_IDS[PARKING_ZONE3]) {
                ParkingZone = "3";
                robot.followTrajectorySequence(goToP3);
            } else {
                robot.followTrajectorySequence(goToP3);
            }
        }

    }

    boolean preloadLiftFlag = false;
    boolean pickConeFlag = false;
    boolean dropFlag = false;

    @Override
    public void loop() {
        Pose2d poseEstimate = robot.getPoseEstimate();
        robot.update();



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
