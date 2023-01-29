package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Autonomous
public class RedLeftHigh_Long extends LinearOpMode {
    int cone = 4;
    double sliderPos = 0;
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;
    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timer1 = new ElapsedTime();
    final double MAX_SPEED_AUTO = DriveConstants.MAX_VEL;

    double kp_gyro = 0.04;
    double kd_gyro = 0;
    double ki_gyro = 0;
    double prevError_gyro = 0;

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

    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;

    int[] MATRIX_IDS = {3,7,9};

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d PARKING1 = new Pose2d(-60, -12, Math.toRadians(180));
        Pose2d PARKING2 = new Pose2d(-36, -12, Math.toRadians(180));
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
        Pose2d startPose = new Pose2d(-41.5, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence startToCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->Servos.Wrist.goTop())
                .addTemporalMarker(()->lift.extendToLowPole())
                .lineToConstantHeading(new Vector2d(-36, -63))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->turret.setDegree(-42))
                .lineToConstantHeading(new Vector2d(-36, -12))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->lift.extendToHighPole())
                .addTemporalMarker(()->Servos.Slider.moveSlider(0.4))
                .waitSeconds(1)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();

        TrajectorySequence alignToPoint = drive.trajectorySequenceBuilder(startToCenter.end())
                .addTemporalMarker(()-> {Servos.Slider.moveInside();
        lift.extendTo(lift.AUTO_POSITION[cone], 1);})
                .lineToConstantHeading(new Vector2d(-38, -12))
                        .build();

            waitForStart();

            drive.followTrajectorySequence(startToCenter);
            drive.followTrajectorySequence(alignToPoint);




//        States currentState = States.START_TO_CENTER;
//        lift.extendToHighPole();
//        Servos.Slider.moveInside();
//        Servos.Gripper.openGripper();
//        Servos.Wrist.goGripping();
//        sleep(2000);
//        drive.followTrajectorySequence(startToCenter);
//        drive.followTrajectorySequence(alignToPoint);
////        for(cone=4;cone>=4;cone--) {
//            lift.extendTo(lift.AUTO_POSITION[cone], 1);
//            turret.setDegree(87);
//            sleep(2000);
////            if(turret.getDegree() > 80 && lift.getPosition()[0]<300){
////                Servos.Slider.moveSlider(1);
////                sleep(1000);
//        turret.setDegree(90);
//        sleep(2000);
//        lift.extendTo(lift.AUTO_POSITION[cone], 1);
//        sleep(2000);
//        Servos.Slider.moveSlider(0.9);
//        sliderPos = 0.9;

//                while(Sensors.GripperSensor.getDistanceMM() > 20 && opModeIsActive()){
//                    sliderPos += 0.0009;
//                    Servos.Slider.moveSlider(sliderPos);
//                    telemetry.addData("POs", Sensors.GripperSensor.getDistanceMM());
//                    telemetry.update();
//                }
//
//                while(Sensors.GripperSensor.getDistanceMM() < 19.5 && opModeIsActive()){
//                    sliderPos = sliderPos - 0.0005;
//                    Servos.Slider.moveSlider(sliderPos);
//                }
//        Servos.Gripper.closeGripper();
//                sleep(200);
//                lift.extendToLowPole();
//                sleep(200);
//                turret.setDegree(-42);


//        }
        while(opModeIsActive()){

//            telemetry.addData("cone", cone);
//            telemetry.update();
//
//            drive.update();
//
//            switch (currentState){
//                case START_TO_CENTER:
//                    if(!drive.isBusy()){
//                        Servos.Slider.moveInside();
//                        lift.extendTo(lift.AUTO_POSITION[cone], 1);
////                        telemetry.addLine("Done with Auto");
//                        drive.followTrajectorySequenceAsync(alignToPoint);
//                        currentState = States.PRETO5;
//                    }
//                    break;
//                case PRETO5:
//                    if(!drive.isBusy()){
//                        lift.extendTo(lift.AUTO_POSITION[cone], 1);
//                        turret.setDegree(87);
//                        if(turret.getDegree() > 80 && lift.getPosition()[0]<300){
//                            Servos.Slider.moveSlider(0.8);
//                            timer.reset();
//                            currentState = States.PICK5;
//                        }
//
//                    }
//
//                    break;
//                case PICK5:
//                    if((timer.milliseconds() > 1500 && Sensors.GripperSensor.getDistanceINCH()<2 && Sensors.GripperSensor.getDistanceINCH()>0.5) && (Sensors.GripperSensor.checkRed() || Sensors.GripperSensor.checkBlue())){
//                        Servos.Gripper.closeGripper();
//                        timer.reset();
//                        currentState = States.MOVE5;
//                    }
//                    else if(Sensors.GripperSensor.getDistanceINCH() > 2){
//                        Servos.Slider.moveOutside();
//                    }
//                    else if(Sensors.GripperSensor.getDistanceINCH() < 0.5){
//                        sliderPos-=0.01;
//                        Servos.Slider.moveSlider(sliderPos);
//                    }
//                    break;
//
//                case MOVE5:
//                    if(timer.milliseconds() > 1700){
//                        lift.extendToLowPole();
//                    }
//                    if(timer.milliseconds() > 1800){
//                        timer.reset();
//                        Servos.Slider.moveInside();
//                        lift.extendToHighPole();
//                        if(lift.getPosition()[0] > lift.POSITIONS[lift.MID_POLE]) {
//                            turret.setDegree(-42);
//                            if(turret.getDegree() < -40){
//                                Servos.Slider.moveHalfway();
//                                timer.reset();
//                                currentState = States.DROP5;
//                            }
//                        }
//                    }
//                    break;
//
//                case DROP5:
//                    if(timer.milliseconds()>1000) {
//                        cone--;
//                        if (cone < 0) {
//                            Servos.Gripper.openGripper();
//                            currentState = States.PARK;
//                            break;
//                        } else {
//                            Servos.Gripper.openGripper();
//                            currentState = States.PRETO5;
//                            break;
//                        }
//                    }
//
//
//
//            }

        }
    }

    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        turret.setDegree(0);
    }



    public double gyroPID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_gyro;
        double Ierror = error + prevError_gyro;

        prevError_gyro = error;

        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
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

    enum States{
        START_TO_CENTER,
        PRETO5,
        PICK5,
        MOVE5,
        DROP5,
        PARK
//        DROP5_TO_PICKCONE4,
//        DROP4_TO_PICKCONE3,
//        DROP3_TO_PICKCONE2,
//        DROP2_TO_PICKCONE1,
//        PICK5_TO_DROPCONE5,
//        PICK4_TO_DROPCONE4,
//        DROP3_TO_DROPCONE3,
//        DROP2_TO_DROPCONE2,
//        DROP1_TO_DROPCONE1,
//        DROPCONE1_TO_PARKING
    }
}
