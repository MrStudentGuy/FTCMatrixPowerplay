package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Autonomous.TrajectoriySequences.TrajectorySequences;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
@Disabled
public class V2 extends LinearOpMode {

    boolean aFlag = false;
    boolean bFlag = false;
    boolean RBFlag = false;
    boolean LBFlag = false;

    double pos = 0;

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    RevColorSensorV3 gripperSensor = null;

    ElapsedTime timer0 = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    boolean flag0 = false;
    boolean flag1 = false;

    private int liftPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(12.00, 60.00, Math.toRadians(180));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        while(opModeInInit()){
            telemetry.addData("Position: ", lift.getPosition()[0] + "," + lift.getPosition()[1]);
            telemetry.update();
            if(gamepad1.dpad_up){
                liftPos+=2;
            }
            else if(gamepad1.dpad_down){
                liftPos-=2;
            }
            lift.extendTo((int)liftPos, 0.5);
        }

        liftPos = 0;

//        lift.reset();
        turret.reset(

        );
        setInitialPositions();


        while(opModeIsActive()){
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(-gamepad1.left_stick_y * 0.75, -gamepad1.left_stick_x * 0.75).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * 0.85));
//            drive.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x,
                            -gamepad2.right_stick_x * 0.7
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();




            boolean A = gamepad1.a;
            boolean B = gamepad1.b;
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean R3 = gamepad1.right_stick_button;
            boolean L3 = gamepad1.left_stick_button;

//            if(gamepad1.right_trigger>0.8){
//                drive.followTrajectorySequence(trajSeq);
//            }
            if (UP) {
                Servos.Wrist.goGripping();
                lift.extendToHighPole();
            } else if (RIGHT) {
                Servos.Wrist.goGripping();
                Servos.Gripper.openGripper();
                lift.extendToGrippingPosition();
            } else if (DOWN) {
                Servos.Wrist.goGripping();
                lift.extendToLowPole();
            } else if (LEFT) {
                Servos.Wrist.goGripping();
                lift.extendToMidPole();
            }


            if(RB && !RBFlag){
                RBFlag = true;
                if(Servos.Gripper.gripperState == "OPEN"){
                    Servos.Gripper.closeGripper();
                }
                else if(Servos.Gripper.gripperState == "CLOSED"){
                    flag0 = false;
                    flag1 = false;
                    Servos.Gripper.openGripper();
                }
            }

            if(lift.getPosition()[0] < 0){
                if(gripperSensor.red() > 120 && gripperSensor.getDistance(DistanceUnit.INCH) < 1.0){
                    Servos.Gripper.closeGripper();
                    if(!flag0){
                        flag0 = true;
                        timer0.reset();
                    }
                    if(timer0.milliseconds() > 1000){
                        if(!flag1) {
                            flag1 = true;
                            timer1.reset();
                        }
                        if(timer1.milliseconds() > 500){
                            Servos.Wrist.goInit();
                        }
                        lift.extendTo(200, 1);

                    }

                }
            }

            if(LB && !LBFlag){
                LBFlag = true;
                if(Servos.Wrist.wristState == "GRIPPING"){
                    Servos.Wrist.goTop();
                }
                else if(Servos.Wrist.wristState == "TOP"){
                    Servos.Wrist.goGripping();
                }
                else{
                    Servos.Wrist.goGripping();
                }
            }
            telemetry.addData("LbFlag", LBFlag);

            if(!RB){
                RBFlag = false;
            }
            if(!LB){
                LBFlag = false;
            }

            if(gamepad1.right_stick_x > 0.5){
                pos-=2;
            }
            else if(gamepad1.right_stick_x < -0.5){
                pos+=2;
            }

//            if (A && !aFlag) {
//                aFlag = true;
//                pos += 45;
//
//            } else if (!A) {
//                aFlag = false;
//            }
//
//            if (B && !bFlag) {
//                bFlag = true;
//                pos  -=45;
//            } else if (!B) {
//                bFlag = false;
//            }


            Servos.Slider.moveSlider(1-Math.abs(gamepad1.left_trigger));
            if(lift.getPosition()[0] > 0 && Servos.Wrist.wristState != "INIT"){
                turret.setDegree((int)pos);
            }

//            lift.extendTo((int) liftPos);

            telemetry.addData("Red Color: ", gripperSensor.red());
            telemetry.addData("Distance: ", gripperSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Values: ", lift.getPosition()[0] + ", " + lift.getPosition()[1]);
            telemetry.addData("Currents: ", lift.getCurrent()[0] + ", " + lift.getCurrent()[1]);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


    private void setInitialPositions(){
        lift.extendTo(0, 0.5);
//        lift.extendTo(0);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        pos = 0;
        turret.setDegree(0);
    }

    private void setTurret(){
        if(lift.getPosition()[0] < lift.SAFE_POSITION){

        }
        else{
            turret.setDegree((int) (pos));
        }
    }
}
