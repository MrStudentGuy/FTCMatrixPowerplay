package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.util.utilitiyFunctionsMatrix.angleWrap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Autonomous.TrajectoriySequences.TrajectorySequences;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp
@Disabled
public class V3 extends LinearOpMode {

    boolean aFlag = false;
    boolean bFlag = false;
    boolean RBFlag = false;
    boolean LBFlag = false;

    public static double kp_gyro = 0.021;
    public static double kd_gyro = 0.01;
    public static double ki_gyro = 0.03;
    double prevError_gyro = 0;

    double previousHeading = 0;
    double integratedHeading = 0;

    double pos = 0;

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;


    private int liftPos = 0;
    private double headingTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

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
//        turret.reset();
        setInitialPositions();


        while(opModeIsActive()){
            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = getIntegratedHeading(poseEstimate);

            if(gamepad1.right_stick_x > 0.5){
                headingTarget-=5;
            }
            else if(gamepad1.right_stick_x < -0.5){
                headingTarget+=5;
            }

            double headingPower = gyroPID(heading, headingTarget);
            headingPower = Range.clip(headingPower, -1.0, 1.0);



            Vector2d input = new Vector2d(-gamepad1.left_stick_y * 0.8, -gamepad1.left_stick_x * 0.4).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -headingPower));
            drive.update();

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * 0.4,
//                            -gamepad1.left_stick_x * 0.4,
//                            -gamepad1.right_stick_x * 0.6
//                    )
//            );
//
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();




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
                Servos.Wrist.goTop();
                lift.extendToHighPole();
            } else if (RIGHT) {
                Servos.Wrist.goGripping();
                lift.extendToGrippingPosition();
            } else if (DOWN) {
                Servos.Wrist.goTop();
                lift.extendToLowPole();
            } else if (LEFT) {
                Servos.Wrist.goTop();
                lift.extendToMidPole();
            }


            if(RB && !RBFlag){
                RBFlag = true;
                if(Servos.Gripper.gripperState == "OPEN"){
                    Servos.Gripper.closeGripper();
                }
                else if(Servos.Gripper.gripperState == "CLOSED"){
                    Servos.Gripper.openGripper();
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

            if (A && !aFlag) {
                aFlag = true;
                pos += 45;
                setTurret();

            } else if (!A) {
                aFlag = false;
            }

            if (B && !bFlag) {
                bFlag = true;
                pos  -=45;
                setTurret();
            } else if (!B) {
                bFlag = false;
            }


            Servos.Slider.moveSlider(Math.abs(gamepad1.left_trigger));


//            lift.extendTo((int) liftPos);


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

    private double gyroPID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_gyro;
        double Ierror = error + prevError_gyro;

        prevError_gyro = error;

        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
    }


    private double getIntegratedHeading(Pose2d poseEstimate) {
        double currentHeading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
        double deltaHeading = currentHeading - previousHeading;

        if(deltaHeading < -180){
            deltaHeading +=360;
        }
        else if(deltaHeading >= 180){
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}

