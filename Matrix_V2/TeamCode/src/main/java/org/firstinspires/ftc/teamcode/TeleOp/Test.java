package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp
@Config
public class Test extends LinearOpMode {
    boolean aFlag = false;
    boolean bFlag = false;
    boolean RBFlag = false;
    boolean LBFlag = false;
    double pos = 0;
    double liftPos = 0;
    public static double posA = 0;
    Turret turret;
    Lift lift;
    Servos servos;



    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        lift = new Lift(hardwareMap, telemetry);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeInInit()){
            if(gamepad1.dpad_up){
                liftPos+=5;
            }
            else if(gamepad1.dpad_down){
                liftPos-=5;
            }
            lift.extendTo((int)liftPos, 1);
        }

        lift.reset();


        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
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

            if (UP) {
                Servos.Wrist.goTop();
                liftPos = lift.POSITIONS[lift.HIGH_POLE];
            } else if (RIGHT) {
                Servos.Wrist.goGripping();
                liftPos = lift.POSITIONS[lift.GRIPPING_POSITION];
            } else if (DOWN) {
                Servos.Wrist.goGripping();
                liftPos = lift.POSITIONS[lift.LOW_POLE];
            } else if (LEFT) {
                Servos.Wrist.goGripping();
                liftPos = lift.POSITIONS[lift.MID_POLE];
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
            }

            if(!RB){
                RBFlag = false;
            }
            if(!LB){
                LBFlag = false;
            }

            if (A && !aFlag) {
                aFlag = true;
                pos += 90;
            } else if (!A) {
                aFlag = false;
            }

            if (B && !bFlag) {
                bFlag = true;
                pos -= 90;
            } else if (!B) {
                bFlag = false;
            }


            turret.setDegree((int) (pos + posA));
            lift.extendTo((int) liftPos, 1);


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Turret Position: ", turret.getPosition());
            telemetry.addData("Lift Positions: ", lift.getPosition()[0] + ", " + lift.getPosition()[1]);
            telemetry.addData("Lift Currents: ", lift.getCurrent()[0] + ", " + lift.getCurrent()[1]);
            telemetry.update();
        }

    }


    public void rotateTo(double position){
        turret.setDegree(position);
    }


}
