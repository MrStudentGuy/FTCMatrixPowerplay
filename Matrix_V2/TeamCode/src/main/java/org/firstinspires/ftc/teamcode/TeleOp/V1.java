package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TransferClass.poseStorage;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

@TeleOp(name = "-->TELEOP \uD83D\uDC4C\uD83D\uDC4C\uD83D\uDE0D\uD83C\uDFB6\uD83C\uDFB6\uD83D\uDE0E\uD83D\uDE1C\uD83D\uDE2D\uD83E\uDD70\uD83D\uDE08\uD83D\uDC7A\uD83D\uDC7A\uD83E\uDD23\uD83E\uDD23\uD83D\uDE15\uD83D\uDE1C\uD83D\uDE2D\uD83E\uDD70\uD83E\uDD70\uD83D\uDE18")
@Config
public class V1 extends LinearOpMode {
    public static double targetDegree = 0;
    private final double GEAR_RATIO = 10.5 * 122.0 / 18.0;
    private final double CPR = 28;                //counts
    private final double ticks_in_degree = CPR * GEAR_RATIO / 360.0;

    private PIDController controller; //meant for the turret
    public static double Kp = 0.07;
    public static double Ki = 0;
    public static double Kd = 0.001;
    public static double Kf = 0; //feedforward, turret no gravity so 0


    boolean aFlag = false;
    boolean bFlag = false;
    boolean RBFlag = false;
    boolean LBFlag = false;
    boolean AutoCycleFlag = false;
    boolean AutoCycleProceed = false;


    ElapsedTime AutoCycleTimer = new ElapsedTime();

    double pos = 0;

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;

    enum AutoCycleCenterStates {
        IDLE,
        GOGRIPPING,
        RETRACT,
        DROPANTICLOCKWISE,
        DROPCLOCKWISE,
        RETURN
    }

    AutoCycleCenterStates autoCycleCenterStates = AutoCycleCenterStates.IDLE;
    private int liftPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime teleOpTime = new ElapsedTime();
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        sensors = new Sensors(hardwareMap, telemetry);

        controller = new PIDController(Kp, Ki, Kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        Pose2d startPose = poseStorage;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeInInit()) {
            telemetry.addData("Position: ", lift.getPosition()[0] + "," + lift.getPosition()[1]);
            telemetry.update();
            if (gamepad1.dpad_up) {
                liftPos += 2;
            } else if (gamepad1.dpad_down) {
                liftPos -= 2; ///point of fakiure
            }
            lift.extendTo(liftPos, 0.5);
        }
        targetDegree = 0;

        liftPos = 0;

//        lift.reset();
//        turret.reset();
        setInitialPositions();
        teleOpTime.reset();

        while (opModeIsActive()) {
            controller.setPID(Kp, Ki, Kd); //turret cant use default values, too big, nota ccurate, hence custom values


            double currentTurretValue = turret.getDegree();

            double pid = controller.calculate(currentTurretValue, targetDegree); //output power for motor

            double ff = Math.cos(Math.toRadians(targetDegree / ticks_in_degree)) * Kf; //feedforward calculation

            double power = pid + ff; //in future in case

            turret.set(power);

//            telemetry.addData("Current Distance ", currentTurretValue);
//            telemetry.addData("Target Distance ", targetDegree);
////            telemetry.addData("PIDF: ", Kp + ", " + Ki + ", " + Kd + ", " + Kf);
//            telemetry.update();

            double drivePowerThrottle, drivePowerTurn, drivePowerHeading;

            if (gamepad1.right_trigger > 0.3 || gamepad1.left_stick_button || gamepad1.right_stick_button) {       //Turn on slow mode
                drivePowerThrottle = 0.3;
                drivePowerTurn = 0.3;
                drivePowerHeading = 0.2; //slow down - delicate situation
            } else {
                drivePowerThrottle = 1;
                drivePowerTurn = 1; //behaves like strafing (left right)
                drivePowerHeading = 0.7; //(turning)
            }


            //Field Oriented Drive
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(-gamepad1.left_stick_y * drivePowerThrottle, -gamepad1.left_stick_x * drivePowerTurn).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * drivePowerHeading));
            drive.update();//drive is sample mecanum drive


            //UNCOMMENT FOLLOWING FOR ROBOT ORIENTED DRIVE
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


            boolean A = gamepad1.a;                  //x
            boolean B = gamepad1.b;                  //o
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean R3 = gamepad1.right_stick_button;
            boolean L3 = gamepad1.left_stick_button;


//            if(gamepad1.right_stick_button && lift.getPosition()[0] >= lift.POSITIONS[lift.LOW_POLE]){
//                Servos.Slider.moveOutside();
//                sleep(1000);
//                Servos.AlignBar.outside();
//                sleep(1000);
//            }

            boolean UP2 = gamepad2.dpad_up;
            boolean DOWN2 = gamepad2.dpad_down;
            boolean LEFT2 = gamepad2.dpad_left;
            boolean RIGHT2 = gamepad2.dpad_right;

            boolean A2 = gamepad2.a;
            boolean B2 = gamepad2.b;
            boolean X2 = gamepad2.x;
            boolean Y2 = gamepad2.y;


            if(gamepad1.x){
                if(lift.getPosition()[0] >= 0){
                    Servos.Wrist.goInit();
                }
                else{
                    
                }
            }


            if (A2) { //incase finished all 18, auto not used, then use auto stack
                Servos.Wrist.goGripping();
                lift.extendTo(lift.AUTO_POSITION[4], 0.8);
            } else if (B2) {
                Servos.Wrist.goGripping();
                lift.extendTo(lift.AUTO_POSITION[3], 0.8);
            } else if (Y2) {
                Servos.Wrist.goGripping();
                lift.extendTo(lift.AUTO_POSITION[2], 0.8);
            } else if (X2) {
                Servos.Wrist.goGripping();
                lift.extendTo(lift.AUTO_POSITION[1], 0.8);
            }


            if (gamepad2.right_bumper && !AutoCycleFlag) {
                AutoCycleFlag = true;
                AutoCycleTimer.reset();
                autoCycleCenterStates = AutoCycleCenterStates.GOGRIPPING;
            }
            if (gamepad2.start) {
                autoCycleCenterStates = AutoCycleCenterStates.IDLE;
            }
            if (AutoCycleFlag) {
                switch (autoCycleCenterStates) {
                    case IDLE:
                        AutoCycleFlag = false;
                        AutoCycleProceed = false;
                        turret.setDegree(0);
                        lift.extendToLowPole();
                        Servos.Slider.moveInside();
                        Servos.Gripper.openGripper();
                        break;

                    case GOGRIPPING:
                        AutoCycleFlag = true;
                        Servos.Slider.moveOutside();
                        Servos.Gripper.openGripper();
                        lift.extendToGrippingPosition();
                        turret.setDegree(0);
                        Servos.Slider.moveOutside();
                        if (AutoCycleTimer.milliseconds() > 1400) {
                            Servos.Gripper.closeGripper();
                            Servos.Slider.moveHalfway();
                            autoCycleCenterStates = AutoCycleCenterStates.RETRACT;
                            AutoCycleTimer.reset();
                        }
                        break;

                    case RETRACT:
                        if (gamepad2.right_bumper) {
                            lift.extendToHighPole();
                            sleep(500);
                            turret.setDegreeHighPower(150);
                            sleep(800);
                            turret.setDegree(170);
                            sleep(400);
                            Servos.Slider.moveSlider(0.65);
                            sleep(800);
                            Servos.Wrist.goTop();
                            sleep(200);
                            Servos.Wrist.goGripping();
                            sleep(500);
                            Servos.Gripper.setPosition(1);
//                Servos.AlignBar.inside();
                            sleep(50);
                            Servos.Slider.moveHalfway();
                            sleep(50);
                            autoCycleCenterStates = AutoCycleCenterStates.IDLE;
//                        sleep(500);

                            AutoCycleTimer.reset();
                        } else if (gamepad2.left_bumper) {
                            autoCycleCenterStates = AutoCycleCenterStates.GOGRIPPING;
                            AutoCycleTimer.reset();
                        }
                        break;

                }

            } else {
                Servos.Slider.moveSlider(Math.abs(gamepad1.left_trigger));
            }

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


            if (RB && !RBFlag) {
                RBFlag = true; //only once
                if (Objects.equals(Servos.Gripper.gripperState, "OPEN")) {
                    Servos.Gripper.closeGripper();
                } else if (Objects.equals(Servos.Gripper.gripperState, "CLOSED")) {
                    Servos.Gripper.openGripper();
//                    if(lift.getPosition()[0] > lift.POSITIONS[lift.LOW_POLE]) {
//                        Servos.AlignBar.moveTo(0.7);
//                        if (Servos.AlignBar.getPosition() > 0.4) {
//                            Servos.Slider.moveOutside();
//                            sleep(1000);
//                            Servos.AlignBar.inside();
//                            sleep(500);
//                            Servos.Slider.moveInside();
//                            sleep(300);
//                        }
//                    }
                }
            }

            if (LB && !LBFlag) {
                LBFlag = true;
                if (Objects.equals(Servos.Wrist.wristState, "GRIPPING")) {
                    Servos.Wrist.goTop();
                } else if (Objects.equals(Servos.Wrist.wristState, "TOP")) {
                    Servos.Wrist.goGripping();
                } else {
                    Servos.Wrist.goGripping();
                }
            }
            telemetry.addData("LbFlag", LBFlag);

            if (!RB) {
                RBFlag = false;
            }
            if (!LB) {
                LBFlag = false;
            }

            if ((A || LEFT2) && !aFlag) {
                aFlag = true;
                targetDegree += 45;
                setTurret();

            } else if (!A && !LEFT2) {
                aFlag = false;
            }

            if ((B || RIGHT2) && !bFlag) {
                bFlag = true;
                targetDegree -= 45;
                setTurret();
            } else if (!B && !RIGHT2) {
                bFlag = false;
            }


            if (Sensors.GripperSensor.getDistanceMM() < 25 && Sensors.GripperSensor.getDistanceMM() > 10 && Servos.Gripper.gripperState != "CLOSED" && Servos.Wrist.wristState == "GRIPPING") {
                gamepad1.rumble(0.5, 0.5, 100);
            }

//            if(teleOpTime.seconds())


//            lift.extendTo((int) liftPos);


            telemetry.addData("Currents: ", lift.getCurrent()[0] + ", " + lift.getCurrent()[1]);
            telemetry.addData("Positions: ", lift.getPosition()[0] + ", " + lift.getPosition()[1]);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }


    private void setInitialPositions() {
        lift.extendTo(0, 0.5);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        pos = 0;
        turret.setDegree(0);
    }

    private void setTurret() {
        if (lift.getPosition()[0] < lift.SAFE_POSITION) {
//do nothing
        } else {
            turret.setDegree((int) (pos));
        }
    }
}
