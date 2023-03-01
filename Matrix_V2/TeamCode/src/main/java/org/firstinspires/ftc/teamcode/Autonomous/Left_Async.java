package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Left_Async extends OpMode {


    final Pose2d droppingPosition0 = new Pose2d(-37.4, -12, Math.toRadians(180));
    final Pose2d droppingPosition = new Pose2d(-37.4, -12.00, Math.toRadians(180));
    final Pose2d pickingPosition = new Pose2d(-49, -12, Math.toRadians(180));


    Lift lift = null;
    Servos servos = null;
    Turret turret = null;

    enum State{
        STARTTOCENTER,
        PICK,
        DROP, PARK
    }


    State currentState = State.STARTTOCENTER;

    ElapsedTime timer = new ElapsedTime();

    private PIDController turretController; //meant for the turret
    private PIDController liftController;

    public static double Kp_turret = 0.1;
    public static double Ki_turret = 0.1;
    public static double Kd_turret = 0.001;
    public static double Kf_turret = 0; //feedforward, turret no gravity so 0

    public static double Kp_lift = 0.1;
    public static double Ki_lift = 0;
    public static double Kd_lift = 0.001;
    public static double Kf_lift = 0;

    public static double targetDegree = 0;
    public static double targetHeight = 0;

    private final double GEAR_RATIO_turret = 10.5 * 122.0 / 18.0;
    private final double CPR_turret = 28;             //counts
    private final double ticks_in_degree_turret = CPR_turret * GEAR_RATIO_turret / 360.0;
    private final double ticks_in_degree_lift = (10.5 * 28)/360;

    SampleMecanumDrive drive;


    int counter = 4;
    Trajectory startToCenter;
    Trajectory pick0;
    Trajectory pick;
    Trajectory drop;
    @Override
    public void init() {
        PhotonCore.enable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);
        liftController = new PIDController(Kp_lift, Ki_lift, Kd_lift);

        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        drive = new SampleMecanumDrive(hardwareMap, telemetry);

        Servos.Gripper.closeGripper();
        Servos.Wrist.goInit();
        Servos.AlignBar.inside();



        Pose2d startPose = new Pose2d(-32, -63.3, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        startToCenter = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(droppingPosition0)
                .build();

        pick0 = drive.trajectoryBuilder(droppingPosition0)
                .lineToLinearHeading(pickingPosition)
                .build();


        pick = drive.trajectoryBuilder(droppingPosition)
                .lineToLinearHeading(pickingPosition)
                .build();

        drop = drive.trajectoryBuilder(pickingPosition)
                .lineToLinearHeading(droppingPosition)
                .build();

    }

    @Override
    public void start() {
        super.start();
        turret.reset();
        lift.reset();



        targetDegree = 0;
        targetHeight = 0;
        timer.reset();
        drive.followTrajectoryAsync(startToCenter);
        Servos.Wrist.goGripping();
//        Servos.Slider.moveOutside();
//        targetHeight = lift.POSITIONS[lift.LOW_POLE];
//        Servos.Wrist.goTop();

    }

    boolean preloadLiftFlag = false;
    boolean pickConeFlag = false;
    boolean dropFlag = false;

    @Override
    public void loop() {



        switch (currentState){
            case STARTTOCENTER:
                if(timer.milliseconds() > 600){
                    targetDegree = -140;
                }
                if (timer.milliseconds() > 2100) {
                    Servos.Slider.moveInside();
                    targetHeight = lift.POSITIONS[lift.HIGH_POLE] - 50;
                    Servos.Wrist.goTop();
//                targetDegree = -144;
                }
                if(lift.getPosition()[0] > lift.POSITIONS[lift.HIGH_POLE] - 400 && !preloadLiftFlag){
                    Servos.Slider.moveSlider(0.55);
                    timer.reset();
                    preloadLiftFlag = true;
                }
                if(timer.milliseconds() > 600 && preloadLiftFlag){
                    Servos.Gripper.openGripper();
                    Servos.Slider.moveInside();
                    timer.reset();
                    drive.followTrajectoryAsync(pick);
                    currentState = State.PICK;
                }
                break;

            case PICK:
                if(timer.milliseconds() > 100 && timer.milliseconds() < 500){
                    targetDegree = 0;
                    targetHeight = lift.AUTO_POSITION[counter];
                    Servos.Wrist.goGripping();
                }
                if(timer.milliseconds() > 2000 && lift.getPosition()[0] < 200 && !pickConeFlag){
                    counter--;
                    timer.reset();
                    pickConeFlag = true;
                    Servos.Slider.moveOutside();
                }
                if(pickConeFlag) {
                    if (timer.milliseconds() > 1000) {
                        dropFlag = false;
                        Servos.Gripper.closeGripper();
                    }
                    if (timer.milliseconds() > 1500) {
                        targetHeight = lift.POSITIONS[lift.LOW_POLE];
                        Servos.Wrist.goTop();
                    }
                    if (timer.milliseconds() > 2000) {
                        Servos.Slider.moveInside();
                    }
                    if (timer.milliseconds() > 2500) {
//                        targetHeight = 0;
                        targetDegree = -140;
                    }
                    if(timer.milliseconds() > 2800){
                        timer.reset();
                        drive.followTrajectoryAsync(drop);
                        currentState = State.DROP;
                    }
                }
                break;

            case DROP:
                if (timer.milliseconds() > 2100) {
                    Servos.Slider.moveInside();
                    targetHeight = lift.POSITIONS[lift.HIGH_POLE];
                    Servos.Wrist.goTop();
//                targetDegree = -144;
                }
                if(lift.getPosition()[0] > lift.POSITIONS[lift.HIGH_POLE] - 400 && !dropFlag){
                    Servos.Slider.moveSlider(0.55);
                    timer.reset();
                    dropFlag = true;
                }
                if(timer.milliseconds() > 1000 && dropFlag){
                    Servos.Gripper.openGripper();
                    Servos.Slider.moveInside();
                    pickConeFlag = false;
                    timer.reset();
                    drive.followTrajectoryAsync(pick);
                    if(counter >= 0)
                    currentState = State.PICK;
                    else
                        currentState = State.PARK;
                }

                break;

        }

        Pose2d poseEstimate = drive.getPoseEstimate();


        turretController.setPID(Kp_turret, Ki_turret, Kd_turret);
        liftController.setPID(Kp_lift, Ki_lift, Kd_lift);


        double currentTurretDegree = turret.getDegree();
        double turret_pid = turretController.calculate(-currentTurretDegree, targetDegree);
        double ff_turret = Math.cos(Math.toRadians(targetDegree / ticks_in_degree_turret)) * Kf_turret;
        double turretPower = ff_turret + turret_pid;


        double currentLiftHeight = lift.getPosition()[0];
        double lift_pid = liftController.calculate(currentLiftHeight, targetHeight);
        double ff_lift = Math.cos(Math.toRadians(targetHeight / ticks_in_degree_lift)) * Kf_lift;
        double liftPower = ff_lift + lift_pid;


        turret.set(turretPower);
        if(!drive.isBusy()) {
            lift.setPower(liftPower);
        }


        telemetry.addData("Counter: ", counter);
        telemetry.addData("Lift Current Position: ", currentLiftHeight);
        telemetry.addData("Lift Target Position: ", targetHeight);
        telemetry.addData("Turret Current Position: ", currentTurretDegree);
        telemetry.addData("Turret Target Position: ", targetDegree);

        telemetry.update();
        if(drive.isBusy()) {
            drive.update();
        }
    }


        }
