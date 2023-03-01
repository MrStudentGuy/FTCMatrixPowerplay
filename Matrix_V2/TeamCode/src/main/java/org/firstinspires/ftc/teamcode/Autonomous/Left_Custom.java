package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class Left_Custom extends LinearOpMode {


    Pose2d pose;
    double previousHeading = 0;
    double integratedHeading = 0;
    PIDController xController;
    PIDController yController;
    PIDController hController;


    public static double Kp_x = 0.11;
    public static double Ki_x = 0.01;
    public static double Kd_x = 0.01;

    public static double Kp_y = 0.13;
    public static double Ki_y = 0.01;
    public static double Kd_y = 0.01;

    public static double Kp_h = 0.04;
    public static double Ki_h = 0.1;
    public static double Kd_h = 0.001;

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double hTarget = 0;

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

    boolean preloadLiftFlag = false;
    boolean pickConeFlag = false;
    boolean dropFlag = false;

    Left_Custom.State currentState = Left_Custom.State.STARTTOCENTER;

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

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);
        liftController = new PIDController(Kp_lift, Ki_lift, Kd_lift);
        xController = new PIDController(Kp_x, Ki_x, Kd_x);
        yController = new PIDController(Kp_y, Ki_y, Kd_y);
        hController = new PIDController(Kp_h, Ki_h, Kd_h);

        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        drive = new SampleMecanumDrive(hardwareMap, telemetry);

        Servos.Gripper.closeGripper();
        Servos.Wrist.goInit();
        Servos.AlignBar.inside();
//        Pose2d startPose = new Pose2d(-32, -63.3, Math.toRadians(180));
//        drive.setPoseEstimate(startPose);



        waitForStart();

        turret.reset();
        lift.reset();


//        targetDegree = 0;
//        targetHeight = 0;
        timer.reset();
        Servos.Wrist.goGripping();
         xTarget = 2;
         yTarget = -51;



        while(opModeIsActive()){






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
                lift.setPower(liftPower);


            telemetry.addData("Lift Current Position: ", currentLiftHeight);
            telemetry.addData("Lift Target Position: ", targetHeight);
            telemetry.addData("Turret Current Position: ", currentTurretDegree);
            telemetry.addData("Turret Target Position: ", targetDegree);

            telemetry.update();

            xController.setPID(Kp_x, Ki_x, Kd_x);
            yController.setPID(Kp_y, Ki_y, Kd_y);
            hController.setPID(Kp_h, Ki_h, Kd_h);

            pose = drive.getPoseEstimate();
            double heading =  getIntegratedHeading(pose);

            double xPID = xController.calculate(pose.getX(), xTarget);
            double yPID = yController.calculate(pose.getY(), yTarget);
            double hPID = hController.calculate(heading, hTarget);


            if(timer.milliseconds() > 0){
                Pose2d power = new Pose2d(xPID, yPID, hPID);
                drive.setWeightedDrivePower(power);
                drive.update();
            }
            if(timer.milliseconds() > 200){
                targetDegree = -140;
            }
            if(timer.milliseconds() > 2000){
                targetHeight = lift.POSITIONS[lift.HIGH_POLE];
            }
//            if(drive.isBusy()) {
                telemetry.addLine("Is busy;");

//            }


            telemetry.addData("Heading Power: ", hPID);
            telemetry.addData("Integ Heading: ", heading);

        }
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

    public double getIntegratedHeading(Pose2d poseEstimate){
        double currentHeading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
        double deltaHeading = currentHeading - previousHeading;

        if(deltaHeading < -180){
            deltaHeading+=360;
        }
        else if(deltaHeading >= 180){
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;

        previousHeading = currentHeading;

        return integratedHeading;
    }
}
