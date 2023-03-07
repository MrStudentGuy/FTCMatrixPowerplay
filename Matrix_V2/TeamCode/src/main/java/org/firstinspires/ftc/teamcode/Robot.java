package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TransferClass.poseStorage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
public class Robot extends SampleMecanumDrive {

    Lift robotLift = null;
    Turret robotTurret = null;
    Servos robotServos = null;
    Telemetry telemetry = null;

    private PIDController turretController; //meant for the turret
    private PIDController liftController;

    public static double Kp_turret = 0.06;
    public static double Ki_turret = 0;
    public static double Kd_turret = 0.002;
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


    public Robot(HardwareMap hardwareMap, Telemetry localtelemetry, Lift lift, Turret turret, Servos servos) {
        super(hardwareMap, localtelemetry);
        robotTurret = turret;
        robotLift = lift;
        robotServos = servos;
        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);
//        liftController = new PIDController(Kp_lift, Ki_lift, Kd_lift);
        telemetry = localtelemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void update() {
        super.update();
        Pose2d pose = super.getPoseEstimate();
        poseStorage = pose;

        turretController.setPID(Kp_turret, Ki_turret, Kd_turret);
//        liftController.setPID(Kp_lift, Ki_lift, Kd_lift);


        double currentTurretDegree = robotTurret.getDegree();
        double turret_pid = turretController.calculate(currentTurretDegree, targetDegree);
        double ff_turret = Math.cos(Math.toRadians(targetDegree / ticks_in_degree_turret)) * Kf_turret;
        double turretPower = ff_turret + turret_pid;


//        double currentLiftHeight = robotLift.getPosition()[0];
//        double lift_pid = liftController.calculate(currentLiftHeight, targetHeight);
//        double ff_lift = Math.cos(Math.toRadians(targetHeight / ticks_in_degree_lift)) * Kf_lift;
//        double liftPower = ff_lift + lift_pid;


        robotTurret.set(turretPower);
//        robotLift.setPower(liftPower);


//        telemetry.addData("Counter: ", counter);
        telemetry.addData("Lift Current Positions: ", robotLift.getPosition()[0] + "::::" + robotLift.getPosition()[1]);
        telemetry.addData("Lift Target Positions: ", robotLift.getTarget()[0] + "::::" + robotLift.getTarget()[1]);
//        telemetry.addData("Lift Target Position: ", targetHeight);

        telemetry.addData("Turret Current Position: ", currentTurretDegree);
        telemetry.addData("Turret Target Position: ", targetDegree);

        telemetry.update();
    }
}
