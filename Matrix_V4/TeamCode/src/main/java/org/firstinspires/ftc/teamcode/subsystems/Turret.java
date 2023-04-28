package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVars;

import java.util.function.DoubleSupplier;

@Config
public class Turret extends SubsystemBase {

    public static double Kp_turret = 0.06;
    public static double Ki_turret = 0;
    public static double Kd_turret = 0.000;
    public static double Kf_turret = 0; //feedforward, turret no gravity so 0
    public static double targetDegree = 0;
    private final MotorEx motor;
    private final double GEAR_RATIO = 122.0 / 44.0;
    private final double CPR = 8192;                //counts
    public final double CountsPerDegree = CPR * GEAR_RATIO / 360.0;
    private final double ticks_in_degree = CountsPerDegree;
    public DoubleSupplier turretDegreesSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getDegree();
        }
    };
    private final PIDController turretController; //meant for the turret
    ElapsedTime turretProfileTimer;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private MotionProfile turretProfile;
    private double turretPower = 0;


    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = new MotorEx(hardwareMap, "turret");
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);
    }


    @Override
    public void periodic() {
        super.periodic();
        turretController.setPID(Kp_turret, Ki_turret, Kd_turret);
        double currentTurretDegree = getDegree();
        MotionState profileState = turretProfile.get(turretProfileTimer.seconds());
        double turret_pid = turretController.calculate(currentTurretDegree, profileState.getX());
        double ff_turret = 1 * Kf_turret;
        turretPower = ff_turret + turret_pid;
        if (turretController.getPositionError() > 10) {
            if (motor.getVelocity() == 0) {
                setDegree(getNearest90(), 1000, 1000, 1000);
            }
        }
        write();
        if (GlobalVars.turretTelemetry) {
            telemetry.addData("Turret Target: ", targetDegree);
            telemetry.addData("Turret Degrees: ", getDegree());
            telemetry.addData("Turret Motor Counts: ", getPosition());
            telemetry.addData("Ticks in degree: ", ticks_in_degree);
        }
    }

    public void write() {
        motor.set(turretPower);
    }

    public void setDegree(double degree, double maxVel, double maxAccel, double maxJerk) {
        targetDegree = degree;
        turretProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getDegree(), 0, 0), new MotionState(targetDegree, 0, 0), maxVel, maxAccel, maxJerk);
        turretProfileTimer.reset();
    }


    public void reset() {
        motor.resetEncoder();
    }

    public double getNearest90() {
        double currentAngle = getDegree();
        double sign = currentAngle / Math.abs(currentAngle);
        currentAngle = Math.abs(currentAngle);
        double rem = currentAngle % 90;

        if (rem > 45) {
            currentAngle = currentAngle + (90 - rem);
        } else {
            currentAngle = currentAngle - rem;
        }

        return currentAngle * sign;
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public double getDegree() {
        double degree = getPosition() * 1 / ticks_in_degree;
        return degree; // 1/countsperdegree = degrees per count and position is no. of counts
    }


}
