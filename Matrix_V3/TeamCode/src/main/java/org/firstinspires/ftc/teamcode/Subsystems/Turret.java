package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.GlobalVars;

import java.util.function.DoubleSupplier;

@Config
public class Turret extends SubsystemBase {

    private MotorEx motor;
    private final double GEAR_RATIO = 122.0/44.0 ;
    private final double CPR = 8192;                //counts
    public final double CountsPerDegree = CPR * GEAR_RATIO/360.0;
    private final double ticks_in_degree = CountsPerDegree;

    private PIDController turretController; //meant for the turret
    public static double Kp_turret = 0.06;
    public static double Ki_turret = 0;
    public static double Kd_turret = 0.000;
    public static double Kf_turret = 0; //feedforward, turret no gravity so 0
    public static double targetDegree = 0;
    private double turretPower = 0;

    public DoubleSupplier turretDegreesSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getDegree();
        }
    };

    HardwareMap hardwareMap;;
    Telemetry telemetry;


    public Turret(HardwareMap hardwareMap, String deviceName, Telemetry telemetry){
        motor = new MotorEx(hardwareMap, deviceName);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);
    }


    @Override
    public void periodic() {
        super.periodic();
        turretController.setPID(Kp_turret, Ki_turret, Kd_turret);
        double currentTurretDegree = getDegree();
        double turret_pid = turretController.calculate(currentTurretDegree, targetDegree);
        double ff_turret = Math.cos(Math.toRadians(targetDegree / ticks_in_degree)) * Kf_turret;
        turretPower = ff_turret + turret_pid;

        
        
        

        write();
        if(GlobalVars.turretTelemetry){
            telemetry.addData("Turret Target: ", targetDegree);
            telemetry.addData("Turret Degrees: ", getDegree());
            telemetry.addData("Turret Motor Counts: ", getPosition());
            telemetry.addData("Ticks in degree: ", ticks_in_degree);
            telemetry.update();
        }
    }

    public void write(){
        motor.set(turretPower);
    }

    public void setDegree(double degree){
        targetDegree = degree;
    }


    public void reset(){
        motor.resetEncoder();
    }

    public double getPosition(){
        return motor.getCurrentPosition();
    }

    public double getDegree() {
        double degree = getPosition() * 1/ticks_in_degree;
        return degree; // 1/countsperdegree = degrees per count and position is no. of counts
    }




}
