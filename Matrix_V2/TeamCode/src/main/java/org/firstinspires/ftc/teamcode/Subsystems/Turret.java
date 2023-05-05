package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Turret {
    private DcMotorEx motor;
    private final double GEAR_RATIO = 122.0/44.0;
    private final double CPR = 8192; //28               //counts
    public final double CountsPerDegree = CPR * GEAR_RATIO/360.0;

    public ElapsedTime turretProfileTimer;
    public MotionProfile turretProfile;

    public static double maxVel = 1800, maxAccel = 1000, maxJerk = 0;

    public Turret(HardwareMap hardwareMap, String deviceName, Telemetry telemetry){
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        turretProfileTimer = new ElapsedTime();
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetDegree(double degree){
        double targetDegree = degree;
        turretProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getDegree(), 0, 0), new MotionState(targetDegree, 0, 0), maxVel, maxAccel, maxJerk);
        turretProfileTimer.reset();
    }

    public void setDegree(double degree){
        double counts = degree * CountsPerDegree;
        setPosition((int)counts); //calls set position
    }

    public void setDegreeHighPower(double degree){
        double counts = degree * CountsPerDegree;
        setPositionHighPower((int)counts);
    }

    public void setPosition(int position){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.25);
    }

    public void setPositionHighPower(int position){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
    }

    public void set(double power){
        power = Range.clip(power, -0.7, 0.7);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getPosition(){
        return motor.getCurrentPosition();
    }

    public double getDegree() {
        double degree = getPosition() * 1/CountsPerDegree;
        return degree; // 1/countsperdegree = degrees per count and position is no. of counts
    }

    public double getCurrent(){
        return motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
