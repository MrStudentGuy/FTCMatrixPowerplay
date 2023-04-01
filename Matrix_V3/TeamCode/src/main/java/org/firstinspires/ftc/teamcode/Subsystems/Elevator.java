package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVars;

public class Elevator extends SubsystemBase {

    MotorEx leftMotor, rightMotor;
    HardwareMap hardwareMap;;
    Telemetry telemetry;

    public static double targetCounts = 100;
    private final double ticks_in_degree = (10.5 * 28)/360;

    private PIDController controller;
    public static double Kp = 0.004;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.1;

    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
    public final int SAFE_POSITION = 0;
    public final int[] POSITIONS = {-380, 672, 1510, 2250};  //Junction Height for dropping cones normally
    public final int[] AUTO_POSITION = {-320, -250, -115, -20, 75};  //Cone Stack heights

    public final int[] POSITIONS_AUTO = {-380, 672, 1150, 1850};   //Junction Height for auto
//    {-420, -374, -334, -160, -35};

    int liftPosition = 0;
    private double power = 0;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftMotor = new MotorEx(hardwareMap, "leftMotor");
        rightMotor = new MotorEx(hardwareMap, "rightMotor");

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        rightMotor.setPositionTolerance(10);
        leftMotor.setPositionTolerance(10);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        controller.setPID(Kp, Ki, Kd);
    }


    @Override
    public void periodic() {
        super.periodic();
        double currentCounts = rightMotor.getCurrentPosition();
        double pid = controller.calculate(currentCounts, targetCounts);

        double ff = Math.cos(Math.toRadians(targetCounts/ticks_in_degree)) * Kf;

        power = pid + ff;


        if(GlobalVars.elevatorTelemetry){
            telemetry.addData("Elevator Position: ", getPosition()[0] + " and " + getPosition()[1]);
        }
    }

    public void write(){
        this.setPower(power);
    }


    public void setPower(double power){
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void extendTo(double pos){
        targetCounts = pos;
    }

    public void extendToHigh(){

        if(GlobalVars.currentOpModeType == GlobalVars.runMode.TELEOP) {
            extendTo(POSITIONS[HIGH_POLE]);
        }
        else{
            extendTo(POSITIONS_AUTO[HIGH_POLE]);
        }
    }

    public void extendToMid(){
        if(GlobalVars.currentOpModeType == GlobalVars.runMode.TELEOP) {
            extendTo(POSITIONS[MID_POLE]);
        }
        else{
            extendTo(POSITIONS_AUTO[MID_POLE]);
        }
    }

    public void extendToLow(){
        if(GlobalVars.currentOpModeType == GlobalVars.runMode.TELEOP) {
            extendTo(POSITIONS[LOW_POLE]);
        }
        else{
            extendTo(POSITIONS_AUTO[LOW_POLE]);
        }
    }

    public void extendToGripping(){
        if(GlobalVars.currentOpModeType == GlobalVars.runMode.TELEOP) {
            extendTo(POSITIONS[GRIPPING_POSITION]);
        }
        else{
            extendTo(POSITIONS_AUTO[GRIPPING_POSITION]);
        }
    }

    public void goCone(int coneNumber){
        extendTo(AUTO_POSITION[coneNumber - 1]);
    }

    public double[] getPosition(){
        return new double[] {leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()};
    }

    public void reset(){
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

//    public double[] getCurrent(){
//        return new double[] {leftMotor.motor}
//    }


}
