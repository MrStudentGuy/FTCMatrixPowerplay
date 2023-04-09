package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVars;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

@Config
public class Elevator extends SubsystemBase {

    MotorEx leftMotor, rightMotor;
    HardwareMap hardwareMap;;
    Telemetry telemetry;

    public static double targetCounts = 0;
    private final double ticks_in_degree = (10.5 * 28)/360;

    private PIDController leftController;
    private PIDController rightController;
    public static double Kp = 0.03;
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
    private double leftPower = 0;
    private double rightPower = 0;

    DoubleSupplier turretAngleSupplier;
    IntSupplier wristStateSupplier;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry, DoubleSupplier turretAngleSupplier, IntSupplier wristStateSupplier){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.turretAngleSupplier = turretAngleSupplier;
        this.wristStateSupplier = wristStateSupplier;
        leftController = new PIDController(Kp, Ki, Kd);
        rightController = new PIDController(Kp, Ki, Kd);
        leftMotor = new MotorEx(hardwareMap, "leftMotor");
        rightMotor = new MotorEx(hardwareMap, "rightMotor");

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

//        rightMotor.setPositionTolerance(10);
//        leftMotor.setPositionTolerance(10);

        leftController.setTolerance(1);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        leftController.setPID(Kp, Ki, Kd);
    }


    @Override
    public void periodic() {
        super.periodic();
        leftController.setPID(Kp, Ki, Kd);
        rightController.setPID(Kp, Ki, Kd);
        double rightCurrentCounts = rightMotor.getCurrentPosition();
        double leftCurrentCounts = leftMotor.getCurrentPosition();
        double leftPid = leftController.calculate(leftCurrentCounts, targetCounts);
        double rightPid = rightController.calculate(rightCurrentCounts, targetCounts);

        double ff = Math.cos(Math.toRadians(targetCounts/ticks_in_degree)) * Kf;
//        telemetry.addData("Position: ", currentCounts);
        leftPower = leftPid + ff;
        rightPower = rightPid + ff;


        //TODO -------------------------------TEST THIS-------------------------------------------------------
        if(Math.abs(turretAngleSupplier.getAsDouble()) < 15 || wristStateSupplier.getAsInt() == 0){
            if(getPosition()[0] < 0 || getPosition()[1] < 0){
                leftPower = ff;                                                                         //   PROTECT ROBOT FROM
                rightPower = ff;                                                                        //         DAMAGE
            }
        }

        if(getPosition()[0] > POSITIONS[MID_POLE]+20 || getPosition()[1] > POSITIONS[MID_POLE]+20){
            GlobalVars.slowDriveFlag = true;
        }
        else{
            GlobalVars.slowDriveFlag = false;
        }
        //-------------------------------------------------------------------------------------------------------
        write();

        if(GlobalVars.elevatorTelemetry){
            telemetry.addData("Elevator Position: ", getPosition()[0] + " and " + getPosition()[1]);
            telemetry.addData("Elevator Target Position: ", targetCounts);
            telemetry.addData("Elevator PID power: ", this.leftPower + ", " + this.rightPower);
            telemetry.update();
        }
    }

    public void write(){
        this.setPower(this.leftPower, this.rightPower);
    }


    public void setPower(double power1, double power2){
        leftMotor.set(power1);
        rightMotor.set(power2);
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
