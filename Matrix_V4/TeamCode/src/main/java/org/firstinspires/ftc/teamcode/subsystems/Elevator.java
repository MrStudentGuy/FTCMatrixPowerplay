package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.GlobalVars;

@Config
public class Elevator extends SubsystemBase {

    MotorEx leftMotor, rightMotor;
    DcMotorEx leftMotorEx, rightMotorEx;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public static double targetCounts = 0;
//    private final double ticks_in_mm = (10.5 * 28)/360;

    private final PIDController leftController;
    private final PIDController rightController;
    public static double Kp = 0.03;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.1;

    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
    public final int[] POSITIONS = {0, 700, 1200, 1840};  //-390
    public final int[] AUTO_POSITION = {50, 150, 230, 330, 379};

    private double leftPower = 0;
    private double rightPower = 0;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftController = new PIDController(Kp, Ki, Kd);
        rightController = new PIDController(Kp, Ki, Kd);
        leftMotor = new MotorEx(hardwareMap, "leftMotor");
        rightMotor = new MotorEx(hardwareMap, "rightMotor");
        leftMotorEx = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotorEx = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);


        leftController.setTolerance(1);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        leftController.setPID(Kp, Ki, Kd);
    }


    @Override
    public void periodic() {
        super.periodic();
//        leftController.setPID(Kp, Ki, Kd);
//        rightController.setPID(Kp, Ki, Kd);
//        double rightCurrentCounts = rightMotor.getCurrentPosition();
//        double leftCurrentCounts = leftMotor.getCurrentPosition();
//        double leftPid = leftController.calculate(leftCurrentCounts, targetCounts);
//        double rightPid = rightController.calculate(rightCurrentCounts, targetCounts);
//
//        double ff = 1 * Kf;
////        telemetry.addData("Position: ", currentCounts);
//        leftPower = leftPid + ff;
//        rightPower = rightPid + ff;

//        if(Math.abs(leftController.getPositionError()) > 50){
//            if(leftMotor.getVelocity() < 100){
//                //stall detected somewhere, stop the controller from acting further
//                leftPower = ff;
//                rightPower = ff;
//            }
//        }

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
        leftMotorEx.setTargetPosition((int)targetCounts);
        rightMotorEx.setTargetPosition((int)targetCounts);
        leftMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorEx.setPower(1);
        rightMotorEx.setPower(1);
    }


    public void setPower(double power1, double power2){
        leftMotor.set(power1);
        rightMotor.set(power2);
    }

    public void extendTo(double pos){
        targetCounts = pos;
    }

    public void extendToHigh(){
            extendTo(POSITIONS[HIGH_POLE]);
    }

    public void extendToMid(){
            extendTo(POSITIONS[MID_POLE]);
    }

    public void extendToLow(){
            extendTo(POSITIONS[LOW_POLE]);
    }

    public void extendToGripping(){
            extendTo(POSITIONS[GRIPPING_POSITION]);
    }

    public void goToCone(int coneNumber){
        extendTo(AUTO_POSITION[coneNumber - 1]);
    }

    public double[] getPosition(){
        return new double[] {leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()};
    }

    public void reset(){
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    public double[] getCurrent(){
        return new double[] {leftMotorEx.getCurrent(CurrentUnit.AMPS), rightMotorEx.getCurrent(CurrentUnit.AMPS)};
    }


}
