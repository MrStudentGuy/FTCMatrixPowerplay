package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift {

    ElapsedTime timer = new ElapsedTime();

    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 2.26, 0.34, 0.01);

////    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile()
//    public static double left_integralSum = 0;
//    public static double leftMotorP = 0;
//    public static double leftMotorI = 0;
//    public static double leftMotorD = 0;
//    public static double leftMotorF = 0;
//    public static double left_prevError = 0;
//
//    public static double rightMotorP = 0;
//    public static double rightMotorI = 0;
//    public static double rightMotorD = 0;
//    public static double rightMotorF = 0;
//    public static double right_prevError = 0;

    DcMotorEx leftMotor, rightMotor;
    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
    public final int SAFE_POSITION = 0;
    public final int[] POSITIONS = {-340, 672, 1510, 2200};
    public final int[] AUTO_POSITION = {-200, -42, -10, 46, 100};
//    {-420, -374, -334, -160, -35};

    int liftPosition = 0;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setTargetPositionTolerance(10);
        rightMotor.setTargetPositionTolerance(10);
        PIDFCoefficients leftCoeff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients rightCoeff = rightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotorP = leftCoeff.p;
//        leftMotorI = leftCoeff.i;
//        leftMotorD = leftCoeff.d;
//        leftMotorF = leftCoeff.f;
//
//        rightMotorP = rightCoeff.p;
//        rightMotorI = rightCoeff.i;
//        rightMotorD = rightCoeff.d;
//        rightMotorF = rightCoeff.f;
    }

    public void applyFeedforward(int position, int vel, int acc){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(feedforward.calculate(vel, acc));
    }

    public void extendTo(int position, double power){
//        PIDFCoefficients left_coeffs = new PIDFCoefficients( leftMotorP, leftMotorI, leftMotorD, leftMotorF);
//        PIDFCoefficients right_coeffs = new PIDFCoefficients(rightMotorP, rightMotorI, rightMotorD, rightMotorF);
//        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, left_coeffs);
//        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, right_coeffs);

        leftMotor.setTargetPosition(position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(power);

        rightMotor.setTargetPosition(position);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(power);
    }

    //MAX is 2799 eps

    public void extendTousingVelo(int position, double velocity){
        leftMotor.setTargetPosition(position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setVelocity(velocity);

        rightMotor.setTargetPosition(position);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setVelocity(velocity);
    }

    public void extendToHighPole(){
        liftPosition = 3;
//        extendTo(POSITIONS[HIGH_POLE], 1);
extendTousingVelo(POSITIONS[HIGH_POLE], 2000);
    }

    public void extendToLowPole(){
        double velocity = 2000;
        if(liftPosition > 1){
            velocity = 1000;
        }
        liftPosition = 1;
        extendTousingVelo(POSITIONS[LOW_POLE], velocity);
        //        double power = 1;
//        if(liftPosition > 1){
//            power = 0.8;
//        }
//        liftPosition = 1;
//        extendTo(POSITIONS[LOW_POLE], power);
    }

    public void extendToMidPole(){
        double velocity = 2000;
        if(liftPosition > 2){
            velocity = 1000;
        }
        liftPosition = 2;
        extendTousingVelo(POSITIONS[MID_POLE], velocity);
//        double power = 1;
//        if(liftPosition > 2){
//            power = 0.8;
//        }
//        liftPosition = 2;
//        extendTo(POSITIONS[MID_POLE], power);
    }


    public void extendToGrippingPosition(){
        liftPosition = 0;
        extendTo(POSITIONS[GRIPPING_POSITION], 1000);
//        liftPosition = 0;
//        extendTo(POSITIONS[GRIPPING_POSITION], 0.6);
    }

//    private double leftPIDF(double current, double target){
//        double error = target - current;
//        double pError = error;
//        double dError = error - left_prevError / ;
//        double Ierror = error + left_prevError;
//
//        left_prevError = error;
//
//        return pError * leftMotorP + dError * leftMotorI + Ierror * leftMotorD + leftMotorF * ;
//    }

    public double[] getPosition(){
        return new double[]{leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()};
    }

    public void reset(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = Range.clip(power, -1, 1);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public double[] getCurrent(){
        return new double[]{leftMotor.getCurrent(CurrentUnit.MILLIAMPS), rightMotor.getCurrent(CurrentUnit.MILLIAMPS)};
    }
}
