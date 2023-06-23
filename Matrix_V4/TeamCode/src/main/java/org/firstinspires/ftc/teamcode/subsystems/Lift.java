package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Subsystem class containing lift functions and hardware mappings.
 */
@Config
public class Lift extends Subsystem{

//    ElapsedTime timer = new ElapsedTime();

    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 2.26, 0.34, 0.01);



    DcMotorEx leftMotor, rightMotor;
    public final int GRIPPING_POSITION = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
    public final int SAFE_POSITION = 0;
    private final double GEAR_RATIO = 5.23;
    private final double CPR = 28;
    private final double CIRCUMFERENCE = 30 * Math.PI;
    private final double TICKS_PER_MM = CPR * GEAR_RATIO/CIRCUMFERENCE;
    private final double MM_PER_TICK = GEAR_RATIO * CIRCUMFERENCE / CPR;
    public final int[] POSITIONS = {7, 440, 850, 1265};  //-390
    public final int[] AUTO_POSITION = {5, 72, 123, 187, 238};

    public final int[] POSITIONS_AUTO = {-278, 320, 930, 1500};
//    {-420, -374, -334, -160, -35};

    int liftPosition = 0;

    /**
     * Creates a new Lift subsystem, setting the directions of the motors in accordance with the physical mechanism. A Tolerance of 10 counts is added in case of uneven extension.
     * @param hardwareMap The hardware map being used by the current opMode
     * @param telemetry Telemetry needs to be passed in to display currents and positions when required.
     */
    public Lift(HardwareMap hardwareMap, Telemetry telemetry){

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setTargetPositionTolerance(5);
        rightMotor.setTargetPositionTolerance(5);
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


    @Deprecated
    public void applyFeedforward(int position, int vel, int acc){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(feedforward.calculate(vel, acc));
    }

    /**
     * Extend to a given position while limiting to a certain amount of power.
     * @param position The position the lift should extend to (in encoder counts)
     * @param power The maximum power the lift can use to reach the required position
     */

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

    /**
     * Extend to a given position while limiting to a certain velocity(in encoder counts per second).
     * @param position The position the lift should extend to (in encoder counts)
     * @param velocity The maximum velocity the lift can use to reach the required position (in encoder counts/second).
     */
    public void extendTousingVelo(int position, double velocity){
        leftMotor.setTargetPosition(position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setVelocity(velocity);

        rightMotor.setTargetPosition(position);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setVelocity(velocity);
    }

    /**
     * Quick method of setting the lift to the Highest Junction
     */
    public void extendToHighPole(){
        liftPosition = 3;
//        extendTo(POSITIONS[HIGH_POLE], 1);
extendTousingVelo(POSITIONS[HIGH_POLE], 2000);
    }

    /**
     * Quick method of setting the lift to the Low Junction
     */
    public void extendToLowPole(){
        double velocity = 2000; //2800 encoder counts per second is max, if battery voltage drops it cant reach that 
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

    /**
     * Quick method of setting the lift to the Mid Junction
     */

    public void extendToMidPole(){
        double velocity = 2000;
        if(liftPosition > 2){
            velocity = 1000; //pulleys cant retract fast enough
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


    /**
     * Quick method of setting the lift to the Gripping position
     */
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

    /**
     * Get the position of the motors as an array
     * @return The positions of the motors as an array of two elements. 0-Left Motor, 1-Right Motor
     */

    public double[] getPosition(){
        return new double[]{leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()};
    }

    /**
     * Reset encoder counts for both motors
     */

    public void reset(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     * Use raw power to control the lift.
     * @param power Power to drive the lifts at (-1 to 1)
     */

    public void setPower(double power){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = Range.clip(power, -1, 1);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /**
     * Get the current being consumed by each motor as an array
     * @return The current consumption in Milliamperes of the motors as an array of two elements. 0-Left Motor, 1-Right Motor
     */

    public double[] getCurrent(){
        return new double[]{leftMotor.getCurrent(CurrentUnit.MILLIAMPS), rightMotor.getCurrent(CurrentUnit.MILLIAMPS)};
    }


    public double[] getTarget(){
        return new double[]{leftMotor.getTargetPosition(), rightMotor.getTargetPosition()};
    }
}
