package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class turret2 {

    DcMotorEx motor = null;
    OpMode turretOpMode;

    HardwareMap turretHardwareMap;

    //-------------------------------CHANGES HERE-----------------------------------------------
    private int dropPosition = 500;
    private double maxPower = 1;


    public final double GEAR_RATIO = 122.0/44.0;
    private final double CPR = 8192;
    public final double CountsPerDegree = CPR * GEAR_RATIO/360.0;


    //---------------------------------------------------------------------------------
    public turret2(OpMode thisOpMode){
        this.turretOpMode = thisOpMode;

        this.turretHardwareMap = turretOpMode.hardwareMap;

        motor = turretHardwareMap.get(DcMotorEx.class, "turret");
    }

    public turret2(HardwareMap hardwareMap){

        this.turretHardwareMap = hardwareMap;
        motor = turretHardwareMap.get(DcMotorEx.class, "turret");
    }

    private void setTurretCounts(int pos){
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        maxPower = Range.clip(maxPower, -1, 1);
        motor.setPower(maxPower);
    }

    public void setTurretToDropPosition(){
        setTurretCounts(dropPosition);
    }

    public void setTurretDegrees(double degrees){
        degrees = Range.clip(degrees, -360, 360);
        double counts = CountsPerDegree * degrees;
        setTurretCounts((int)counts);
    }

    public double getCurrentDegrees(){
        return getCounts()/CountsPerDegree;
    }

    public int getCounts(){
        return motor.getCurrentPosition();
    }

    public double getCurrent(){
        return motor.getCurrent(CurrentUnit.AMPS);
    }



}
