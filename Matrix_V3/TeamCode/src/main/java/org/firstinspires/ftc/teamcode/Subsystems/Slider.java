package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalVars;

public class Slider extends SubsystemBase {
    private ServoEx sliderServo;
    HardwareMap hardwareMap;;
    Telemetry telemetry;

    private final double outPos = 0.45;
    private final double inPos = 1;
    public Slider(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        sliderServo = new SimpleServo(hardwareMap, "Slider", 0, 180, AngleUnit.DEGREES);
    }


    @Override
    public void periodic() {
        super.periodic();
        if(GlobalVars.sliderTelemetry){
            telemetry.addData("Slider Servo Position: ", getPosition());
            telemetry.addData("Slider Servo Angle: ", getAngle());
        }
    }

    public double getPosition(){
        return sliderServo.getPosition();
    }

    public double getAngle(){
        return sliderServo.getAngle(AngleUnit.DEGREES);
    }

    public void setPosition(double pos){
        sliderServo.setPosition(pos);
    }

    public void goOutside(){
        sliderServo.setPosition(outPos);
    }

    public void goInside(){
        sliderServo.setPosition(inPos);
    }
}
