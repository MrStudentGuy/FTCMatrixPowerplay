package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sticks extends SubsystemBase {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo leftStick, rightStick;

    public static double stickOutPosition = 1;
    public static double stickInPosition = 0;

    private double currentStickPosition = stickInPosition;

    public static double prevStickPosition = 0;
    public Sticks(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftStick = hardwareMap.get(Servo.class, "leftStick");
        rightStick = hardwareMap.get(Servo.class, "rightStick");
    }

    @Override
    public void periodic() {
        super.periodic();
        if(currentStickPosition != prevStickPosition){
            leftStick.setPosition(currentStickPosition);
        }
    }

    public void stickIn(){
        currentStickPosition = stickInPosition;
    }

    public void stickOut(){
        currentStickPosition = stickOutPosition;
    }
}
