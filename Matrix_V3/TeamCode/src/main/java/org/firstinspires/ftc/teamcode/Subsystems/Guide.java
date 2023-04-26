package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Guide extends SubsystemBase {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo guideServo;

    public static double guideOutPosition = 0.24;
    public static double guideIntermediatePosition = 0.8;
    public static double guideInsidePosition = 1;

    public Guide(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        guideServo = hardwareMap.get(Servo.class, "Align");
    }
    @Override
    public void periodic() {

        super.periodic();
    }

    public void inside(){
        guideServo.setPosition(guideInsidePosition);
    }

    public void outside(){
        guideServo.setPosition(guideOutPosition);
    }

    public void intermediate(){
        guideServo.setPosition(guideIntermediatePosition);
    }
}
