package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GlobalVars;

import java.util.function.IntSupplier;

public class EndEffector extends SubsystemBase {

    private static final double gripperOpenPosition = 0.6;
    private static final double gripperClosePosition = 0;
    private static final double gripperBeaconPosition = 0.42;
    private static final double TopPosition = 0.6;
    private static final double InitPosition = 0;
    private static final double GrippingPosition = 0.45;
    private static final double TopAutoPosition = 1;
    public static String gripperState = "CLOSED";
    public static int gripperStateID = 0;
    public static String wristState = "INIT";
    public static int wristStateID = 0;
    public IntSupplier gripperStateSupplier = new IntSupplier() {
        @Override
        public int getAsInt() {
            return gripperStateID;
        }
    };
    public IntSupplier wristStateSupplier = new IntSupplier() {
        @Override
        public int getAsInt() {
            return wristStateID;
        }
    };
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ServoEx WristServo, ClawServo;

    public EndEffector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        WristServo = new SimpleServo(hardwareMap, "Wrist", 0, 180);
        ClawServo = new SimpleServo(hardwareMap, "Gripper", 0, 180);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (GlobalVars.endEffectorTelemetry) {
            telemetry.addData("Claw Position: ", ClawServo.getPosition());
            telemetry.addData("Claw Degrees: ", ClawServo.getAngle(AngleUnit.DEGREES));
            telemetry.addData("Wrist Position: ", WristServo.getPosition());
            telemetry.addData("Wrist Degrees: ", WristServo.getAngle(AngleUnit.DEGREES));
        }
    }

    public void openGripper() {
        gripperState = "OPEN";
        gripperStateID = 1;
        ClawServo.setPosition(gripperOpenPosition);
    }

    public void closeGripper() {
        gripperState = "CLOSED";
        gripperStateID = 0;
        ClawServo.setPosition(gripperClosePosition);
    }

    public void gripBeacon() {
        gripperStateID = 1;
        gripperState = "OPEN";
        ClawServo.setPosition(gripperBeaconPosition);
    }

    public void goTop() {
        wristStateID = 2;
        wristState = "TOP";
        WristServo.setPosition(TopPosition);
    }

    public void goInit() {
        wristStateID = 0;
        wristState = "INIT";
        WristServo.setPosition(InitPosition);
    }

    public void goGripping() {
        wristStateID = 1;
        wristState = "GRIPPING";
        WristServo.setPosition(GrippingPosition);
    }

    public void goAutoTop() {
        wristStateID = 1;
        wristState = "TOP";
        WristServo.setPosition(TopAutoPosition);
    }

    public void setPosition(double pos) {
        ClawServo.setPosition(pos); //auto
    }


}
