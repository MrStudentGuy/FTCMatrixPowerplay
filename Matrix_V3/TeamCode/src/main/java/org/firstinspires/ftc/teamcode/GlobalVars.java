package org.firstinspires.ftc.teamcode;

public class GlobalVars {

    public static boolean sliderTelemetry = false;
    public static boolean elevatorTelemetry = false;
    public static boolean turretTelemetry = true;
    public static boolean endEffectorTelemetry = false;
    public static boolean driveTelemetry = false;
    public static boolean traditionalFlag = false;

    public enum runMode{
        AUTO, TELEOP
    };

    public static runMode currentOpModeType = runMode.TELEOP;

}
