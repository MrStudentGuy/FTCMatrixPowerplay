package org.firstinspires.ftc.teamcode;

public class GlobalVars {

    public static boolean sliderTelemetry = false;
    public static boolean elevatorTelemetry = false;
    public static boolean turretTelemetry = false;
    public static boolean endEffectorTelemetry = false;
    public static boolean driveTelemetry = false;

    public enum runMode{
        AUTO, TELEOP
    };

    public static runMode currentOpModeType = runMode.TELEOP;

}
