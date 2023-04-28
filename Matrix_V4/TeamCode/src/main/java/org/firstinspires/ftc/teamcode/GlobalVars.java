package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GlobalVars {

    public static boolean sliderTelemetry = false;
    public static boolean elevatorTelemetry = false;
    public static boolean turretTelemetry = false;
    public static boolean endEffectorTelemetry = false;
    public static boolean driveTelemetry = true;

    public static boolean traditionalFlag = false;
    public static boolean slowDriveFlag = false;

    public static double OpModeLoopTime = 0;

    public enum runMode{
        AUTO, TELEOP
    };

    public static runMode currentOpModeType = runMode.TELEOP;

}
