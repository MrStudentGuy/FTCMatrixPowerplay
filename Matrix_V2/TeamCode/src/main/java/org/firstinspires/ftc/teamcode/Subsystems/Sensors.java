package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem Class containing all Sensors used on Team Matrix's PowerPlay Robot
 */
public class Sensors {


    private static Telemetry localTelemetry;

    private RevTouchSensor AlignSensor;
    /**
     * Create a new Subsystem of Sensors.
     * @param hardwareMap Pass in the hardwaremap used by the opMode
     * @param telemetry Pass in telemetry to display values
     */
    public Sensors(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
        AlignSensor = hardwareMap.get(RevTouchSensor.class, "alignSensor");
//        GripperSensor  =hardwareMap.get(RevTouchSensor.class, "gripperSensor");
//        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");
//        poleSensor = hardwareMap.get(Rev2mDistanceSensor.class, "poleSensor");
    }

    public boolean read(){
        return AlignSensor.isPressed();
    }

}
